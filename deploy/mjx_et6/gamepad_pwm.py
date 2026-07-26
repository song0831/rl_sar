#!/usr/bin/env python3
"""Drive two RC PWM channels from a Linux gamepad event device."""

from __future__ import annotations

import argparse
import copy
import json
import logging
import math
import os
import signal
import statistics
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from select import select
from typing import Any


LOG = logging.getLogger("gamepad-pwm")

EV_KEY = 0x01
EV_ABS = 0x03

DEFAULT_CONFIG: dict[str, Any] = {
    "input": {
        "device": "auto",
        "name_contains": [
            "G5",
            "MACHENIKE",
            "Xbox",
            "Wireless Controller",
            "Gamepad",
            "Controller",
        ],
        "grab": True,
        "reconnect_s": 1.0,
    },
    "mapping": {
        "steering_axis": ["ABS_X"],
        "throttle_axis": ["ABS_GAS", "ABS_RZ"],
        "brake_axis": ["ABS_BRAKE", "ABS_Z"],
        "throttle_button": [],
        "brake_button": ["BTN_TL", "BTN_TL2"],
        "steering_invert": True,
        "throttle_invert": False,
        "brake_invert": False,
        "arm_button": "BTN_SOUTH",
        "disarm_button": "BTN_EAST",
        "neutral_button": "BTN_START",
        "cruise_button": "BTN_X",
        "steering_limit_button": "BTN_Y",
        "steering_neutral_button": "BTN_TR",
        "deadman_button": None,
        "exit_button": None,
    },
    "control": {
        "frequency_hz": 50.0,
        "min_pulse_ms": 1.0,
        "neutral_pulse_ms": 1.5,
        "max_pulse_ms": 2.0,
        "steering_deadzone": 0.05,
        "throttle_deadzone": 0.04,
        "steering_limit": 1.0,
        "throttle_limit": 1.0,
        "brake_limit": 1.0,
        "steering_expo": 0.20,
        "throttle_expo": 0.10,
        "cruise_steps": 10,
        "cruise_max": 1.0,
        "steering_limit_levels": [0.40, 0.55, 0.70, 0.85, 1.0],
        "steering_neutral_min_ms": 1.40,
        "steering_neutral_max_ms": 1.60,
        "steering_neutral_step_ms": 0.025,
        "max_steering_slew_per_s": 6.0,
        "max_throttle_slew_per_s": 4.0,
        "require_arm": True,
        "armed_on_start": False,
        "require_deadman": False,
    },
    "pwm": {
        "backend": "sysfs",
        "steering": "32e0000.pwm:0",
        "throttle": "32c0000.pwm:0",
        "steering_output_inverted": True,
        "throttle_output_inverted": True,
        "dry_run": False,
        "disable_on_exit": False,
    },
    "gyro_assist": {
        "enabled": False,
        "required_for_motion": False,
        "imu_url": "http://127.0.0.1:8080/api/data",
        "wheel_url": "http://127.0.0.1:8081/api/data",
        "request_timeout_s": 0.08,
        "max_data_age_s": 0.5,
        "poll_rate_hz": 60.0,
        "startup_calibration_samples": 30,
        "startup_max_abs_gyro_radps": 0.15,
        "startup_max_wheel_rpm": 20.0,
        "imu_mount_roll_rad": 0.0,
        "imu_mount_pitch_rad": 0.0,
        "imu_gyro_z_sign": 1.0,
        "wheel_rpm_key": "fast_rpm",
        "front_wheel_names": ["front_left", "front_right"],
        "wheel_radius_m": 0.05,
        "wheelbase_m": 0.375,
        "steering_curve_commands": [],
        "steering_curve_angles_rad": [],
        "max_steering_angle_rad": 0.45,
        "cutoff_hz": 8.0,
        "kp": 0.08,
        "max_correction": 0.12,
        "error_deadband_radps": 0.10,
        "activation_speed_mps": 0.8,
        "activation_blend_mps": 0.4,
        "correction_slew_per_s": 4.0,
        "max_target_yaw_rate_radps": 6.0,
    },
    "status": {
        "print_rate_hz": 1.0,
    },
}

BOARD_PWM_ALIASES = {
    # Jetson Orin Nano / Orin NX 40-pin header PWM-capable pins.
    "board15": "3280000.pwm:0",
    "pin15": "3280000.pwm:0",
    "board32": "32e0000.pwm:0",
    "pin32": "32e0000.pwm:0",
    "board33": "32c0000.pwm:0",
    "pin33": "32c0000.pwm:0",
}


class ConfigError(RuntimeError):
    pass


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path: str | None) -> dict[str, Any]:
    if not path:
        return copy.deepcopy(DEFAULT_CONFIG)
    config_path = Path(path)
    if not config_path.exists():
        raise ConfigError(f"Config file not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as handle:
        user_config = json.load(handle)
    if not isinstance(user_config, dict):
        raise ConfigError("Config root must be a JSON object")
    return deep_merge(DEFAULT_CONFIG, user_config)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    deadzone = clamp(deadzone, 0.0, 0.95)
    if abs(value) <= deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(clamp(scaled, 0.0, 1.0), value)


def apply_trigger_deadzone(value: float, deadzone: float) -> float:
    deadzone = clamp(deadzone, 0.0, 0.95)
    if value <= deadzone:
        return 0.0
    return clamp((value - deadzone) / (1.0 - deadzone), 0.0, 1.0)


def apply_expo(value: float, expo: float) -> float:
    expo = clamp(expo, 0.0, 1.0)
    return (1.0 - expo) * value + expo * value * value * value


def slew(current: float, target: float, rate_per_s: float, dt: float) -> float:
    if rate_per_s <= 0:
        return target
    step = rate_per_s * max(0.0, dt)
    return current + clamp(target - current, -step, step)


def signed_to_pulse(value: float, min_ms: float, neutral_ms: float, max_ms: float) -> float:
    value = clamp(value, -1.0, 1.0)
    if value >= 0:
        return neutral_ms + value * (max_ms - neutral_ms)
    return neutral_ms + value * (neutral_ms - min_ms)


def interpolate_curve(value: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
    """Linearly interpolate a monotonic calibration curve without extra dependencies."""
    if not xs:
        raise ValueError("interpolation curve is empty")
    if value <= xs[0]:
        return ys[0]
    if value >= xs[-1]:
        return ys[-1]
    for upper in range(1, len(xs)):
        if value <= xs[upper]:
            lower = upper - 1
            span = xs[upper] - xs[lower]
            if span <= 0.0:
                return ys[upper]
            fraction = (value - xs[lower]) / span
            return ys[lower] + fraction * (ys[upper] - ys[lower])
    return ys[-1]


def import_evdev() -> Any:
    try:
        import evdev  # type: ignore
    except ImportError as exc:
        raise ConfigError(
            "python3-evdev is required on Jetson. Install it with: "
            "sudo apt install -y python3-evdev"
        ) from exc
    return evdev


def event_code(evdev: Any, name_or_code: str | int | None, expected_type: int) -> int | None:
    if name_or_code is None:
        return None
    if isinstance(name_or_code, int):
        return name_or_code
    text = str(name_or_code).strip()
    if not text:
        return None
    if text.isdigit():
        return int(text)
    code = evdev.ecodes.ecodes.get(text)
    if code is None:
        raise ConfigError(f"Unknown input code name: {text}")
    if expected_type == EV_ABS and not text.startswith("ABS_"):
        raise ConfigError(f"{text} is not an absolute axis name")
    if expected_type == EV_KEY and not (text.startswith("BTN_") or text.startswith("KEY_")):
        raise ConfigError(f"{text} is not a key/button name")
    return int(code)


def event_codes(evdev: Any, values: Any, expected_type: int) -> list[int]:
    if values is None:
        return []
    if isinstance(values, (str, int)):
        values = [values]
    return [
        code
        for value in values
        if (code := event_code(evdev, value, expected_type)) is not None
    ]


def code_name(evdev: Any, ev_type: int, code: int) -> str:
    name = evdev.ecodes.bytype.get(ev_type, {}).get(code)
    if isinstance(name, (list, tuple)):
        return "/".join(name)
    return str(name) if name else str(code)


def axis_label(evdev: Any, code: int) -> str:
    return code_name(evdev, EV_ABS, code)


def key_label(evdev: Any, code: int) -> str:
    return code_name(evdev, EV_KEY, code)


def short_input_name(name: str) -> str:
    preferred = ("BTN_A", "BTN_B", "BTN_X", "BTN_Y", "BTN_START", "BTN_SELECT", "BTN_MODE")
    parts = name.split("/")
    for wanted in preferred:
        if wanted in parts:
            return wanted.replace("BTN_", "")
    cleaned = parts[0].replace("BTN_", "").replace("KEY_", "").replace("ABS_", "")
    return cleaned[:14]


def cap_code(entry: Any) -> int:
    return int(entry[0] if isinstance(entry, tuple) else entry)


def cap_absinfo(entry: Any) -> Any | None:
    if isinstance(entry, tuple) and len(entry) >= 2:
        return entry[1]
    return None


DASHBOARD_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Gamepad PWM</title>
<style>
:root {
  color-scheme: dark;
  --bg: #0b1014;
  --panel: #111a21;
  --panel2: #16222b;
  --line: #2b3e49;
  --text: #ecf5f7;
  --muted: #91a5ad;
  --cyan: #53d2ff;
  --green: #62d98f;
  --amber: #f5b84b;
  --red: #ff6464;
}
* { box-sizing: border-box; }
body {
  margin: 0; min-height: 100vh; background: var(--bg); color: var(--text);
  font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
.wrap { width: min(1280px, 94vw); margin: 0 auto; padding: 24px 0; }
header { display: flex; justify-content: space-between; gap: 18px; align-items: center; margin-bottom: 16px; }
h1 { margin: 0; font-size: 30px; font-weight: 750; }
.sub { color: var(--muted); margin-top: 3px; }
.status { display: flex; align-items: center; gap: 10px; color: var(--muted); }
.dot { width: 10px; height: 10px; border-radius: 50%; background: var(--red); box-shadow: 0 0 14px var(--red); }
.dot.online { background: var(--green); box-shadow: 0 0 14px var(--green); }
.grid { display: grid; grid-template-columns: repeat(12, 1fr); gap: 12px; }
.panel { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 14px; min-width: 0; }
.span-3 { grid-column: span 3; }
.span-4 { grid-column: span 4; }
.span-5 { grid-column: span 5; }
.span-6 { grid-column: span 6; }
.span-7 { grid-column: span 7; }
.span-12 { grid-column: span 12; }
h2 { margin: 0 0 10px; color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: .12em; }
.big { font: 700 28px/1.1 ui-monospace, SFMono-Regular, Consolas, monospace; }
.metric { display: grid; grid-template-columns: repeat(3, minmax(0,1fr)); gap: 9px; }
.cell { background: var(--panel2); border: 1px solid rgba(255,255,255,.05); border-radius: 6px; padding: 10px; min-width: 0; }
.label { color: var(--muted); font-size: 11px; text-transform: uppercase; letter-spacing: .09em; }
.value { margin-top: 4px; font: 650 19px/1.15 ui-monospace, SFMono-Regular, Consolas, monospace; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.armed { color: var(--green); }
.locked { color: var(--red); }
.bar { position: relative; height: 18px; background: #071014; border: 1px solid var(--line); border-radius: 5px; overflow: hidden; margin-top: 8px; }
.bar .mid { position: absolute; inset-block: 0; left: 50%; width: 1px; background: rgba(255,255,255,.35); }
.fill { position: absolute; top: 0; bottom: 0; left: 50%; width: 0; background: var(--cyan); }
.fill.neg { left: auto; right: 50%; background: var(--amber); }
.row { display: grid; grid-template-columns: 120px 1fr 92px; gap: 8px; align-items: center; padding: 6px 0; border-bottom: 1px solid rgba(255,255,255,.06); }
.row:last-child { border-bottom: 0; }
.name { color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.raw { text-align: right; font: 600 13px ui-monospace, SFMono-Regular, Consolas, monospace; }
.buttons { display: flex; flex-wrap: wrap; gap: 7px; }
.btn {
  min-width: 58px; padding: 7px 9px; border-radius: 6px; border: 1px solid var(--line);
  background: var(--panel2); color: var(--muted); text-align: center; font: 650 12px ui-monospace, SFMono-Regular, Consolas, monospace;
}
.btn.on { color: #06120b; background: var(--green); border-color: var(--green); }
.conn { display: grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap: 9px; }
@media (max-width: 1000px) {
  .span-3,.span-4,.span-5,.span-6,.span-7 { grid-column: span 12; }
  .conn { grid-template-columns: repeat(2, minmax(0,1fr)); }
}
@media (max-width: 620px) {
  header { align-items: flex-start; flex-direction: column; }
  .metric,.conn { grid-template-columns: 1fr; }
  .row { grid-template-columns: 88px 1fr 76px; }
}
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div>
      <h1>Gamepad PWM</h1>
      <div class="sub">Jetson steering and throttle signal monitor</div>
    </div>
    <div class="status"><span id="dot" class="dot"></span><span id="status">Connecting...</span></div>
  </header>
  <main class="grid">
    <section class="panel span-3">
      <h2>Safety</h2>
      <div id="armed" class="big locked">LOCKED</div>
      <div class="sub" id="lastEvent">--</div>
    </section>
    <section class="panel span-3">
      <h2>Steering PWM</h2>
      <div class="big" id="steerMs">1.500 ms</div>
      <div class="bar"><div class="mid"></div><div id="steerFill" class="fill"></div></div>
    </section>
    <section class="panel span-3">
      <h2>Throttle PWM</h2>
      <div class="big" id="driveMs">1.500 ms</div>
      <div class="bar"><div class="mid"></div><div id="driveFill" class="fill"></div></div>
    </section>
    <section class="panel span-3">
      <h2>Raw Pedals</h2>
      <div class="metric">
        <div class="cell"><div class="label">RT</div><div id="rawThrottle" class="value">0.00</div></div>
        <div class="cell"><div class="label">LT</div><div id="rawBrake" class="value">0.00</div></div>
        <div class="cell"><div class="label">Drive</div><div id="rawDrive" class="value">0.00</div></div>
        <div class="cell"><div class="label">Cruise</div><div id="cruise" class="value">0/10</div></div>
      </div>
    </section>
    <section class="panel span-5">
      <h2>Command</h2>
      <div class="metric">
        <div class="cell"><div class="label">Steering</div><div id="steerCmd" class="value">0.00</div></div>
        <div class="cell"><div class="label">Drive</div><div id="driveCmd" class="value">0.00</div></div>
        <div class="cell"><div class="label">Steer Limit</div><div id="steerLimit" class="value">100%</div></div>
        <div class="cell"><div class="label">Steer Center</div><div id="steerCenter" class="value">1.500 ms</div></div>
        <div class="cell"><div class="label">Rate</div><div id="rate" class="value">0 Hz</div></div>
      </div>
    </section>
    <section class="panel span-7">
      <h2>Connection</h2>
      <div class="conn">
        <div class="cell"><div class="label">Device</div><div id="device" class="value">--</div></div>
        <div class="cell"><div class="label">Path</div><div id="path" class="value">--</div></div>
        <div class="cell"><div class="label">Period</div><div id="period" class="value">20.000 ms</div></div>
        <div class="cell"><div class="label">Updated</div><div id="age" class="value">--</div></div>
      </div>
    </section>
    <section class="panel span-6">
      <h2>Axes</h2>
      <div id="axes"></div>
    </section>
    <section class="panel span-6">
      <h2>Buttons</h2>
      <div id="buttons" class="buttons"></div>
    </section>
  </main>
</div>
<script>
const $ = id => document.getElementById(id);
const fmt = (v, d=2) => Number.isFinite(Number(v)) ? Number(v).toFixed(d) : '--';

function setSignedFill(id, value) {
  const el = $(id);
  const v = Math.max(-1, Math.min(1, Number(value) || 0));
  el.classList.toggle('neg', v < 0);
  el.style.width = `${Math.abs(v) * 50}%`;
}

function axisRow(axis) {
  const v = Math.max(-1, Math.min(1, Number(axis.signed) || 0));
  const width = Math.abs(v) * 50;
  const cls = v < 0 ? 'fill neg' : 'fill';
  return `<div class="row">
    <div class="name" title="${axis.name}">${axis.name}</div>
    <div class="bar"><div class="mid"></div><div class="${cls}" style="width:${width}%"></div></div>
    <div class="raw">${axis.raw}</div>
  </div>`;
}

function render(data) {
  $('dot').classList.toggle('online', !!data.online);
  $('status').textContent = data.message || (data.online ? 'Streaming' : 'Offline');
  $('armed').textContent = data.armed ? 'ARMED' : 'LOCKED';
  $('armed').className = `big ${data.armed ? 'armed' : 'locked'}`;
  $('lastEvent').textContent = data.last_button_event || '--';

  $('steerMs').textContent = `${fmt(data.steering_ms, 3)} ms`;
  $('driveMs').textContent = `${fmt(data.throttle_ms, 3)} ms`;
  $('steerCmd').textContent = fmt(data.steering, 2);
  $('driveCmd').textContent = fmt(data.drive, 2);
  $('steerLimit').textContent = `${fmt((data.steering_limit || 0) * 100, 0)}%`;
  $('steerCenter').textContent = `${fmt(data.steering_neutral_ms, 3)} ms`;
  $('rawThrottle').textContent = fmt(data.raw_throttle, 2);
  $('rawBrake').textContent = fmt(data.raw_brake, 2);
  $('rawDrive').textContent = fmt(data.raw_drive, 2);
  $('cruise').textContent = `${data.cruise_level || 0}/${data.cruise_steps || 10}`;
  $('rate').textContent = `${fmt(data.update_rate_hz, 1)} Hz`;
  $('device').textContent = data.device_name || '--';
  $('path').textContent = data.device_path || '--';
  $('period').textContent = `${fmt(data.period_ms, 3)} ms`;
  $('age').textContent = data.data_age_ms == null ? '--' : `${fmt(data.data_age_ms, 0)} ms`;
  setSignedFill('steerFill', data.steering);
  setSignedFill('driveFill', data.drive);

  $('axes').innerHTML = (data.axes || []).map(axisRow).join('') || '<div class="sub">No axes</div>';
  $('buttons').innerHTML = (data.buttons || []).map(button =>
    `<div class="btn ${button.pressed ? 'on' : ''}" title="${button.name}">${button.short_name}</div>`
  ).join('') || '<div class="sub">No buttons</div>';
}

async function poll() {
  const response = await fetch('/api/status', {cache: 'no-store'});
  render(await response.json());
}

const events = new EventSource('/events');
events.onmessage = event => render(JSON.parse(event.data));
events.onerror = () => {
  $('dot').classList.remove('online');
  $('status').textContent = 'Dashboard connection lost; polling';
  setTimeout(() => poll().catch(() => {}), 500);
};
</script>
</body>
</html>
"""


class ControllerStatus:
    def __init__(self, config: dict[str, Any]) -> None:
        control = config["control"]
        self.lock = threading.Lock()
        self.rate_start = time.monotonic()
        self.rate_frames = 0
        self.data: dict[str, Any] = {
            "online": False,
            "message": "Waiting for gamepad",
            "device_name": None,
            "device_path": None,
            "armed": False,
            "steering": 0.0,
            "drive": 0.0,
            "raw_steering": 0.0,
            "raw_throttle": 0.0,
            "raw_brake": 0.0,
            "raw_drive": 0.0,
            "steering_limit": float(control["steering_limit"]),
            "steering_limit_level": 0,
            "steering_limit_steps": len(control.get("steering_limit_levels", [])) or 1,
            "steering_neutral_ms": float(control["neutral_pulse_ms"]),
            "steering_neutral_level": 0,
            "steering_neutral_steps": 1,
            "manual_drive": 0.0,
            "cruise_drive": 0.0,
            "cruise_level": 0,
            "cruise_steps": int(control.get("cruise_steps", 10)),
            "steering_ms": float(control["neutral_pulse_ms"]),
            "throttle_ms": float(control["neutral_pulse_ms"]),
            "neutral_ms": float(control["neutral_pulse_ms"]),
            "period_ms": 1000.0 / float(control["frequency_hz"]),
            "last_button_event": "",
            "last_update": 0.0,
            "update_rate_hz": 0.0,
            "axes": [],
            "buttons": [],
            "gyro_assist": {
                "enabled": False,
                "healthy": True,
                "message": "disabled",
            },
        }

    def set_connection(
        self,
        online: bool,
        message: str,
        device_name: str | None = None,
        device_path: str | None = None,
    ) -> None:
        with self.lock:
            self.data["online"] = online
            self.data["message"] = message
            self.data["device_name"] = device_name
            self.data["device_path"] = device_path
            self.data["last_update"] = time.time()

    def update(
        self,
        state: "GamepadState",
        steering: float,
        drive: float,
        steering_ms: float,
        throttle_ms: float,
        gyro_assist: "GyroAssistSample | None" = None,
    ) -> None:
        controls = state.control_values()
        now = time.monotonic()
        with self.lock:
            self.data.update(
                {
                    "online": True,
                    "message": (
                        "Streaming"
                        if gyro_assist is None or not gyro_assist.enabled or gyro_assist.healthy
                        else f"Gyro assist: {gyro_assist.message}"
                    ),
                    "armed": state.armed,
                    "steering": steering,
                    "drive": drive,
                    "raw_steering": controls["steering"],
                    "raw_throttle": controls["throttle"],
                    "raw_brake": controls["brake"],
                    "raw_drive": controls["drive"],
                    "steering_limit": controls["steering_limit"],
                    "steering_limit_level": controls["steering_limit_level"],
                    "steering_limit_steps": controls["steering_limit_steps"],
                    "steering_neutral_ms": controls["steering_neutral_ms"],
                    "steering_neutral_level": controls["steering_neutral_level"],
                    "steering_neutral_steps": controls["steering_neutral_steps"],
                    "manual_drive": controls["manual_drive"],
                    "cruise_drive": controls["cruise_drive"],
                    "cruise_level": controls["cruise_level"],
                    "cruise_steps": controls["cruise_steps"],
                    "steering_ms": steering_ms,
                    "throttle_ms": throttle_ms,
                    "last_button_event": state.last_button_event,
                    "last_update": time.time(),
                    "axes": state.axis_snapshot(),
                    "buttons": state.button_snapshot(),
                    "gyro_assist": (
                        {
                            "enabled": gyro_assist.enabled,
                            "healthy": gyro_assist.healthy,
                            "calibrated": gyro_assist.calibrated,
                            "message": gyro_assist.message,
                            "driver_steering": gyro_assist.driver_steering,
                            "final_steering": gyro_assist.final_steering,
                            "front_speed_mps": gyro_assist.front_speed_mps,
                            "measured_yaw_rate_radps": gyro_assist.measured_yaw_rate_radps,
                            "filtered_yaw_rate_radps": gyro_assist.filtered_yaw_rate_radps,
                            "target_yaw_rate_radps": gyro_assist.target_yaw_rate_radps,
                            "yaw_rate_error_radps": gyro_assist.yaw_rate_error_radps,
                            "correction": gyro_assist.correction,
                            "sensor_age_ms": gyro_assist.sensor_age_ms,
                        }
                        if gyro_assist is not None
                        else {"enabled": False, "healthy": True, "message": "disabled"}
                    ),
                }
            )
            self.rate_frames += 1
            elapsed = now - self.rate_start
            if elapsed >= 1.0:
                self.data["update_rate_hz"] = self.rate_frames / elapsed
                self.rate_frames = 0
                self.rate_start = now

    def neutral(self) -> None:
        with self.lock:
            self.data.update(
                {
                    "armed": False,
                    "steering": 0.0,
                    "drive": 0.0,
                    "raw_drive": 0.0,
                    "manual_drive": 0.0,
                    "cruise_drive": 0.0,
                    "cruise_level": 0,
                    "steering_ms": self.data["neutral_ms"],
                    "throttle_ms": self.data["neutral_ms"],
                    "last_update": time.time(),
                }
            )

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            result = json.loads(json.dumps(self.data))
        if result["last_update"]:
            result["data_age_ms"] = max(0.0, (time.time() - result["last_update"]) * 1000.0)
        else:
            result["data_age_ms"] = None
        result["server_time"] = time.time()
        return result


class DashboardHandler(BaseHTTPRequestHandler):
    status: ControllerStatus

    def do_GET(self) -> None:
        if self.path == "/":
            body = DASHBOARD_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/status":
            self._send_json(self.status.snapshot())
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(self.status.snapshot(), separators=(",", ":"))
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def _send_json(self, value: Any) -> None:
        body = json.dumps(value).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: Any) -> None:
        LOG.debug("HTTP: " + fmt, *args)


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def start_dashboard(status: ControllerStatus, host: str, port: int) -> DashboardServer | None:
    if port <= 0:
        return None
    DashboardHandler.status = status
    server = DashboardServer((host, port), DashboardHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    LOG.info("Dashboard listening on http://%s:%d", host, port)
    return server


def gamepad_score(evdev: Any, device: Any, config: dict[str, Any]) -> int:
    name = (device.name or "").lower()
    blocked = ("keyboard", "mouse", "touchpad", "consumer control", "system control", "hda", "gpio-keys")
    if any(word in name for word in blocked):
        return -100

    try:
        caps = device.capabilities()
    except OSError:
        return -100

    abs_codes = {cap_code(entry) for entry in caps.get(EV_ABS, [])}
    key_codes = {cap_code(entry) for entry in caps.get(EV_KEY, [])}
    if not abs_codes or not key_codes:
        return -100

    score = 0
    if evdev.ecodes.ABS_X in abs_codes:
        score += 20
    if evdev.ecodes.ABS_Y in abs_codes:
        score += 10
    for trigger in ("ABS_Z", "ABS_RZ", "ABS_GAS", "ABS_BRAKE"):
        code = evdev.ecodes.ecodes.get(trigger)
        if code in abs_codes:
            score += 6
    if any(0x130 <= code <= 0x13F for code in key_codes):
        score += 30

    wanted_names = config["input"].get("name_contains", [])
    for wanted in wanted_names:
        if str(wanted).lower() in name:
            score += 60
            break
    if "controller" in name or "gamepad" in name or "xbox" in name:
        score += 30
    return score


def find_gamepad(evdev: Any, config: dict[str, Any]) -> Any | None:
    requested = str(config["input"].get("device", "auto"))
    if requested != "auto":
        return evdev.InputDevice(requested)

    candidates = []
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
            score = gamepad_score(evdev, device, config)
            if score > 0:
                candidates.append((score, path, device))
            else:
                device.close()
        except OSError:
            continue
    if not candidates:
        return None
    candidates.sort(key=lambda item: (-item[0], item[1]))
    for _, _, unused in candidates[1:]:
        unused.close()
    return candidates[0][2]


def list_devices() -> int:
    evdev = import_evdev()
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    if not devices:
        print("No readable /dev/input/event* devices found.")
        print("Check Bluetooth pairing and input permissions: sudo usermod -aG input $USER")
        return 1

    for device in devices:
        print(f"{device.path}: {device.name}")
        print(f"  phys={device.phys or '-'} uniq={device.uniq or '-'}")
        try:
            caps = device.capabilities()
        except OSError as exc:
            print(f"  cannot read capabilities: {exc}")
            continue
        abs_entries = caps.get(EV_ABS, [])
        key_entries = caps.get(EV_KEY, [])
        if abs_entries:
            print("  axes:")
            for entry in abs_entries:
                code = cap_code(entry)
                try:
                    info = cap_absinfo(entry) or device.absinfo(code)
                    print(
                        f"    {axis_label(evdev, code):<14} code={code:<3} "
                        f"value={info.value:<6} min={info.min:<6} max={info.max:<6} flat={info.flat}"
                    )
                except OSError:
                    print(f"    {axis_label(evdev, code):<14} code={code}")
        if key_entries:
            names = [
                key_label(evdev, cap_code(entry))
                for entry in key_entries
                if 0x100 <= cap_code(entry) <= 0x2FF
            ]
            if names:
                print("  buttons:", ", ".join(names[:80]))
        print()
    return 0


def monitor(config: dict[str, Any]) -> int:
    evdev = import_evdev()
    device = find_gamepad(evdev, config)
    if device is None:
        print("No gamepad found. Use --list-devices to inspect readable input devices.")
        return 1
    print(f"Monitoring {device.path}: {device.name}")
    print("Move sticks/triggers and press buttons. Press Ctrl+C to stop.")
    try:
        for event in device.read_loop():
            if event.type == EV_ABS:
                info = device.absinfo(event.code)
                span = max(1, info.max - info.min)
                normalized = (event.value - info.min) / span
                print(
                    f"ABS {axis_label(evdev, event.code):<14} code={event.code:<3} "
                    f"value={event.value:<7} range=[{info.min},{info.max}] normalized={normalized:.3f}"
                )
            elif event.type == EV_KEY and event.value in (0, 1):
                print(
                    f"KEY {key_label(evdev, event.code):<14} code={event.code:<3} "
                    f"{'down' if event.value else 'up'}"
                )
    except KeyboardInterrupt:
        return 0
    finally:
        device.close()


@dataclass
class AxisInfo:
    minimum: int
    maximum: int
    flat: int = 0


class GamepadState:
    def __init__(self, evdev: Any, device: Any, config: dict[str, Any]) -> None:
        self.evdev = evdev
        self.device = device
        mapping = config["mapping"]
        control = config["control"]

        self.steering_codes = event_codes(evdev, mapping.get("steering_axis"), EV_ABS)
        self.throttle_codes = event_codes(evdev, mapping.get("throttle_axis"), EV_ABS)
        self.brake_codes = event_codes(evdev, mapping.get("brake_axis"), EV_ABS)
        self.throttle_button_codes = event_codes(evdev, mapping.get("throttle_button"), EV_KEY)
        self.brake_button_codes = event_codes(evdev, mapping.get("brake_button"), EV_KEY)
        self.arm_button = event_code(evdev, mapping.get("arm_button"), EV_KEY)
        self.disarm_button = event_code(evdev, mapping.get("disarm_button"), EV_KEY)
        self.neutral_button = event_code(evdev, mapping.get("neutral_button"), EV_KEY)
        self.cruise_button = event_code(evdev, mapping.get("cruise_button"), EV_KEY)
        self.steering_limit_button = event_code(evdev, mapping.get("steering_limit_button"), EV_KEY)
        self.steering_neutral_button = event_code(
            evdev, mapping.get("steering_neutral_button"), EV_KEY
        )
        self.deadman_button = event_code(
            evdev, mapping.get("deadman_button"), EV_KEY
        )
        self.exit_button = event_code(
            evdev, mapping.get("exit_button"), EV_KEY
        )

        self.steering_invert = bool(mapping.get("steering_invert", False))
        self.throttle_invert = bool(mapping.get("throttle_invert", False))
        self.brake_invert = bool(mapping.get("brake_invert", False))

        self.steering_deadzone = float(control["steering_deadzone"])
        self.throttle_deadzone = float(control["throttle_deadzone"])
        self.steering_limit = clamp(float(control["steering_limit"]), 0.0, 1.0)
        self.throttle_limit = clamp(float(control["throttle_limit"]), 0.0, 1.0)
        self.brake_limit = clamp(float(control["brake_limit"]), 0.0, 1.0)
        self.steering_expo = float(control["steering_expo"])
        self.throttle_expo = float(control["throttle_expo"])
        configured_levels = control.get("steering_limit_levels", [self.steering_limit])
        self.steering_limit_levels = [
            clamp(float(value), 0.05, 1.0)
            for value in configured_levels
            if isinstance(value, (int, float))
        ] or [self.steering_limit]
        self.steering_limit_index = min(
            range(len(self.steering_limit_levels)),
            key=lambda index: abs(self.steering_limit_levels[index] - self.steering_limit),
        )
        self.steering_limit = self.steering_limit_levels[self.steering_limit_index]
        neutral_ms = float(control["neutral_pulse_ms"])
        neutral_min = clamp(float(control.get("steering_neutral_min_ms", neutral_ms)), 0.5, 2.5)
        neutral_max = clamp(float(control.get("steering_neutral_max_ms", neutral_ms)), 0.5, 2.5)
        if neutral_max < neutral_min:
            neutral_min, neutral_max = neutral_max, neutral_min
        neutral_step = max(0.001, float(control.get("steering_neutral_step_ms", 0.025)))
        level_count = max(1, int(round((neutral_max - neutral_min) / neutral_step)))
        self.steering_neutral_levels = [
            round(neutral_min + index * neutral_step, 6)
            for index in range(level_count + 1)
        ]
        if self.steering_neutral_levels[-1] < neutral_max - 0.0005:
            self.steering_neutral_levels.append(round(neutral_max, 6))
        self.steering_neutral_index = min(
            range(len(self.steering_neutral_levels)),
            key=lambda index: abs(self.steering_neutral_levels[index] - neutral_ms),
        )
        self.steering_neutral_ms = self.steering_neutral_levels[self.steering_neutral_index]
        self.cruise_steps = max(1, int(control.get("cruise_steps", 10)))
        self.cruise_max = clamp(float(control.get("cruise_max", 1.0)), 0.0, 1.0)
        self.cruise_level = 0
        self.require_arm = bool(control["require_arm"])
        self.armed = bool(control["armed_on_start"]) or not self.require_arm
        self.require_deadman = bool(control.get("require_deadman", False))
        if self.require_deadman and self.deadman_button is None:
            raise ConfigError("require_deadman needs mapping.deadman_button")
        self.deadman_held = False
        self.exit_requested = False
        self.last_button_event = ""

        self.axis_info: dict[int, AxisInfo] = {}
        self.axis_values: dict[int, int] = {}
        self.button_names: dict[int, str] = {}
        self.button_values: dict[int, int] = {}

        try:
            caps = device.capabilities()
        except OSError:
            caps = {}

        for entry in caps.get(EV_ABS, []):
            code = cap_code(entry)
            try:
                info = cap_absinfo(entry) or device.absinfo(code)
            except OSError:
                continue
            self.axis_info[code] = AxisInfo(info.min, info.max, info.flat)
            self.axis_values[code] = info.value

        for entry in caps.get(EV_KEY, []):
            code = cap_code(entry)
            if 0x100 <= code <= 0x2FF:
                self.button_names[code] = key_label(evdev, code)
                self.button_values[code] = 0

        if self.require_deadman:
            try:
                self.deadman_held = self.deadman_button in set(device.active_keys())
            except OSError:
                self.deadman_held = False

    def process(self, event: Any) -> None:
        if event.type == EV_ABS:
            self.axis_values[event.code] = event.value
        elif event.type == EV_KEY:
            if event.value in (0, 1) and event.code in self.button_values:
                self.button_values[event.code] = event.value
            if event.code == self.deadman_button and event.value in (0, 1):
                self.deadman_held = event.value == 1
                if not self.deadman_held:
                    self.cruise_level = 0
                self.last_button_event = (
                    "RB motion enabled" if self.deadman_held else "RB neutral"
                )
            elif event.value == 1 and event.code == self.exit_button:
                self.armed = False
                self.cruise_level = 0
                self.exit_requested = True
                self.last_button_event = "START exit"
                LOG.warning("Manual field control exit requested")
            elif event.value == 1 and event.code == self.disarm_button:
                self.armed = False
                self.cruise_level = 0
                self.last_button_event = f"{short_input_name(key_label(self.evdev, event.code))} disarmed"
                LOG.warning("Disarmed by %s", key_label(self.evdev, event.code))
            elif event.value == 1 and event.code == self.arm_button:
                if self._triggers_released():
                    self.armed = True
                    self.last_button_event = f"{short_input_name(key_label(self.evdev, event.code))} armed"
                    LOG.warning("Armed by %s", key_label(self.evdev, event.code))
                else:
                    self.last_button_event = "arm rejected: release triggers"
                    LOG.warning("Arm rejected: release throttle/brake triggers first")
            elif event.value == 1 and event.code == self.neutral_button:
                self.armed = False
                self.cruise_level = 0
                self.last_button_event = f"{short_input_name(key_label(self.evdev, event.code))} neutral"
                LOG.warning("Neutral/disarmed by %s", key_label(self.evdev, event.code))
            elif event.value == 1 and event.code == self.cruise_button:
                if self.armed:
                    self.cruise_level = (self.cruise_level + 1) % (self.cruise_steps + 1)
                    self.last_button_event = f"X cruise {self.cruise_level}/{self.cruise_steps}"
                    LOG.warning("Cruise throttle level %s/%s", self.cruise_level, self.cruise_steps)
                else:
                    self.last_button_event = "X cruise rejected: press A to arm"
                    LOG.warning("Cruise rejected: controller is locked")
            elif event.value == 1 and event.code == self.steering_limit_button:
                self.steering_limit_index = (
                    self.steering_limit_index + 1
                ) % len(self.steering_limit_levels)
                self.steering_limit = self.steering_limit_levels[self.steering_limit_index]
                self.last_button_event = f"Y steer limit {self.steering_limit * 100:.0f}%"
                LOG.warning("Steering limit set to %.0f%%", self.steering_limit * 100)
            elif event.value == 1 and event.code == self.steering_neutral_button:
                self.steering_neutral_index = (
                    self.steering_neutral_index + 1
                ) % len(self.steering_neutral_levels)
                self.steering_neutral_ms = self.steering_neutral_levels[
                    self.steering_neutral_index
                ]
                self.last_button_event = f"TR steer center {self.steering_neutral_ms:.3f}ms"
                LOG.warning("Steering neutral set to %.3f ms", self.steering_neutral_ms)
            elif event.value == 1:
                self.last_button_event = f"{short_input_name(key_label(self.evdev, event.code))} down"

    def _triggers_released(self) -> bool:
        throttle = self._first_trigger(self.throttle_codes, self.throttle_invert)
        brake = self._first_trigger(self.brake_codes, self.brake_invert)
        if self._any_button_pressed(self.throttle_button_codes + self.brake_button_codes):
            return False
        return throttle < 0.08 and brake < 0.08

    def command(self) -> tuple[float, float]:
        controls = self.control_values()
        if not self.motion_permitted():
            return 0.0, 0.0
        return clamp(controls["steering"], -1.0, 1.0), clamp(controls["drive"], -1.0, 1.0)

    def motion_permitted(self) -> bool:
        return self.armed and (not self.require_deadman or self.deadman_held)

    def control_values(self) -> dict[str, float]:
        steering = 0.0
        for code in self.steering_codes:
            if code in self.axis_values:
                steering = self._signed_axis(code, self.steering_invert)
                break
        steering = apply_deadzone(steering, self.steering_deadzone)
        steering = apply_expo(steering, self.steering_expo)
        steering *= self.steering_limit

        throttle = self._first_trigger(self.throttle_codes, self.throttle_invert)
        brake = self._first_trigger(self.brake_codes, self.brake_invert)
        if self._any_button_pressed(self.throttle_button_codes):
            throttle = 1.0
        if self._any_button_pressed(self.brake_button_codes):
            brake = 1.0
        throttle = apply_trigger_deadzone(throttle, self.throttle_deadzone) * self.throttle_limit
        brake = apply_trigger_deadzone(brake, self.throttle_deadzone) * self.brake_limit
        manual_drive = apply_expo(throttle - brake, self.throttle_expo)
        cruise_drive = (
            self.cruise_level / self.cruise_steps * self.cruise_max * self.throttle_limit
        )
        drive = manual_drive if (throttle > 0.0 or brake > 0.0) else cruise_drive
        return {
            "steering": clamp(steering, -1.0, 1.0),
            "throttle": clamp(throttle, 0.0, 1.0),
            "brake": clamp(brake, 0.0, 1.0),
            "drive": clamp(drive, -1.0, 1.0),
            "steering_limit": self.steering_limit,
            "steering_limit_level": self.steering_limit_index + 1,
            "steering_limit_steps": len(self.steering_limit_levels),
            "steering_neutral_ms": self.steering_neutral_ms,
            "steering_neutral_level": self.steering_neutral_index + 1,
            "steering_neutral_steps": len(self.steering_neutral_levels),
            "manual_drive": clamp(manual_drive, -1.0, 1.0),
            "cruise_drive": clamp(cruise_drive, 0.0, 1.0),
            "cruise_level": self.cruise_level,
            "cruise_steps": self.cruise_steps,
        }

    def _signed_axis(self, code: int, invert: bool) -> float:
        info = self.axis_info.get(code)
        if info is None:
            return 0.0
        raw = self.axis_values.get(code, (info.minimum + info.maximum) // 2)
        center = (info.minimum + info.maximum) / 2.0
        half_span = max(1.0, (info.maximum - info.minimum) / 2.0)
        value = clamp((raw - center) / half_span, -1.0, 1.0)
        return -value if invert else value

    def _trigger_axis(self, code: int, invert: bool) -> float:
        info = self.axis_info.get(code)
        if info is None:
            return 0.0
        raw = self.axis_values.get(code, info.minimum)
        span = max(1.0, info.maximum - info.minimum)
        value = clamp((raw - info.minimum) / span, 0.0, 1.0)
        return 1.0 - value if invert else value

    def _first_trigger(self, codes: list[int], invert: bool) -> float:
        for code in codes:
            if code in self.axis_values:
                return self._trigger_axis(code, invert)
        return 0.0

    def _any_button_pressed(self, codes: list[int]) -> bool:
        return any(self.button_values.get(code, 0) == 1 for code in codes)

    def axis_snapshot(self) -> list[dict[str, Any]]:
        axes: list[dict[str, Any]] = []
        for code, info in sorted(self.axis_info.items()):
            raw = self.axis_values.get(code, info.minimum)
            span = max(1.0, info.maximum - info.minimum)
            normalized = clamp((raw - info.minimum) / span, 0.0, 1.0)
            signed = clamp((normalized - 0.5) * 2.0, -1.0, 1.0)
            axes.append(
                {
                    "code": code,
                    "name": axis_label(self.evdev, code),
                    "raw": raw,
                    "min": info.minimum,
                    "max": info.maximum,
                    "normalized": normalized,
                    "signed": signed,
                }
            )
        return axes

    def button_snapshot(self) -> list[dict[str, Any]]:
        return [
            {
                "code": code,
                "name": name,
                "short_name": short_input_name(name),
                "pressed": bool(self.button_values.get(code, 0)),
            }
            for code, name in sorted(self.button_names.items())
        ]


@dataclass(frozen=True)
class GyroAssistSample:
    enabled: bool = False
    healthy: bool = True
    calibrated: bool = True
    message: str = "disabled"
    driver_steering: float = 0.0
    final_steering: float = 0.0
    front_speed_mps: float = 0.0
    measured_yaw_rate_radps: float = 0.0
    filtered_yaw_rate_radps: float = 0.0
    target_yaw_rate_radps: float = 0.0
    yaw_rate_error_radps: float = 0.0
    correction: float = 0.0
    sensor_age_ms: float | None = None


class ManualGyroAssist:
    """Driver-commanded yaw-rate loop for manual RC control."""

    def __init__(self, config: dict[str, Any]) -> None:
        cfg = config.get("gyro_assist", {})
        self.enabled = bool(cfg.get("enabled", False))
        self.required_for_motion = bool(cfg.get("required_for_motion", False))
        self.imu_url = str(cfg.get("imu_url", ""))
        self.wheel_url = str(cfg.get("wheel_url", ""))
        self.request_timeout_s = max(0.01, float(cfg.get("request_timeout_s", 0.08)))
        self.max_data_age_s = max(0.02, float(cfg.get("max_data_age_s", 0.5)))
        self.poll_rate_hz = max(1.0, float(cfg.get("poll_rate_hz", 60.0)))
        self.calibration_samples = max(1, int(cfg.get("startup_calibration_samples", 30)))
        self.startup_max_abs_gyro = max(
            0.0, float(cfg.get("startup_max_abs_gyro_radps", 0.15))
        )
        self.startup_max_wheel_rpm = max(
            0.0, float(cfg.get("startup_max_wheel_rpm", 20.0))
        )
        self.mount_roll = float(cfg.get("imu_mount_roll_rad", 0.0))
        self.mount_pitch = float(cfg.get("imu_mount_pitch_rad", 0.0))
        self.gyro_z_sign = float(cfg.get("imu_gyro_z_sign", 1.0))
        self.wheel_rpm_key = str(cfg.get("wheel_rpm_key", "fast_rpm"))
        self.front_wheel_names = tuple(
            str(name) for name in cfg.get("front_wheel_names", ["front_left", "front_right"])
        )
        self.wheel_radius_m = float(cfg.get("wheel_radius_m", 0.05))
        self.wheelbase_m = float(cfg.get("wheelbase_m", 0.375))
        self.curve_commands = tuple(
            float(value) for value in cfg.get("steering_curve_commands", [])
        )
        self.curve_angles = tuple(
            float(value) for value in cfg.get("steering_curve_angles_rad", [])
        )
        self.max_steering_angle = float(cfg.get("max_steering_angle_rad", 0.45))
        self.cutoff_hz = float(cfg.get("cutoff_hz", 8.0))
        self.kp = float(cfg.get("kp", 0.08))
        self.max_correction = float(cfg.get("max_correction", 0.12))
        self.error_deadband = float(cfg.get("error_deadband_radps", 0.10))
        self.activation_speed = float(cfg.get("activation_speed_mps", 0.8))
        self.activation_blend = float(cfg.get("activation_blend_mps", 0.4))
        self.correction_slew = float(cfg.get("correction_slew_per_s", 4.0))
        self.max_target_yaw_rate = float(cfg.get("max_target_yaw_rate_radps", 6.0))

        if self.enabled:
            self._validate()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._calibration_values: list[float] = []
        self._calibrated = not self.enabled
        self._gyro_bias = 0.0
        self._measured_yaw_rate = 0.0
        self._front_rpm = 0.0
        self._last_sensor_update = 0.0
        self._last_error = "waiting for sensors" if self.enabled else ""
        self._last_error_log = 0.0
        self._filter_initialized = False
        self._filtered_yaw_rate = 0.0
        self._correction = 0.0
        self._sample = GyroAssistSample(enabled=self.enabled)

    def _validate(self) -> None:
        if not self.imu_url or not self.wheel_url:
            raise ConfigError("gyro_assist requires imu_url and wheel_url")
        if len(self.front_wheel_names) != 2:
            raise ConfigError("gyro_assist.front_wheel_names must contain two names")
        if self.wheel_radius_m <= 0.0 or self.wheelbase_m <= 0.0:
            raise ConfigError("gyro_assist wheel_radius_m and wheelbase_m must be positive")
        if self.cutoff_hz <= 0.0 or self.kp < 0.0:
            raise ConfigError("gyro_assist cutoff_hz must be positive and kp non-negative")
        if not 0.0 <= self.max_correction <= 1.0:
            raise ConfigError("gyro_assist.max_correction must be in [0, 1]")
        if self.activation_blend <= 0.0 or self.max_target_yaw_rate <= 0.0:
            raise ConfigError(
                "gyro_assist activation_blend_mps and max_target_yaw_rate_radps must be positive"
            )
        if len(self.curve_commands) != len(self.curve_angles):
            raise ConfigError("gyro_assist steering curve commands/angles length mismatch")
        if self.curve_commands and (
            len(self.curve_commands) < 2
            or any(
                self.curve_commands[index] >= self.curve_commands[index + 1]
                for index in range(len(self.curve_commands) - 1)
            )
        ):
            raise ConfigError("gyro_assist.steering_curve_commands must be strictly increasing")

    def start(self) -> None:
        if not self.enabled or self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._sensor_loop,
            name="manual-gyro-assist",
            daemon=True,
        )
        self._thread.start()
        LOG.info(
            "Manual gyro assist enabled: driver yaw target, Kp=%.3f, correction=+/-%.3f",
            self.kp,
            self.max_correction,
        )

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.5, 4.0 * self.request_timeout_s))

    def reset_control(self) -> None:
        with self._lock:
            self._filter_initialized = False
            self._filtered_yaw_rate = 0.0
            self._correction = 0.0
            self._sample = self._make_sample(
                driver_steering=0.0,
                final_steering=0.0,
                target_yaw_rate=0.0,
                yaw_rate_error=0.0,
            )

    def ready_for_motion(self) -> bool:
        if not self.enabled or not self.required_for_motion:
            return True
        with self._lock:
            return self._sensor_healthy_locked(time.monotonic())

    def sample(self) -> GyroAssistSample:
        with self._lock:
            return self._sample

    def update(self, driver_steering: float, dt: float) -> float:
        driver_steering = clamp(driver_steering, -1.0, 1.0)
        if not self.enabled:
            return driver_steering

        with self._lock:
            now = time.monotonic()
            if not self._sensor_healthy_locked(now):
                self._filter_initialized = False
                self._filtered_yaw_rate = 0.0
                self._correction = 0.0
                self._sample = self._make_sample(
                    driver_steering=driver_steering,
                    final_steering=driver_steering,
                    target_yaw_rate=0.0,
                    yaw_rate_error=0.0,
                    now=now,
                )
                return driver_steering

            safe_dt = clamp(dt, 1.0e-3, 0.1)
            measured_yaw_rate = self._measured_yaw_rate
            if not self._filter_initialized:
                self._filtered_yaw_rate = measured_yaw_rate
                self._filter_initialized = True
            else:
                alpha = 1.0 - math.exp(-2.0 * math.pi * self.cutoff_hz * safe_dt)
                self._filtered_yaw_rate += alpha * (
                    measured_yaw_rate - self._filtered_yaw_rate
                )

            front_speed_mps = (
                abs(self._front_rpm) * (2.0 * math.pi / 60.0) * self.wheel_radius_m
            )
            steering_angle = self._steering_angle(driver_steering)
            target_yaw_rate = clamp(
                front_speed_mps / self.wheelbase_m * math.tan(steering_angle),
                -self.max_target_yaw_rate,
                self.max_target_yaw_rate,
            )
            yaw_rate_error = target_yaw_rate - self._filtered_yaw_rate
            controlled_error = 0.0
            if abs(yaw_rate_error) > self.error_deadband:
                controlled_error = math.copysign(
                    abs(yaw_rate_error) - self.error_deadband,
                    yaw_rate_error,
                )
            activation = clamp(
                (front_speed_mps - self.activation_speed) / self.activation_blend,
                0.0,
                1.0,
            )
            requested_correction = activation * clamp(
                self.kp * controlled_error,
                -self.max_correction,
                self.max_correction,
            )
            self._correction = slew(
                self._correction,
                requested_correction,
                self.correction_slew,
                safe_dt,
            )
            self._correction = clamp(
                self._correction, -self.max_correction, self.max_correction
            )
            final_steering = clamp(driver_steering + self._correction, -1.0, 1.0)
            self._sample = self._make_sample(
                driver_steering=driver_steering,
                final_steering=final_steering,
                target_yaw_rate=target_yaw_rate,
                yaw_rate_error=yaw_rate_error,
                now=now,
            )
            return final_steering

    def _steering_angle(self, driver_steering: float) -> float:
        if self.curve_commands:
            return interpolate_curve(driver_steering, self.curve_commands, self.curve_angles)
        return driver_steering * self.max_steering_angle

    def _sensor_loop(self) -> None:
        period = 1.0 / self.poll_rate_hz
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                gyro_z, front_rpm = self._read_sensors()
                with self._lock:
                    self._front_rpm = front_rpm
                    self._last_sensor_update = time.monotonic()
                    self._last_error = ""
                    if not self._calibrated:
                        if (
                            abs(gyro_z) <= self.startup_max_abs_gyro
                            and abs(front_rpm) <= self.startup_max_wheel_rpm
                        ):
                            self._calibration_values.append(gyro_z)
                        else:
                            self._calibration_values.clear()
                        if len(self._calibration_values) >= self.calibration_samples:
                            self._gyro_bias = float(statistics.median(self._calibration_values))
                            self._calibrated = True
                            LOG.info(
                                "Manual gyro assist calibrated: gyro_z bias=%+.6f rad/s",
                                self._gyro_bias,
                            )
                    self._measured_yaw_rate = gyro_z - self._gyro_bias
                    self._sample = self._make_sample(
                        driver_steering=self._sample.driver_steering,
                        final_steering=self._sample.final_steering,
                        target_yaw_rate=self._sample.target_yaw_rate_radps,
                        yaw_rate_error=self._sample.yaw_rate_error_radps,
                    )
            except Exception as exc:
                with self._lock:
                    self._last_error = str(exc)
                now = time.monotonic()
                if now - self._last_error_log >= 2.0:
                    LOG.warning("Manual gyro assist sensor error: %s", exc)
                    self._last_error_log = now
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def _read_sensors(self) -> tuple[float, float]:
        imu = self._get_json(self.imu_url)
        wheels = self._get_json(self.wheel_url)
        if not bool(imu.get("online", False)):
            raise RuntimeError("IMU offline")
        if float(imu.get("data_age_ms", 0.0)) / 1000.0 > self.max_data_age_s:
            raise RuntimeError("IMU data stale")

        gyro = imu.get("gyro")
        if not isinstance(gyro, dict):
            raise RuntimeError("IMU gyro object missing")
        vector = [float(gyro.get(axis, 0.0)) for axis in ("x", "y", "z")]
        if "deg" in str(imu.get("gyro_unit", "rad/s")).lower():
            vector = [math.radians(value) for value in vector]
        body_gyro = self._rotate_sensor_to_body(vector)
        gyro_z = body_gyro[2] * self.gyro_z_sign

        channels = wheels.get("channels")
        if not isinstance(channels, list):
            raise RuntimeError("wheel channels missing")
        by_name = {
            str(channel.get("name", "")): channel
            for channel in channels
            if isinstance(channel, dict)
        }
        rpm_values: list[float] = []
        for name in self.front_wheel_names:
            channel = by_name.get(name)
            if channel is None:
                raise RuntimeError(f"front wheel channel missing: {name}")
            rpm_values.append(
                max(0.0, float(channel.get(self.wheel_rpm_key, channel.get("rpm", 0.0))))
            )
        return gyro_z, 0.5 * (rpm_values[0] + rpm_values[1])

    def _rotate_sensor_to_body(self, vector: list[float]) -> tuple[float, float, float]:
        cr = math.cos(self.mount_roll)
        sr = math.sin(self.mount_roll)
        cp = math.cos(self.mount_pitch)
        sp = math.sin(self.mount_pitch)
        rx_x = vector[0]
        rx_y = cr * vector[1] - sr * vector[2]
        rx_z = sr * vector[1] + cr * vector[2]
        return cp * rx_x + sp * rx_z, rx_y, -sp * rx_x + cp * rx_z

    def _get_json(self, url: str) -> dict[str, Any]:
        request = urllib.request.Request(url, headers={"Connection": "close"})
        with urllib.request.urlopen(request, timeout=self.request_timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
        if not isinstance(payload, dict):
            raise RuntimeError(f"non-object JSON from {url}")
        return payload

    def _sensor_healthy_locked(self, now: float) -> bool:
        return self._calibrated and (
            self._last_sensor_update > 0.0
            and now - self._last_sensor_update <= self.max_data_age_s
        )

    def _make_sample(
        self,
        driver_steering: float,
        final_steering: float,
        target_yaw_rate: float,
        yaw_rate_error: float,
        now: float | None = None,
    ) -> GyroAssistSample:
        now = time.monotonic() if now is None else now
        healthy = self._sensor_healthy_locked(now)
        if healthy:
            message = "ready"
        elif not self._calibrated:
            message = f"calibrating {len(self._calibration_values)}/{self.calibration_samples}"
        elif self._last_error:
            message = self._last_error
        else:
            message = "sensor data stale"
        sensor_age_ms = (
            max(0.0, (now - self._last_sensor_update) * 1000.0)
            if self._last_sensor_update > 0.0
            else None
        )
        front_speed_mps = (
            abs(self._front_rpm) * (2.0 * math.pi / 60.0) * self.wheel_radius_m
        )
        return GyroAssistSample(
            enabled=self.enabled,
            healthy=healthy,
            calibrated=self._calibrated,
            message=message,
            driver_steering=driver_steering,
            final_steering=final_steering,
            front_speed_mps=front_speed_mps,
            measured_yaw_rate_radps=self._measured_yaw_rate,
            filtered_yaw_rate_radps=self._filtered_yaw_rate,
            target_yaw_rate_radps=target_yaw_rate,
            yaw_rate_error_radps=yaw_rate_error,
            correction=self._correction,
            sensor_age_ms=sensor_age_ms,
        )


class PwmChannel:
    def start(self, frequency_hz: float, neutral_ms: float) -> None:
        raise NotImplementedError

    def set_pulse_ms(self, pulse_ms: float) -> None:
        raise NotImplementedError

    def close(self, neutral_ms: float, disable: bool) -> None:
        raise NotImplementedError


class DryRunPwmChannel(PwmChannel):
    def __init__(self, label: str, inverted: bool = False) -> None:
        self.label = label
        self.inverted = inverted
        self.last: float | None = None

    def start(self, frequency_hz: float, neutral_ms: float) -> None:
        LOG.info(
            "DRY PWM %s start %.1f Hz neutral %.3f ms polarity=%s",
            self.label,
            frequency_hz,
            neutral_ms,
            "inverted" if self.inverted else "normal",
        )
        self.set_pulse_ms(neutral_ms)

    def set_pulse_ms(self, pulse_ms: float) -> None:
        self.last = pulse_ms

    def close(self, neutral_ms: float, disable: bool) -> None:
        self.set_pulse_ms(neutral_ms)
        LOG.info("DRY PWM %s neutral %.3f ms", self.label, neutral_ms)


class SysfsPwmChannel(PwmChannel):
    def __init__(self, spec: str, label: str, inverted: bool = False) -> None:
        self.spec = spec
        self.label = label
        self.inverted = inverted
        self.chip_path, self.index = self._resolve(spec)
        self.pwm_path = self.chip_path / f"pwm{self.index}"
        self.period_ns = 20_000_000

    @staticmethod
    def _resolve(spec: str) -> tuple[Path, int]:
        spec = BOARD_PWM_ALIASES.get(spec.lower(), spec)
        if ":" in spec:
            chip_spec, index_text = spec.rsplit(":", 1)
            index = int(index_text)
        else:
            chip_spec, index = spec, 0

        if chip_spec.startswith("/sys/class/pwm/"):
            chip = Path(chip_spec)
            return chip, index
        if chip_spec.startswith("pwmchip"):
            return Path("/sys/class/pwm") / chip_spec, index

        for chip in sorted(Path("/sys/class/pwm").glob("pwmchip*")):
            try:
                target = chip.resolve()
            except OSError:
                continue
            if chip_spec in str(target):
                return chip, index
        raise ConfigError(
            f"Could not resolve PWM spec '{spec}'. Available chips: "
            + ", ".join(str(p) for p in sorted(Path("/sys/class/pwm").glob("pwmchip*")))
        )

    def start(self, frequency_hz: float, neutral_ms: float) -> None:
        self.period_ns = int(round(1_000_000_000 / frequency_hz))
        if not self.pwm_path.exists():
            self._write(self.chip_path / "export", str(self.index))
            deadline = time.monotonic() + 1.0
            while not self.pwm_path.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
        if not self.pwm_path.exists():
            raise ConfigError(f"PWM export failed: {self.pwm_path}")

        enable = self.pwm_path / "enable"
        if enable.exists():
            try:
                self._write(enable, "0")
            except OSError:
                pass
        self._write_checked(self.pwm_path / "period", str(self.period_ns))
        self._write_checked(self.pwm_path / "duty_cycle", str(self._duty_ns(neutral_ms)))
        self._write_checked(enable, "1")
        LOG.info(
            "PWM %s -> %s pwm%d period %.3f ms polarity=%s",
            self.label,
            self.chip_path,
            self.index,
            self.period_ns / 1_000_000,
            "inverted" if self.inverted else "normal",
        )

    def set_pulse_ms(self, pulse_ms: float) -> None:
        self._write_checked(self.pwm_path / "duty_cycle", str(self._duty_ns(pulse_ms)))

    def _duty_ns(self, pulse_ms: float) -> int:
        duty_ns = int(round(pulse_ms * 1_000_000))
        duty_ns = max(0, min(self.period_ns, duty_ns))
        if self.inverted:
            duty_ns = self.period_ns - duty_ns
        return duty_ns

    def close(self, neutral_ms: float, disable: bool) -> None:
        try:
            self.set_pulse_ms(neutral_ms)
            if disable:
                self._write_checked(self.pwm_path / "enable", "0")
        except OSError as exc:
            LOG.warning("Could not close PWM %s cleanly: %s", self.label, exc)

    def _write_checked(self, path: Path, value: str) -> None:
        try:
            self._write(path, value)
        except PermissionError as exc:
            raise ConfigError(
                f"Permission denied writing {path}. Add the user to gpio group "
                "or run the service with PWM permissions."
            ) from exc
        except OSError as exc:
            raise ConfigError(
                f"Could not write {value!r} to {path}: {exc.strerror or exc}. "
                "If this is a PWM enable/period error, enable the header pin's PWM "
                "function in Jetson-IO and reboot."
            ) from exc

    @staticmethod
    def _write(path: Path, value: str) -> None:
        fd = os.open(path, os.O_WRONLY)
        try:
            os.write(fd, value.encode("ascii"))
        finally:
            os.close(fd)


class JetsonGpioPwmChannel(PwmChannel):
    def __init__(self, board_pin: int, label: str, inverted: bool = False) -> None:
        self.board_pin = int(board_pin)
        self.label = label
        self.inverted = inverted
        self.GPIO: Any = None
        self.pwm: Any = None
        self.period_ms = 20.0

    def start(self, frequency_hz: float, neutral_ms: float) -> None:
        try:
            import Jetson.GPIO as GPIO  # type: ignore
        except Exception as exc:
            raise ConfigError(f"Jetson.GPIO backend is unavailable: {exc}") from exc
        self.GPIO = GPIO
        self.period_ms = 1000.0 / frequency_hz
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.board_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.board_pin, frequency_hz)
        self.pwm.start(self._duty(neutral_ms))
        LOG.info(
            "Jetson.GPIO PWM %s -> BOARD%d polarity=%s",
            self.label,
            self.board_pin,
            "inverted" if self.inverted else "normal",
        )

    def set_pulse_ms(self, pulse_ms: float) -> None:
        self.pwm.ChangeDutyCycle(self._duty(pulse_ms))

    def close(self, neutral_ms: float, disable: bool) -> None:
        if self.pwm is not None:
            self.set_pulse_ms(neutral_ms)
            if disable:
                self.pwm.stop()
        if self.GPIO is not None and disable:
            self.GPIO.cleanup(self.board_pin)

    def _duty(self, pulse_ms: float) -> float:
        duty = clamp(pulse_ms / self.period_ms * 100.0, 0.0, 100.0)
        return 100.0 - duty if self.inverted else duty


class PwmOutputs:
    def __init__(self, config: dict[str, Any]) -> None:
        pwm_config = config["pwm"]
        control = config["control"]
        self.frequency_hz = float(control["frequency_hz"])
        self.min_ms = float(control["min_pulse_ms"])
        self.neutral_ms = float(control["neutral_pulse_ms"])
        self.max_ms = float(control["max_pulse_ms"])
        self.disable_on_exit = bool(pwm_config.get("disable_on_exit", False))
        self.dry_run = bool(pwm_config.get("dry_run", False))
        self.steering_output_inverted = bool(pwm_config.get("steering_output_inverted", False))
        self.throttle_output_inverted = bool(pwm_config.get("throttle_output_inverted", False))
        backend = str(pwm_config.get("backend", "sysfs")).lower()

        if self.dry_run or backend == "dry-run":
            self.steering = DryRunPwmChannel("steering", self.steering_output_inverted)
            self.throttle = DryRunPwmChannel("throttle", self.throttle_output_inverted)
        elif backend == "sysfs":
            self.steering = SysfsPwmChannel(
                str(pwm_config["steering"]),
                "steering",
                self.steering_output_inverted,
            )
            self.throttle = SysfsPwmChannel(
                str(pwm_config["throttle"]),
                "throttle",
                self.throttle_output_inverted,
            )
        elif backend == "jetson-gpio":
            self.steering = JetsonGpioPwmChannel(
                int(pwm_config["steering"]),
                "steering",
                self.steering_output_inverted,
            )
            self.throttle = JetsonGpioPwmChannel(
                int(pwm_config["throttle"]),
                "throttle",
                self.throttle_output_inverted,
            )
        else:
            raise ConfigError(f"Unsupported PWM backend: {backend}")

    def start(self) -> None:
        self.steering.start(self.frequency_hz, self.neutral_ms)
        self.throttle.start(self.frequency_hz, self.neutral_ms)

    def set_command(
        self,
        steering: float,
        drive: float,
        steering_neutral_ms: float | None = None,
    ) -> tuple[float, float]:
        steering_center = self.neutral_ms if steering_neutral_ms is None else steering_neutral_ms
        steering_ms = signed_to_pulse(steering, self.min_ms, steering_center, self.max_ms)
        throttle_ms = signed_to_pulse(drive, self.min_ms, self.neutral_ms, self.max_ms)
        self.steering.set_pulse_ms(steering_ms)
        self.throttle.set_pulse_ms(throttle_ms)
        return steering_ms, throttle_ms

    def neutral(self) -> None:
        self.set_command(0.0, 0.0)

    def close(self) -> None:
        self.steering.close(self.neutral_ms, self.disable_on_exit)
        self.throttle.close(self.neutral_ms, self.disable_on_exit)


def run_controller(config: dict[str, Any], host: str, port: int) -> int:
    evdev = import_evdev()
    status = ControllerStatus(config)
    outputs = PwmOutputs(config)
    gyro_assist = ManualGyroAssist(config)
    outputs.start()
    outputs.neutral()
    status.neutral()
    gyro_assist.start()
    server = start_dashboard(status, host, port)

    stop = False

    def request_stop(signum: int, frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    control = config["control"]
    status_rate = float(config.get("status", {}).get("print_rate_hz", 5.0))
    status_period = 1.0 / status_rate if status_rate > 0 else 0.0
    update_period = 1.0 / float(control["frequency_hz"])
    reconnect_s = float(config["input"].get("reconnect_s", 1.0))
    grab = bool(config["input"].get("grab", True))
    max_steer_slew = float(control["max_steering_slew_per_s"])
    max_drive_slew = float(control["max_throttle_slew_per_s"])

    current_steering = 0.0
    current_driver_steering = 0.0
    current_drive = 0.0
    last_update = time.monotonic()
    last_status = 0.0

    try:
        while not stop:
            device = find_gamepad(evdev, config)
            if device is None:
                outputs.neutral()
                gyro_assist.reset_control()
                status.neutral()
                status.set_connection(False, "No gamepad found", None, None)
                LOG.warning("No gamepad found; waiting %.1f s", reconnect_s)
                time.sleep(reconnect_s)
                continue

            LOG.info("Using gamepad %s: %s", device.path, device.name)
            status.set_connection(True, "Gamepad connected", device.name, device.path)
            if grab:
                try:
                    device.grab()
                    LOG.info("Grabbed %s so other apps will not consume it", device.path)
                except OSError as exc:
                    LOG.warning("Could not grab %s: %s", device.path, exc)
            state = GamepadState(evdev, device, config)

            try:
                while not stop:
                    now = time.monotonic()
                    wait_s = max(0.0, update_period - (now - last_update))
                    readable, _, _ = select([device.fd], [], [], wait_s)
                    if readable:
                        for event in device.read():
                            state.process(event)

                    if state.exit_requested:
                        current_steering = 0.0
                        current_driver_steering = 0.0
                        current_drive = 0.0
                        outputs.neutral()
                        gyro_assist.reset_control()
                        status.neutral()
                        stop = True
                        break

                    now = time.monotonic()
                    dt = now - last_update
                    if dt < update_period * 0.5:
                        continue
                    motion_permitted = state.motion_permitted() and gyro_assist.ready_for_motion()
                    if not motion_permitted:
                        current_steering = 0.0
                        current_driver_steering = 0.0
                        current_drive = 0.0
                        outputs.neutral()
                        gyro_assist.reset_control()
                        steering_ms = float(control["neutral_pulse_ms"])
                        throttle_ms = float(control["neutral_pulse_ms"])
                    else:
                        target_steering, target_drive = state.command()
                        current_driver_steering = slew(
                            current_driver_steering, target_steering, max_steer_slew, dt
                        )
                        current_steering = gyro_assist.update(current_driver_steering, dt)
                        current_drive = slew(
                            current_drive, target_drive, max_drive_slew, dt
                        )
                        steering_ms, throttle_ms = outputs.set_command(
                            current_steering,
                            current_drive,
                            state.steering_neutral_ms,
                        )
                    gyro_sample = gyro_assist.sample()
                    status.update(
                        state,
                        current_steering,
                        current_drive,
                        steering_ms,
                        throttle_ms,
                        gyro_sample,
                    )
                    last_update = now

                    if status_period and now - last_status >= status_period:
                        LOG.info(
                            "motion=%s steer=%+.2f %.3fms drive=%+.2f %.3fms "
                            "gyro=%s yaw=%+.2f/%+.2f corr=%+.3f %s",
                            "yes" if motion_permitted else "no",
                            current_steering,
                            steering_ms,
                            current_drive,
                            throttle_ms,
                            gyro_sample.message,
                            gyro_sample.measured_yaw_rate_radps,
                            gyro_sample.target_yaw_rate_radps,
                            gyro_sample.correction,
                            state.last_button_event,
                        )
                        last_status = now
            except OSError as exc:
                LOG.warning("Gamepad disconnected/read error: %s", exc)
                outputs.neutral()
                gyro_assist.reset_control()
                status.neutral()
                status.set_connection(False, f"Gamepad disconnected: {exc}", None, None)
            finally:
                try:
                    if grab:
                        device.ungrab()
                except OSError:
                    pass
                device.close()
    finally:
        LOG.info("Setting neutral PWM outputs")
        outputs.neutral()
        gyro_assist.reset_control()
        status.neutral()
        outputs.close()
        gyro_assist.close()
        if server is not None:
            server.shutdown()
            server.server_close()
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Map a Bluetooth gamepad to two 50 Hz RC PWM outputs."
    )
    parser.add_argument("--config", help="JSON config file")
    parser.add_argument("--list-devices", action="store_true", help="List readable Linux input devices")
    parser.add_argument("--monitor", action="store_true", help="Print gamepad events instead of driving PWM")
    parser.add_argument("--dry-run", action="store_true", help="Do not touch PWM hardware")
    parser.add_argument("--device", help="Override input device path, or 'auto'")
    parser.add_argument("--host", default="0.0.0.0", help="Dashboard bind host")
    parser.add_argument("--port", type=int, default=8090, help="Dashboard port, or 0 to disable")
    parser.add_argument("--print-default-config", action="store_true")
    parser.add_argument("--write-default-config", help="Write default JSON config to this path")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    if args.print_default_config:
        print(json.dumps(DEFAULT_CONFIG, indent=2, sort_keys=True))
        return 0
    if args.write_default_config:
        path = Path(args.write_default_config)
        path.write_text(json.dumps(DEFAULT_CONFIG, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Wrote {path}")
        return 0
    if args.list_devices:
        return list_devices()

    try:
        config = load_config(args.config)
        if args.device:
            config["input"]["device"] = args.device
        if args.dry_run:
            config["pwm"]["dry_run"] = True
        if args.monitor:
            return monitor(config)
        return run_controller(config, args.host, args.port)
    except ConfigError as exc:
        LOG.error("%s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
