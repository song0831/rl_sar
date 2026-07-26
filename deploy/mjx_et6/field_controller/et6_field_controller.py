#!/usr/bin/env python3
"""Gamepad-only launcher for ET6 manual driving and ONNX policy control."""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import select
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from types import ModuleType
from typing import Any


LOG = logging.getLogger("et6-field-controller")
STOP_REQUESTED = False


def request_stop(_signum: int, _frame: Any) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def load_module(path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location("et6_gamepad_pwm", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load gamepad module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def run_user_systemctl(action: str, *services: str) -> None:
    subprocess.run(
        ["systemctl", "--user", action, *services],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def wait_for_mode(
    gamepad_module: ModuleType,
    gamepad_config: dict[str, Any],
    manual_button: str,
    policy_button: str,
    deadman_button: str,
    hold_s: float,
) -> str | None:
    evdev = gamepad_module.import_evdev()
    manual_code = gamepad_module.event_code(
        evdev, manual_button, gamepad_module.EV_KEY
    )
    policy_code = gamepad_module.event_code(
        evdev, policy_button, gamepad_module.EV_KEY
    )
    deadman_code = gamepad_module.event_code(
        evdev, deadman_button, gamepad_module.EV_KEY
    )
    if None in (manual_code, policy_code, deadman_code):
        raise RuntimeError("X/Y/RB gamepad mappings could not be resolved")

    last_wait_log_s = 0.0
    while not STOP_REQUESTED:
        device = gamepad_module.find_gamepad(evdev, gamepad_config)
        if device is None:
            now_s = time.monotonic()
            if now_s - last_wait_log_s >= 10.0:
                LOG.info("Waiting for gamepad: hold X for manual or Y for policy")
                last_wait_log_s = now_s
            time.sleep(1.0)
            continue

        pressed_since: dict[int, float] = {}
        deadman_held = False
        try:
            deadman_held = deadman_code in set(device.active_keys())
            try:
                device.grab()
            except OSError:
                pass
            LOG.info(
                "Gamepad connected: %s. Hold X %.1fs for manual or Y %.1fs for policy; RB must be released while selecting.",
                device.name,
                hold_s,
                hold_s,
            )
            while not STOP_REQUESTED:
                readable, _, _ = select.select([device.fd], [], [], 0.05)
                if readable:
                    for event in device.read():
                        if (
                            event.type != gamepad_module.EV_KEY
                            or event.value not in (0, 1)
                        ):
                            continue
                        if event.code == deadman_code:
                            deadman_held = event.value == 1
                            if deadman_held:
                                pressed_since.clear()
                            continue
                        if event.code not in (manual_code, policy_code):
                            continue
                        if event.value == 1 and not deadman_held:
                            pressed_since[event.code] = time.monotonic()
                        else:
                            pressed_since.pop(event.code, None)

                now_s = time.monotonic()
                for code, mode in (
                    (manual_code, "manual"),
                    (policy_code, "policy"),
                ):
                    start_s = pressed_since.get(code)
                    if (
                        start_s is not None
                        and not deadman_held
                        and now_s - start_s >= hold_s
                    ):
                        LOG.info("Selected field mode: %s", mode)
                        return mode
        except OSError as exc:
            LOG.warning("Gamepad disconnected while selecting mode: %s", exc)
        finally:
            try:
                device.ungrab()
            except OSError:
                pass
            device.close()
    return None


def write_manifest(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def copy_new_policy_logs(
    policy_log_dir: Path,
    session_dir: Path,
    started_wall_s: float,
) -> list[str]:
    copied: list[str] = []
    destination = session_dir / "policy_telemetry"
    destination.mkdir(parents=True, exist_ok=True)
    for source in sorted(policy_log_dir.glob("rl_log_mjx_et6*.csv")):
        try:
            if source.stat().st_mtime < started_wall_s - 1.0:
                continue
            target = destination / source.name
            shutil.copy2(source, target)
            copied.append(str(target))
        except OSError as exc:
            LOG.warning("Could not archive policy telemetry %s: %s", source, exc)
    return copied


def run_field_mode(args: argparse.Namespace, mode: str) -> None:
    run_user_systemctl("stop", "gamepad-pwm.service")
    run_user_systemctl(
        "start", "imu-dashboard.service", "wheel-speed-monitor.service"
    )

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    session_dir = args.session_root / f"{stamp}_{mode}"
    session_dir.mkdir(parents=True, exist_ok=False)
    manifest_path = session_dir / "manifest.json"
    runner_log_path = session_dir / "runner.log"
    started_wall_s = time.time()
    manifest: dict[str, Any] = {
        "mode": mode,
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "status": "running",
        "session_dir": str(session_dir),
    }
    write_manifest(manifest_path, manifest)

    if mode == "policy":
        command = [
            str(args.policy_root / "run_mjx_et6.sh"),
            "--config",
            str(args.policy_config),
            "--enable-output",
            "--max-runtime-s",
            str(args.policy_runtime_s),
            "--rb-only",
            "--keep-gamepad-service-stopped",
        ]
        cwd = args.policy_root
    elif mode == "manual":
        command = [
            "/usr/bin/python3",
            str(args.gamepad_module),
            "--config",
            str(args.manual_gamepad_config),
            "--host",
            "127.0.0.1",
            "--port",
            "0",
        ]
        cwd = args.gamepad_module.parent
    else:
        raise ValueError(f"Unknown field mode: {mode}")

    LOG.info("Starting %s; release and re-hold RB to permit motion", mode)
    return_code = -1
    try:
        with runner_log_path.open("w", encoding="utf-8") as runner_log:
            completed = subprocess.run(
                command,
                cwd=cwd,
                stdout=runner_log,
                stderr=subprocess.STDOUT,
                check=False,
            )
            return_code = completed.returncode
    finally:
        run_user_systemctl("stop", "gamepad-pwm.service")

    archived_logs: list[str] = []
    if mode == "policy":
        archived_logs = copy_new_policy_logs(
            args.policy_root / "policy" / "MJX_ET6",
            session_dir,
            started_wall_s,
        )

    manifest.update(
        {
            "finished_at": datetime.now().isoformat(timespec="seconds"),
            "status": "complete" if return_code == 0 else "failed",
            "return_code": return_code,
            "runner_log": str(runner_log_path),
            "archived_policy_telemetry": archived_logs,
        }
    )
    write_manifest(manifest_path, manifest)
    LOG.info(
        "%s finished with rc=%d. Data saved under %s",
        mode,
        return_code,
        session_dir,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--field-config",
        type=Path,
        default=Path.home() / "MJX_Drive" / "field_controller" / "config.json",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    raw = json.loads(args.field_config.read_text(encoding="utf-8"))
    args.policy_root = Path(raw["policy_root"])
    args.policy_config = Path(raw["policy_config"])
    args.gamepad_module = Path(raw["gamepad_module"])
    args.manual_gamepad_config = Path(raw["manual_gamepad_config"])
    args.session_root = Path(raw["session_root"])
    args.policy_runtime_s = float(raw.get("policy_runtime_s", 16.0))
    hold_s = float(raw.get("mode_hold_s", 1.0))
    if not 0.5 <= hold_s <= 3.0:
        raise ValueError("mode_hold_s must be between 0.5 and 3.0 seconds")
    if not 1.0 <= args.policy_runtime_s <= 60.0:
        raise ValueError("policy_runtime_s must be between 1 and 60 seconds")
    for required_path in (
        args.policy_root / "run_mjx_et6.sh",
        args.policy_config,
        args.policy_root / "policy" / "MJX_ET6" / "nio_lab" / "drift" / "policy.onnx",
        args.gamepad_module,
        args.manual_gamepad_config,
    ):
        if not required_path.is_file():
            raise FileNotFoundError(f"Required field-mode file is missing: {required_path}")

    gamepad_module = load_module(args.gamepad_module)
    gamepad_config = gamepad_module.load_config(str(args.manual_gamepad_config))
    args.session_root.mkdir(parents=True, exist_ok=True)
    run_user_systemctl("stop", "gamepad-pwm.service")

    LOG.info("ET6 gamepad-only field controller is ready")
    while not STOP_REQUESTED:
        mode = wait_for_mode(
            gamepad_module,
            gamepad_config,
            str(raw.get("manual_button", "BTN_WEST")),
            str(raw.get("policy_button", "BTN_NORTH")),
            str(raw.get("deadman_button", "BTN_TR")),
            hold_s,
        )
        if mode is None:
            break
        run_field_mode(args, mode)
    return 0


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    raise SystemExit(main())
