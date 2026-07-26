# MJX ET6 Jetson Deployment

This package deploys the `Nio-ET6-Circle-Drift-v0` direct-PWM policy to the
MJX Hyper Go 7303 through the existing `rl_sar` ONNX runtime.

## Runtime Contract

- Policy loop: `60 Hz`; RC PWM carrier: `50 Hz`.
- Action: normalized steering and forward throttle, exported by
  `policy_pwm.onnx` as two pulse widths in milliseconds.
- Observation: eight frames of IMU (9), four wheel RPM (4), and previous base
  action (2), in Isaac Lab term-major order (`120` values total).
- Drive: rear-wheel drive; reverse and active braking are disabled.
- Absolute yaw is masked. H30 `gyro_z` uses the identified `-1` sign and the
  measured fixed mounting rotation.

The policy base steering is followed by a bounded yaw-rate residual. The live
policy target is the clockwise training circle. Manual RC mode instead converts
the driver's steering through the identified ET6 steering curve and requests

```text
target_yaw_rate = front_speed / wheelbase * tan(steering_angle)
```

so left and right manual turns both receive gyro stabilization. Both modes use
the H30 body-frame `gyro_z`; correction is limited to `+/-0.12`.

## Files

```text
src/rl_sar/src/rl_real_mjx_et6.cpp       ONNX inference and real PWM runner
policy/MJX_ET6/                          checked-in configuration only
run_mjx_et6.sh                           direct runner launcher
deploy/mjx_et6/                          gamepad and field-selector sources
scripts/install_mjx_et6.sh               Jetson installer
```

Policy models, deployment telemetry, PWM replay CSVs, backups, and field
sessions are intentionally excluded from Git.

## Prerequisites

1. H30 IMU dashboard at `http://127.0.0.1:8080/api/data`.
2. Four-wheel RPM service at `http://127.0.0.1:8081/api/data`.
3. `python3-evdev`, `yaml-cpp`, CMake, and the repository ONNX Runtime bundle.
4. Jetson PWM channels configured for the steering and ESC outputs.

Install ONNX Runtime when it is absent:

```bash
bash scripts/download_inference_runtime.sh onnx
```

## Model And Build

Copy the exported direct-PWM policy without committing it:

```bash
mkdir -p policy/MJX_ET6/nio_lab/drift
cp /path/to/policy_pwm.onnx policy/MJX_ET6/nio_lab/drift/policy.onnx
```

Install runtime files, compile, and enable the gamepad-only field selector:

```bash
./scripts/install_mjx_et6.sh --enable-service
```

To install and build without changing the active PWM owner:

```bash
./scripts/install_mjx_et6.sh
```

## Operation

After the selector service starts:

- hold `X` for one second: normal manual RC mode with bidirectional gyro assist;
- hold `Y` for one second: ONNX drift policy for at most 16 active seconds;
- hold `RB`: permit motion; release it for immediate neutral;
- press `START`: return to the `X`/`Y` selector.

Keep the car stationary while entering either mode. The first 30 H30 samples
establish the current gyro bias. Manual mode remains neutral until IMU and both
front-wheel RPM channels are calibrated and fresh.

Direct dry-run inference does not write PWM:

```bash
./run_mjx_et6.sh --dry-run --max-runtime-s 10
```

## Safety

Only one process may own the steering and throttle PWM channels. The installer
disables `gamepad-pwm.service` before enabling `et6-field-controller.service`.
RB release, START, gamepad loss, stale required sensors, process exit, and
exceptions return both outputs to `1.500 ms`. Test with the driven wheels lifted
before any ground run.
