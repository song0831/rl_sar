#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SOURCE_DIR="$REPO_ROOT/deploy/mjx_et6"
INSTALL_ROOT="${HOME}/MJX_Drive"
ENABLE_SERVICE=false
BUILD_TARGET=true

usage() {
    cat <<'EOF'
Install the MJX ET6 field controller from this rl_sar checkout.

Usage: ./scripts/install_mjx_et6.sh [options]

Options:
  --enable-service       Enable and start the gamepad-only X/Y selector.
  --no-build             Skip building the rl_real_mjx_et6 target.
  --install-root PATH    Runtime files (default: $HOME/MJX_Drive).
  -h, --help             Show this help.

The ONNX policy is never installed from Git. Before --enable-service, place it at:
  policy/MJX_ET6/nio_lab/drift/policy.onnx
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --enable-service)
            ENABLE_SERVICE=true
            shift
            ;;
        --no-build)
            BUILD_TARGET=false
            shift
            ;;
        --install-root)
            [[ $# -ge 2 ]] || { echo "--install-root needs a path" >&2; exit 2; }
            INSTALL_ROOT="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

FIELD_DIR="$INSTALL_ROOT/field_controller"
GAMEPAD_DIR="$INSTALL_ROOT/gamepad_pwm"
SESSION_DIR="$INSTALL_ROOT/field_sessions"
SERVICE_DIR="$HOME/.config/systemd/user"
MODEL_PATH="$REPO_ROOT/policy/MJX_ET6/nio_lab/drift/policy.onnx"

mkdir -p "$FIELD_DIR" "$GAMEPAD_DIR" "$SESSION_DIR" "$SERVICE_DIR"
install -m 755 "$SOURCE_DIR/field_controller/et6_field_controller.py" \
    "$FIELD_DIR/et6_field_controller.py"
install -m 755 "$SOURCE_DIR/gamepad_pwm.py" "$GAMEPAD_DIR/gamepad_pwm.py"
install -m 644 "$SOURCE_DIR/field_controller/manual_field.json.in" \
    "$FIELD_DIR/manual_field.json"
python3 - \
    "$SOURCE_DIR/field_controller/config.json.in" \
    "$FIELD_DIR/config.json" \
    "$SOURCE_DIR/field_controller/policy_field.yaml.in" \
    "$FIELD_DIR/policy_field.yaml" \
    "$SOURCE_DIR/field_controller/et6-field-controller.service.in" \
    "$SERVICE_DIR/et6-field-controller.service" \
    "$REPO_ROOT" \
    "$INSTALL_ROOT" <<'PY'
import json
import sys
from pathlib import Path

(
    config_source,
    config_target,
    policy_source,
    policy_target,
    service_source,
    service_target,
    policy_root,
    install_root,
) = (
    Path(value) if index < 6 else value
    for index, value in enumerate(sys.argv[1:])
)

config = json.loads(config_source.read_text(encoding="utf-8"))
for key, value in list(config.items()):
    if isinstance(value, str):
        config[key] = value.replace("@POLICY_ROOT@", policy_root).replace(
            "@INSTALL_ROOT@", install_root
        )
config_target.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")

policy = policy_source.read_text(encoding="utf-8")
policy = policy.replace("@POLICY_ROOT@", policy_root).replace(
    "@INSTALL_ROOT@", install_root
)
policy_target.write_text(policy, encoding="utf-8")

service = service_source.read_text(encoding="utf-8")
service = service.replace("@INSTALL_ROOT@", install_root)
service_target.write_text(service, encoding="utf-8")
PY

python3 -m py_compile "$FIELD_DIR/et6_field_controller.py" "$GAMEPAD_DIR/gamepad_pwm.py"
python3 -m json.tool "$FIELD_DIR/config.json" >/dev/null
python3 -m json.tool "$FIELD_DIR/manual_field.json" >/dev/null
chmod +x "$REPO_ROOT/run_mjx_et6.sh"

if [[ "$BUILD_TARGET" == true ]]; then
    if [[ ! -d "$REPO_ROOT/library/inference_runtime/onnxruntime/include" ]]; then
        echo "ONNX Runtime is missing. Run: bash scripts/download_inference_runtime.sh onnx" >&2
        exit 1
    fi
    cmake -S "$REPO_ROOT/src/rl_sar" -B "$REPO_ROOT/cmake_build" \
        -DUSE_CMAKE=ON -DRL_SAR_ENABLE_TORCH=OFF -DCMAKE_BUILD_TYPE=Release
    cmake --build "$REPO_ROOT/cmake_build" --target rl_real_mjx_et6 \
        -j"$(nproc 2>/dev/null || echo 4)"
fi

systemctl --user daemon-reload
if [[ "$ENABLE_SERVICE" == true ]]; then
    [[ -f "$MODEL_PATH" ]] || {
        echo "Policy model not found: $MODEL_PATH" >&2
        exit 1
    }
    systemctl --user stop gamepad-pwm.service 2>/dev/null || true
    systemctl --user disable gamepad-pwm.service 2>/dev/null || true
    systemctl --user enable --now et6-field-controller.service
    systemctl --user is-active --quiet et6-field-controller.service
    echo "ET6 field selector enabled: hold X for manual or Y for policy mode."
else
    echo "ET6 runtime installed but not enabled. Re-run with --enable-service after adding policy.onnx."
fi
