#!/bin/bash
# sync_to_jetson.sh — Sync local source code to Qmini-orin Jetson and rebuild
#
# Usage:
#   ./scripts/sync_to_jetson.sh            # sync src/ + policy/ then remote build
#   ./scripts/sync_to_jetson.sh --no-build # sync only, skip remote build
#
# Remote target (matches ~/.ssh/config Host Qmini-orin):
#   User:     qmini
#   Host:     172.20.10.3
#   Root:     /home/qmini/sim2real/rl_sar
#
# Prerequisites:
#   ssh-copy-id qmini@172.20.10.3   (one-time passwordless SSH setup)

set -e

REMOTE_HOST="qmini@172.20.10.3"
REMOTE_ROOT="/home/qmini/sim2real/rl_sar"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOCAL_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"

# ── Parse arguments ──────────────────────────────────────────────────────────
BUILD=true
for arg in "$@"; do
    case "$arg" in
        --no-build) BUILD=false ;;
        -h|--help)
            echo "Usage: $0 [--no-build]"
            echo "  --no-build   Sync files only, skip remote './build.sh -m'"
            exit 0
            ;;
        *)
            echo "[sync] Unknown argument: $arg"
            exit 1
            ;;
    esac
done

# ── Check SSH connectivity ────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo "  sync_to_jetson.sh"
echo "  Local  : ${LOCAL_ROOT}"
echo "  Remote : ${REMOTE_HOST}:${REMOTE_ROOT}"
echo "════════════════════════════════════════════════════════"
echo ""

echo "[sync] Checking SSH connectivity to ${REMOTE_HOST}..."
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "${REMOTE_HOST}" "exit" 2>/dev/null; then
    echo "[sync] ✗ Cannot connect to ${REMOTE_HOST}."
    echo "[sync]   Please run:  ssh-copy-id ${REMOTE_HOST}"
    exit 1
fi
echo "[sync] ✓ SSH OK"
echo ""

# ── Step 1: rsync src/ ────────────────────────────────────────────────────────
echo "[sync] Step 1/3 — Syncing src/ ..."
rsync -avz --delete \
    --exclude='*.o' \
    --exclude='*.a' \
    --exclude='*.so' \
    --exclude='CMakeFiles/' \
    --exclude='__pycache__/' \
    "${LOCAL_ROOT}/src/" \
    "${REMOTE_HOST}:${REMOTE_ROOT}/src/"
echo "[sync] ✓ src/ synced"
echo ""

# ── Step 2: rsync policy/Qmini/ (yaml configs, no large model files) ─────────
echo "[sync] Step 2/3 — Syncing policy/Qmini/ ..."
rsync -avz \
    --exclude='*.onnx' \
    --exclude='*.pt' \
    --exclude='*.pth' \
    "${LOCAL_ROOT}/policy/Qmini/" \
    "${REMOTE_HOST}:${REMOTE_ROOT}/policy/Qmini/"
echo "[sync] ✓ policy/Qmini/ synced (model weights excluded)"
echo ""

# ── Step 2b: rsync scripts/ and README ───────────────────────────────────────
echo "[sync] Step 2b/3 — Syncing scripts/ and README ..."
rsync -avz \
    --exclude='__pycache__/' \
    "${LOCAL_ROOT}/scripts/" \
    "${REMOTE_HOST}:${REMOTE_ROOT}/scripts/"
rsync -avz \
    "${LOCAL_ROOT}/README.md" \
    "${LOCAL_ROOT}/README_CN.md" \
    "${REMOTE_HOST}:${REMOTE_ROOT}/"
echo "[sync] ✓ scripts/ and README synced"
echo ""

# ── Step 3: remote build ──────────────────────────────────────────────────────
if [ "$BUILD" = true ]; then
    echo "[sync] Step 3/3 — Remote build on Jetson (./build.sh -m) ..."
    echo "[sync] This may take a few minutes..."
    echo ""
    ssh "${REMOTE_HOST}" "cd ${REMOTE_ROOT} && ./build.sh -m"
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  ✓ All done! Binaries updated on Jetson:"
    echo "    ${REMOTE_ROOT}/cmake_build/bin/rl_real_qmini"
    echo "    ${REMOTE_ROOT}/cmake_build/bin/rl_calib_qmini"
    echo "    ${REMOTE_ROOT}/cmake_build/bin/rl_mirror_qmini"
    echo "════════════════════════════════════════════════════════"
else
    echo "[sync] Step 3/3 — Skipped (--no-build)"
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  ✓ Sync done. To build on Jetson, run:"
    echo "    ssh ${REMOTE_HOST} 'cd ${REMOTE_ROOT} && ./build.sh -m'"
    echo "════════════════════════════════════════════════════════"
fi
echo ""
