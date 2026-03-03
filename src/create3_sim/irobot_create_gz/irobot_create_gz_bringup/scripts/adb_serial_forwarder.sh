#!/bin/bash
# =============================================================================
# ADB Serial Forwarder for PuduBot LD06 Lidar
#
# Streams raw lidar bytes from the PuduBot's /dev/ttyS2 over ADB's USB
# connection using 'adb exec-out'. Output goes to stdout so it can be piped
# directly to the LD06 driver node or to a TCP server.
#
# Usage:
#   # Direct pipe to driver (simplest):
#   bash adb_serial_forwarder.sh | python3 ld06_driver_node.py --stdin
#
#   # Or serve over TCP with socat:
#   bash adb_serial_forwarder.sh | socat - TCP-LISTEN:4001,reuseaddr,fork
#
#   # Or serve over TCP natively:
#   bash adb_serial_forwarder.sh --tcp 4001
# =============================================================================

set -e

# ── Configuration ───────────────────────────────────────────────────────────
ADB_SERIAL="${ADB_SERIAL:-52D3602B1341763}"
LIDAR_DEV="${LIDAR_DEV:-/dev/ttyS2}"
TCP_PORT=""
RETRY_DELAY=3

# ── Parse arguments ─────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --serial) ADB_SERIAL="$2"; shift 2 ;;
        --dev)    LIDAR_DEV="$2"; shift 2 ;;
        --tcp)    TCP_PORT="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 [--serial ADB_SERIAL] [--dev DEVICE] [--tcp PORT]"
            echo ""
            echo "  --serial   ADB device serial (default: $ADB_SERIAL)"
            echo "  --dev      Lidar serial device on robot (default: $LIDAR_DEV)"
            echo "  --tcp      Serve data over TCP at this port instead of stdout"
            echo ""
            echo "Without --tcp, raw lidar bytes go to stdout for piping."
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ── Helper functions ────────────────────────────────────────────────────────
cleanup() {
    echo "[forwarder] Cleaning up..." >&2
    exit 0
}

trap cleanup EXIT INT TERM

check_adb() {
    if ! adb -s "$ADB_SERIAL" get-state >/dev/null 2>&1; then
        echo "[forwarder] ERROR: ADB device $ADB_SERIAL not found." >&2
        echo "[forwarder] Available devices:" >&2
        adb devices -l >&2
        return 1
    fi
    return 0
}

# ── Main ────────────────────────────────────────────────────────────────────
echo "=============================================" >&2
echo "  PuduBot LD06 Lidar ADB Forwarder"          >&2
echo "=============================================" >&2
echo "  ADB Serial : $ADB_SERIAL"                   >&2
echo "  Lidar Device: $LIDAR_DEV"                    >&2
if [[ -n "$TCP_PORT" ]]; then
    echo "  Output      : TCP port $TCP_PORT"        >&2
else
    echo "  Output      : stdout (pipe mode)"        >&2
fi
echo "=============================================" >&2

# 1. Check ADB connectivity
echo "[forwarder] Checking ADB connection..." >&2
if ! check_adb; then
    exit 1
fi
echo "[forwarder] ADB device connected." >&2

# 2. Stream loop with auto-reconnect
while true; do
    echo "[forwarder] Starting lidar data stream from $LIDAR_DEV..." >&2

    if [[ -n "$TCP_PORT" ]]; then
        # TCP mode: use socat if available, otherwise manual approach
        if command -v socat >/dev/null 2>&1; then
            adb -s "$ADB_SERIAL" exec-out "cat $LIDAR_DEV" | \
                socat - TCP-LISTEN:"$TCP_PORT",reuseaddr 2>/dev/null || true
        else
            echo "[forwarder] ERROR: socat not found. Install it with: sudo apt install socat" >&2
            echo "[forwarder] Or use pipe mode instead (without --tcp)." >&2
            exit 1
        fi
    else
        # Stdout pipe mode: simplest, most reliable
        adb -s "$ADB_SERIAL" exec-out "cat $LIDAR_DEV" || true
    fi

    echo "[forwarder] Stream ended. Reconnecting in ${RETRY_DELAY}s..." >&2
    sleep "$RETRY_DELAY"

    if ! check_adb; then
        echo "[forwarder] ADB connection lost. Waiting for device..." >&2
        adb -s "$ADB_SERIAL" wait-for-device
        echo "[forwarder] Device reconnected." >&2
    fi
done
