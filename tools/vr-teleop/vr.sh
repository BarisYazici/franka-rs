#!/bin/bash
# Run the Quest bridge and the teleop processes on the machine next to the robot,
# so no laptop sits in the control path. The headset is read over ADB: USB when it
# is plugged into that machine (lower latency, far less jitter), Wi-Fi otherwise.
#
#   ./vr.sh status          what is running, and how the headset is attached
#   ./vr.sh bridge          start the bridge (idempotent; USB if present, else Wi-Fi)
#   ./vr.sh teleop L|R ...  run one arm's teleop in the foreground; extra args pass through
#   ./vr.sh probe [secs]    what the bridge is publishing right now, both channels
#   ./vr.sh test            the offline test suite of this checkout (no headset, no robot)
#   ./vr.sh stop            stop the bridge (teleop is stopped with Ctrl-C)
#
# A thin wrapper around the franka-vr-* console scripts of `pip install franka-vr-teleop`:
# they must be on PATH, or in VENV/bin. Settings: VENV, QUEST_IP (required for Wi-Fi,
# unused on USB), ZENOH, EP_L/EP_R, CLIENT_ID_L/CLIENT_ID_R, BRIDGE_LOG. An optional,
# gitignored vr.env beside this script is sourced first, so a site keeps its headset IP
# and arm client ids out of the repository.
#
# Patterns below are bracketed (franka-vr-[b]ridge) so pgrep/pkill cannot match this
# script's own command line, or an ssh invocation that names it, and end at a space or the
# line's end so a command naming the log (tail -f .../franka-vr-bridge.log) does not either.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
[ -f "$HERE/vr.env" ] && . "$HERE/vr.env"

[ -n "${VENV:-}" ] && PATH="$VENV/bin:$PATH"
QUEST="${QUEST_IP:-}"
ZENOH="${ZENOH:-tcp/127.0.0.1:7447}"
CLIENT_ID_L="${CLIENT_ID_L:-7201}"
CLIENT_ID_R="${CLIENT_ID_R:-7202}"
EP_L="${EP_L:-tcp://127.0.0.1:5560}"
EP_R="${EP_R:-tcp://127.0.0.1:5570}"
BRIDGE_LOG="${BRIDGE_LOG:-${TMPDIR:-/tmp}/franka-vr-bridge.log}"
BRIDGE_PAT="franka-vr-[b]ridge( |$)"

need_quest_ip() {
    [ -n "$QUEST" ] && return 0
    echo "no headset on USB, and QUEST_IP is not set."
    echo "Plug the headset in over USB, or set the headset's IP for Wi-Fi:"
    echo "    QUEST_IP=<headset ip> $0 $1        (or put QUEST_IP=... in $HERE/vr.env)"
    echo "The headset reports it with: adb shell ip route"
    return 1
}

# usb | wifi | none
transport() {
    local d
    d=$(adb devices 2>/dev/null | awk '$2 == "device" {print $1}')
    case "$d" in
        "")                          echo none ;;
        *:5555)                      echo wifi ;;
        *"${QUEST:-__unset__}":*)    echo wifi ;;
        *)                           echo usb ;;
    esac
}

case "${1:-status}" in

status)
    t=$(transport)
    printf 'headset   '
    case "$t" in
        usb)  echo "USB      ($(adb devices | awk '$2=="device"{print $1}' | head -1))" ;;
        wifi) echo "Wi-Fi    (${QUEST:-unknown}:5555)" ;;
        none) if [ -n "$QUEST" ]; then
                  echo "NOT attached — plug in USB, or: adb connect $QUEST:5555"
              else
                  echo "NOT attached — plug in USB, or set QUEST_IP and run '$0 bridge'"
              fi ;;
    esac
    printf 'bridge    '
    pgrep -f "$BRIDGE_PAT" >/dev/null && echo "running  (pid $(pgrep -f "$BRIDGE_PAT" | tr '\n' ' '))" || echo "stopped"
    printf 'teleop    '
    pgrep -af "franka-vr-[t]eleop( |$)" || echo "none"
    printf 'node      '
    pgrep -a "franka-node-" | head -1 || echo "stopped"
    ;;

bridge)
    if pgrep -f "$BRIDGE_PAT" >/dev/null; then
        echo "bridge already running (pid $(pgrep -f "$BRIDGE_PAT" | tr '\n' ' '))"
        exit 0
    fi
    t=$(transport)
    if [ "$t" = none ]; then
        need_quest_ip bridge || exit 1
        adb connect "$QUEST:5555" >/dev/null 2>&1
        sleep 2
        t=$(transport)
    fi
    [ "$t" = none ] && { echo "no headset on USB or Wi-Fi"; exit 1; }
    set -- --controller l,r --endpoint "$EP_L,$EP_R" --no-keepalive
    [ "$t" = wifi ] && set -- "$@" --oculus-ip "$QUEST"
    echo "starting the bridge over $t, log in $BRIDGE_LOG"
    nohup franka-vr-bridge "$@" > "$BRIDGE_LOG" 2>&1 &
    sleep 5
    grep -iE "transport|ready|error" "$BRIDGE_LOG" | head -5
    ;;

teleop)
    arm="${2:-}"
    [ -z "$arm" ] && { echo "usage: $0 teleop L|R [extra franka-vr-teleop args]"; exit 2; }
    shift 2
    case "$arm" in
        L) cid="$CLIENT_ID_L" ;;
        R) cid="$CLIENT_ID_R" ;;
        *) echo "arm must be L or R"; exit 2 ;;
    esac
    exec franka-vr-teleop --arm "$arm" --client-id "$cid" --connect "$ZENOH" "$@"
    ;;

probe)
    exec franka-vr-probe --endpoint "$EP_L" --endpoint "$EP_R" --secs "${2:-8}"
    ;;

test)
    cd "$HERE" || exit 1
    rc=0
    python3 -m pytest -q || rc=$?
    PYTHON=python3 tests/teleop/run_tests.sh || rc=$?
    exit "$rc"
    ;;

stop)
    pkill -f "$BRIDGE_PAT" && echo "bridge stopped" || echo "bridge was not running"
    ;;

*)
    sed -n '2,20p' "$0"
    exit 2
    ;;
esac
