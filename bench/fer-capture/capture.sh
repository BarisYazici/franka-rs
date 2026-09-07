#!/usr/bin/env bash
# Capture the FCI UDP traffic of one Franka arm with tcpdump, for offline analysis
# with analyze.py.
#
#   capture.sh start <robot-ip> <iface> <out.pcap>   # background, returns once armed
#   capture.sh <robot-ip> <iface> <out.pcap>         # same thing (start is implied)
#   capture.sh status <out.pcap>
#   capture.sh stop   <out.pcap>                     # SIGINT, then wait for the flush
#
# Side files next to <out.pcap>:
#   <out.pcap>.pid   the tcpdump pid while it runs
#   <out.pcap>.log   tcpdump's stderr (the "listening on ..." marker and the counters)
#   <out.pcap>.meta  what was captured and, importantly, *which timestamp source*
#
# tcpdump needs cap_net_raw+ep (or root).  Grant it once, outside this script:
#   sudo setcap cap_net_raw,cap_net_admin=eip "$(command -v tcpdump)"
# This script never elevates privileges by itself.

set -euo pipefail

START_TIMEOUT=${START_TIMEOUT:-10}   # seconds to wait for the "listening on" marker
STOP_TIMEOUT=${STOP_TIMEOUT:-15}     # seconds to wait for tcpdump to flush and exit

die() { printf 'capture.sh: %s\n' "$*" >&2; exit 1; }
info() { printf 'capture.sh: %s\n' "$*"; }

usage() {
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
}

# Wait until tcpdump has printed its "listening on <iface>" marker, or died.
# $1 pid, $2 log file
wait_for_marker() {
    local pid=$1 log=$2 i=0
    while [ "$i" -lt $((START_TIMEOUT * 20)) ]; do
        if grep -q 'listening on' "$log" 2>/dev/null; then
            return 0
        fi
        if ! kill -0 "$pid" 2>/dev/null; then
            return 1
        fi
        sleep 0.05
        i=$((i + 1))
    done
    return 1
}

# $1 robot-ip, $2 iface, $3 out.pcap
do_start() {
    local robot=$1 iface=$2 out=$3
    local pidfile="$out.pid"
    local log="$out.log"
    local meta="$out.meta"

    command -v tcpdump >/dev/null || die "tcpdump not found"
    if [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
        die "a capture is already running (pid $(cat "$pidfile"), $pidfile)"
    fi
    local dir
    dir=$(dirname -- "$out")
    [ -d "$dir" ] || die "no such directory: $dir"
    rm -f "$pidfile"

    # tcpdump's own capture filter: both directions of every UDP flow with the arm.
    local filter="udp and host $robot"
    local base=(tcpdump -i "$iface" --time-stamp-precision=nano -B 65536 -s 0
                -w "$out" "$filter")

    # Try NIC hardware receive timestamps first; they are immune to any delay
    # between the wire and the kernel.  `ethtool -T <iface>` lists whether the NIC
    # has hardware-receive.  If tcpdump/libpcap rejects the option we silently fall
    # back to kernel (software) timestamps and record which one we got.
    local mode pid
    : >"$log"
    mode="adapter_unsynced"
    "${base[@]}" -j adapter_unsynced >>"$log" 2>&1 &
    pid=$!
    if wait_for_marker "$pid" "$log"; then
        # tcpdump only *warns* when the device cannot do the requested time stamp
        # type and keeps capturing with the kernel's clock - do not claim hardware
        # timestamps in that case.
        if grep -q "trying to set timestamp type" "$log"; then
            info "-j adapter_unsynced not supported by $iface, tcpdump fell back"
            mode="host (software, adapter_unsynced refused)"
        fi
    else
        wait "$pid" 2>/dev/null || true
        info "-j adapter_unsynced rejected, falling back to software timestamps"
        info "  ($(tail -n 2 "$log" | tr '\n' ' '))"
        mode="host (software)"
        : >"$log"
        "${base[@]}" >>"$log" 2>&1 &
        pid=$!
        wait_for_marker "$pid" "$log" || {
            wait "$pid" 2>/dev/null || true
            die "tcpdump failed to start: $(cat "$log")"
        }
    fi
    echo "$pid" >"$pidfile"

    {
        echo "started        : $(date -Is)"
        echo "robot          : $robot"
        echo "interface      : $iface"
        echo "output         : $out"
        echo "filter         : $filter"
        echo "timestamp mode : $mode"
        echo "tcpdump        : $(tcpdump --version 2>&1 | head -1)"
        echo "pid            : $pid"
        echo "--- ethtool -T $iface ---"
        ethtool -T "$iface" 2>&1 || echo "(ethtool unavailable)"
    } >"$meta"

    info "capturing on $iface, filter '$filter', timestamps: $mode (pid $pid)"
    info "stop with: $0 stop $out"
}

# $1 out.pcap
do_stop() {
    local out=$1
    local pidfile="$out.pid"
    local pid i=0
    [ -f "$pidfile" ] || die "no pid file $pidfile - is a capture running?"
    pid=$(cat "$pidfile")
    if ! kill -0 "$pid" 2>/dev/null; then
        rm -f "$pidfile"
        die "pid $pid is gone; the capture already stopped (see $out.log)"
    fi
    # SIGINT makes tcpdump flush the buffer, print the counters and exit 0.
    kill -INT "$pid"
    while kill -0 "$pid" 2>/dev/null; do
        if [ "$i" -ge $((STOP_TIMEOUT * 20)) ]; then
            info "tcpdump did not exit after ${STOP_TIMEOUT}s, sending SIGTERM"
            kill -TERM "$pid" 2>/dev/null || true
            break
        fi
        sleep 0.05
        i=$((i + 1))
    done
    rm -f "$pidfile"
    {
        echo "stopped        : $(date -Is)"
        grep -E 'packets (captured|received|dropped)' "$out.log" || true
    } >>"$out.meta"
    info "stopped; $(du -h "$out" | cut -f1) in $out"
    grep -E 'packets (captured|received|dropped)' "$out.log" || true
    info "next: python3 $(dirname -- "$0")/analyze.py $out"
}

# $1 out.pcap
do_status() {
    local out=$1
    local pidfile="$out.pid"
    if [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
        info "running (pid $(cat "$pidfile")), $(du -h "$out" 2>/dev/null | cut -f1) captured"
        [ -f "$out.meta" ] && grep -E '^(timestamp mode|filter|interface)' "$out.meta"
        return 0
    fi
    info "not running"
    [ -f "$out" ] && info "last capture: $out ($(du -h "$out" | cut -f1))"
    return 1
}

[ $# -ge 1 ] || usage
case "${1:-}" in
    start)  [ $# -eq 4 ] || usage; do_start "$2" "$3" "$4" ;;
    stop)   [ $# -eq 2 ] || usage; do_stop "$2" ;;
    status) [ $# -eq 2 ] || usage; do_status "$2" ;;
    -h|--help|help) usage ;;
    *)      [ $# -eq 3 ] || usage; do_start "$1" "$2" "$3" ;;
esac
