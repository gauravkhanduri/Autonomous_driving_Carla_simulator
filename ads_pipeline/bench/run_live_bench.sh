#!/usr/bin/env bash
# Benchmark the full live pipeline: CARLA -> sensor_manager -> projection_node.
# A CARLA server must already be running on localhost:2000.
#
#   bench/run_live_bench.sh <out_dir> <seconds> <save_png:true|false> [overlay_topic]
#
# sensor_manager stamps with wall-clock time here, so e2e latency = CARLA sensor callback -> overlay out.
set -eu
OUT=$1; SECS=$2; SAVE=$3; OVERLAY=${4:-}
mkdir -p "$OUT"
PREFIX=$(ros2 pkg prefix ads_pipeline)
BIN=$PREFIX/lib/ads_pipeline
EXTRA=()
[ -n "$OVERLAY" ] && EXTRA=(-p overlay_topic:="$OVERLAY")

"$BIN/bench_resources" --out "$OUT" --match projection_node sensor_manager CarlaUE4-Linux \
    > "$OUT/resources.log" 2>&1 &
MON=$!
"$BIN/projection_node" --ros-args \
    -p bench.stats_dir:="$OUT" -p bench.wall_stamps:=true \
    -p save_output:="$SAVE" -p output_path:="$OUT/projection_result.png" "${EXTRA[@]}" \
    > "$OUT/projection.log" 2>&1 &
NODE=$!
"$BIN/sensor_manager" --ros-args -r __node:=sensor_manager \
    --params-file "$PREFIX/share/ads_pipeline/config/config_carla.yaml" \
    -p bench.stats_dir:="$OUT" -p bench.wall_stamps:=true \
    > "$OUT/sensor_manager.log" 2>&1 &
SM=$!

stop() {  # SIGINT, escalating to SIGTERM if the process has not exited after 20 s
    kill -INT "$@" 2>/dev/null || true
    for _ in $(seq 20); do kill -0 "$@" 2>/dev/null || break; sleep 1; done
    kill -TERM "$@" 2>/dev/null || true
    wait "$@" 2>/dev/null || true
}

sleep "$SECS"
stop $SM                             # stop publishing first
sleep 2                              # let the projection node drain its queue
stop $NODE $MON
