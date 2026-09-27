#!/usr/bin/env bash
# Benchmark the LiDAR->camera projection node by replaying a recorded bag at a fixed rate.
#
#   bench/run_projection_bench.sh <bag> <out_dir> <fps> <frames> <save_png:true|false> [overlay_topic]
#
# Writes to <out_dir>: summary.json + frames.csv (projection node), published.json (replay),
# resources.json + resources.csv (CPU/RAM).
set -eu
BAG=$1; OUT=$2; FPS=$3; FRAMES=$4; SAVE=$5; OVERLAY=${6:-}
mkdir -p "$OUT"
# run the installed executables directly: `ros2 run` does not forward SIGINT to its child
BIN=$(ros2 pkg prefix ads_pipeline)/lib/ads_pipeline
EXTRA=()
[ -n "$OVERLAY" ] && EXTRA=(-p overlay_topic:="$OVERLAY")

"$BIN/projection_node" --ros-args \
    -p bench.stats_dir:="$OUT" -p bench.wall_stamps:=true \
    -p save_output:="$SAVE" -p output_path:="$OUT/projection_result.png" "${EXTRA[@]}" \
    > "$OUT/projection.log" 2>&1 &
NODE=$!
"$BIN/bench_resources" --out "$OUT" --match projection_node bench_replay > "$OUT/resources.log" 2>&1 &
MON=$!
sleep 2

"$BIN/bench_replay" --bag "$BAG" --fps "$FPS" --frames "$FRAMES" --stats-dir "$OUT" > "$OUT/replay.log" 2>&1
sleep 2  # let the node drain its queue

kill -INT $MON $NODE
wait $NODE $MON 2>/dev/null || true
