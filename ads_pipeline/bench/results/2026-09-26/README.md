# LiDAR→Camera Projection Benchmark — 2026-09-26

Throughput, latency, dropped frames and CPU/RAM of the `projection_node` pipeline
(camera frame → LiDAR projection → depth overlay output) at **1920×1080**.

## Summary

| Question | Answer |
|---|---|
| 1080p at 30 FPS? | **Yes** with overlay published as a topic (29.9 FPS, 0.17% drop). **No** with PNG output (9–14 FPS). |
| 1080p at 60 FPS? | **Yes** with topic output (60.1 FPS, 0.10% drop, 8.8 ms p95 processing). Ceiling ≈ 79 FPS. |
| p95 latency | Processing **8.8 ms**; end-to-end (publish → overlay out) **119–196 ms**, dominated by ROS transport of 6 MB images. |
| Dropped frames | **0.09%** live (3,512 / 3,515) with topic output; **61–80%** with PNG output. |
| CPU / RAM | Projection node ~52% of one core, 183 MB steady / 227 MB peak. `sensor_manager` saturates one core (~100%). |

**Main finding:** the projection itself is cheap (~3 ms). Writing a PNG every frame
(`save_output: true`, the default) costs 60–130 ms and caps the node at 10–15 FPS.

## Setup

| | |
|---|---|
| CPU | Intel Core Ultra 9 275HX (24 cores) |
| GPU | NVIDIA GeForce RTX 5070 Ti Laptop |
| RAM | 31 GB |
| Software | ROS 2 Humble (default RMW, Fast DDS), CARLA 0.9.16 (Town10HD_Opt, `-RenderOffScreen`), Python 3.10, OpenCV 4.11, NumPy 1.26 |
| Sensors | Camera 1920×1080, FOV 90; LiDAR 32 ch, 100k pts/s, 20 Hz (config_carla.yaml) |

**Replay runs** — `bench_replay` plays 100 camera frames + LiDAR sweeps recorded from CARLA
(looped) at a fixed rate, 3,000 frames per run, stamped with wall-clock time at publish.
Isolates the pipeline from CARLA's render rate.

**Live runs** — CARLA → `sensor_manager` → `projection_node` for 90 s. CARLA runs
asynchronously and produced ~39 camera FPS. `sensor_manager` stamps with wall-clock time
(`bench.wall_stamps`) so end-to-end latency is measured from the CARLA sensor callback.

Output modes:
- **topic** — `save_output:=false overlay_topic:=/carla/camera/rgb/overlay`
- **png** — `save_output:=true` (current default: overwrites one PNG per frame)

## Results

| run | published | received | processed | drop | FPS | proc p50 | proc p95 | proc p99 | e2e p50 | e2e p95 | e2e p99 | CPU % avg/peak | RSS MB steady/peak |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 30fps_topic | 3000 | 2995 | 2995 | 0.17% | 29.9 | 7.5 | 9.1 | 10.3 | 34.2 | 195.7 | 255.4 | 45/69 | 184/232 |
| 60fps_topic | 3000 | 2997 | 2997 | 0.10% | 60.1 | 4.1 | 8.8 | 9.7 | 29.6 | 118.7 | 146.4 | 53/106 | 237/238 |
| 120fps_topic | 3000 | 1981 | 1981 | 33.97% | 79.2 | 8.3 | 8.9 | 9.2 | 91.8 | 95.7 | 97.5 | 92/112 | 219/235 |
| 30fps_png | 3000 | 938 | 938 | 68.73% | 9.3 | 109.4 | 136.2 | 171.0 | 425.8 | 465.6 | 538.1 | 69/95 | 215/226 |
| 60fps_png | 3000 | 588 | 588 | 80.40% | 11.6 | 69.7 | 139.7 | 151.2 | 233.7 | 303.0 | 425.4 | 81/107 | 214/226 |
| live_topic | 3515 | 3513 | 3512 | 0.09% | 41.1 | 7.2 | 8.8 | 9.6 | 27.7 | 153.3 | 186.5 | 52/90 | 183/227 |
| live_png | 3102 | 1204 | 1203 | 61.22% | 14.0 | 69.4 | 79.1 | 103.9 | 333.2 | 365.5 | 459.5 | 97/106 | 220/221 |

Latencies in ms. CPU and RSS are for `projection_node`; CPU is % of one core.

- **proc** — time inside the camera callback (LiDAR decode → output).
- **e2e** — publisher timestamp → overlay output done (includes ROS transport and queueing).
- **drop** — (published − processed) / published. Frames lost in the subscription queue (depth 10)
  never reach the callback, so they appear as published − received.
- **FPS** — processed frames / time between first and last processed frame.

### Where the time goes (p95 ms, live runs)

| stage | live_topic | live_png |
|---|---|---|
| lidar_decode (ros2_numpy) | 0.0 | 0.0 |
| project | 0.2 | 0.2 |
| colorize | 0.3 | 0.2 |
| image_decode (cv_bridge) | 2.2 | 0.9 |
| overlay (draw dots) | 1.1 | 1.1 |
| output (publish / PNG write) | 6.6 | **76.3** |
| **total** | 8.8 | 79.1 |

Transport (e2e − proc) in the replay topic runs: **~25 ms p50, 110–190 ms p95**.
It is not warm-up: the tail persists after excluding the first 100 frames.

### Whole-pipeline resources (live runs)

| process | live_topic CPU avg/peak | live_topic RSS | live_png CPU avg/peak | live_png RSS |
|---|---|---|---|---|
| CARLA server | 160% / 233% | 3.4 GB | 134% / 225% | 3.4 GB |
| sensor_manager | 102% / 109% | 492 / 527 MB | 91% / 105% | 526 / 527 MB |
| projection_node | 52% / 90% | 183 / 227 MB | 97% / 106% | 220 / 221 MB |
| system (all 24 cores) | 14% | — | 14% | — |

## Findings

1. **PNG output is the bottleneck.** 60–130 ms per 1080p `cv2.imwrite` limits the node to 10–15 FPS
   and drops 61–80% of frames. Publishing the overlay instead takes 6.6 ms p95.
2. **The projection supports 60 FPS at 1080p** with 8.8 ms p95 processing and 0.10% drops; the
   single-threaded node saturates at about 79 FPS.
3. **ROS transport of large images drives tail latency.** Each 6.2 MB `bgr8` image adds ~25 ms p50
   and up to ~190 ms p95 between processes with the default Fast DDS setup. Options: run the nodes
   in one process, configure shared-memory transport, or compress the images.
4. **`sensor_manager` is the next ceiling.** It uses a full core converting and publishing 1080p
   frames from CARLA's callback thread at ~39 FPS.
5. Frame drops are silent: the depth-10 subscription queue overwrites frames while the node is busy.
   The 2–3 missing frames in topic runs include the first frame, which arrives before any LiDAR sweep.

## Reproduce

```bash
source ~/carla_ws/install/setup.bash
cd ~/carla_ws/src/Autonomous_driving_Carla_simulator/ads_pipeline

# record input (CARLA running): 15 s of camera + LiDAR
ros2 launch ads_pipeline sensor_launch.py &
ros2 bag record /carla/camera/rgb/image /carla/lidar/points -o /tmp/capture_bag   # Ctrl+C after ~15 s

# replay: <bag> <out_dir> <fps> <frames> <save_png> [overlay_topic]
bench/run_projection_bench.sh /tmp/capture_bag /tmp/runs/60fps_topic 60 3000 false /carla/camera/rgb/overlay

# live (CARLA running): <out_dir> <seconds> <save_png> [overlay_topic]
bench/run_live_bench.sh /tmp/runs/live_topic 90 false /carla/camera/rgb/overlay

python3 bench/report.py /tmp/runs/*
```

Each run folder in this directory contains `summary.json`, `published.json`, `resources.json`,
`frames.csv` (per-frame stage timings) and `resources.csv` (0.5 s CPU/RAM samples).
The recorded input bag (3.7 GB, 14.6 s) is not included.
