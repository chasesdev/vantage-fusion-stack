# Vantage Fusion Stack — R5 C + R5 II + 2× OAK‑D ToF + DMX

A runnable stack for your RTX 6000 workstation:
- **Holoscan app**: R5 C via YUAN/V4L2 → GPU convert → time‑align → **CUDA ToF→RGB projection (Z‑buffer + splat + confidence)** → ROS 2 egress → preview/record.
- **ROS 2 nodes**: DMX (OLA) + LTC decoder.
- **DepthAI launch**: two ToF cameras with **external FSIN**.
- **Calibration kit**: Charuco board generator.
- **Scripts**: V4L2 helpers, LTC piping.

> Assumptions: Ubuntu 22.04, CUDA‑capable RTX 6000, Holoscan SDK ≥ 2.4 (Python), ROS 2 Humble, OLA, depthai‑ros, libltc.
>
> Live ingest remains **4K clean HDMI** from R5 C (8K is RAW‑only to Atomos).

## Quick start

### 1) System packages
```bash
sudo apt-get update
sudo apt-get install -y ola libltc-dev ltc-tools v4l-utils python3-opencv python3-pip
pip install depthai cupy-cuda12x  # pick wheel matching your CUDA
# For Charuco generation (if cv2.aruco missing), use: pip install opencv-contrib-python
```

### 2) DepthAI ToF pair (FSIN)
Wire both OAK‑D ToF **FSIN** to the same TTL. Then:
```bash
# In ros2_ws
colcon build --packages-select vantage_dmx vantage_timecode vantage_depthai
source install/setup.bash
ros2 launch vantage_depthai two_tof.launch.xml
```

### 3) DMX & Timecode (optional)
```bash
ros2 run vantage_dmx vantage_dmx_node
ros2 run vantage_timecode vantage_timecode_node
```

### 4) Holoscan app (R5 C ingest → CUDA fuse → ROS2 egress)
Set your **YUAN** device path in `holoscan_app/config/app.yaml` (default `/dev/video2`), then:
```bash
python3 holoscan_app/app.py --config holoscan_app/config/app.yaml
```

### 5) V4L2 helpers
```bash
bash scripts/v4l2_list.sh
bash scripts/v4l2_set_r5c_4k60.sh /dev/video2
```

## Graph
```
R5C (V4L2) -> FormatConverter -> TimeAlignMuxOp <- Ros2DepthSubscribeOp (/camera/depth,...)
                                      |
                                      -> VideoTofFuseOp (CUDA, Zbuf+splat+conf) -> Ros2PublishOp -> Holoviz/Recorder
```

## Configure topics & calibration
- `holoscan_app/config/app.yaml` → `/camera/depth/image_rect_raw` & `/camera/confidence/image_rect` for depthai_ros_driver.
- Put solved intrinsics/extrinsics in `holoscan_app/config/calibration.json`.

## Notes
- `Ros2PublishOp` publishes `sensor_msgs/Image` by default. Set `use_nitros: true` for NITROS (falls back gracefully).
- `VideoStreamRecorderOp` writes to `/data/cases/case_001` — adjust as needed.
