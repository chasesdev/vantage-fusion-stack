import os, sys
sys.path.append(os.path.dirname(__file__))

# Import ops to register decorator-defined operators
from ops.time_align_mux import time_align_mux  # noqa: F401
from ops.video_tof_fuse import video_tof_fuse  # noqa: F401
from ops.ros2_publish import ros2_publish      # noqa: F401
from ops.ros2_depth_sub import ros2_depth_subscribe  # noqa: F401
from ops.biomarker_score import biomarker_score      # noqa: F401

from holoscan.core import Application
from holoscan.operators import FormatConverterOp, HolovizOp, VideoStreamRecorderOp

try:
    from holoscan.operators import V4L2VideoCaptureOp
except Exception:
    from holoscan.ops import V4L2VideoCaptureOp  # fallback for older SDKs

class PortraitFusionApp(Application):
    def compose(self):
        r5c = V4L2VideoCaptureOp(self, name="r5c_v4l")
        fmt = FormatConverterOp(self, name="fmt", out_dtype="rgb888")

        align = self.make_operator("TimeAlignMuxOp", name="align", tolerance_ns=10_000_000)
        tof_src = self.make_operator("Ros2DepthSubscribeOp", name="tof_src",
                                     depth_topic="/camera/depth/image_rect_raw",
                                     conf_topic="/camera/confidence/image_rect",
                                     conf_min=0)

        fuse  = self.make_operator("VideoTofFuseOp", name="fuse",
                                   calibration_path="holoscan_app/config/calibration.json",
                                   z_min=0.3, z_max=2.0, alpha=0.6,
                                   splat_radius=1, conf_min=0)

        ros_pub = self.make_operator("Ros2PublishOp", name="ros_pub",
                                     topic_name="/vantage/fused/image", use_nitros=False)

        viz = HolovizOp(self, name="viz")
        rec = VideoStreamRecorderOp(self, name="rec")

        self.add_flow(r5c, fmt)
        self.add_flow(fmt, align)
        self.add_flow(tof_src, align)
        self.add_flow(align, fuse)
        self.add_flow(fuse, ros_pub)
        self.add_flow(fuse, viz)
        self.add_flow(fuse, rec)

if __name__ == "__main__":
    app = PortraitFusionApp()
    app.config("holoscan_app/config/app.yaml")
    app.run()
