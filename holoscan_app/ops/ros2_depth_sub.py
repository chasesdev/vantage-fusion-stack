from holoscan.decorator import create_op, Output
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    ROS_OK = True
except Exception:
    ROS_OK = False
    Node = object

try:
    import cupy as cp
    HAS_CUPY = True
except Exception:
    import numpy as cp  # type: ignore
    HAS_CUPY = False

import numpy as np

class _DepthSub(Node):
    def __init__(self, depth_topic: str, conf_topic: Optional[str] = None):
        super().__init__('vantage_depth_sub')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.depth = None; self.conf = None; self.stamp_ns = None
        self.create_subscription(Image, depth_topic, self._on_depth, qos)
        if conf_topic:
            self.create_subscription(Image, conf_topic, self._on_conf, qos)

    def _on_depth(self, msg: Image):
        h, w = msg.height, msg.width
        enc = msg.encoding.lower()
        buf = memoryview(msg.data)
        if enc in ('32fc1','32fe1','32fc'):
            arr = np.frombuffer(buf, dtype=np.float32).reshape(h, w)
            depth_m = arr
        elif enc in ('16uc1','mono16'):
            arr = np.frombuffer(buf, dtype=np.uint16).reshape(h, w)
            depth_m = arr.astype(np.float32) / 1000.0
        else:
            return
        self.depth = depth_m
        self.stamp_ns = (msg.header.stamp.sec * 1_000_000_000) + msg.header.stamp.nanosec

    def _on_conf(self, msg: Image):
        h, w = msg.height, msg.width
        enc = msg.encoding.lower()
        buf = memoryview(msg.data)
        if enc in ('8uc1','mono8'):
            arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, w)
        elif enc in ('16uc1','mono16'):
            arr = (np.frombuffer(buf, dtype=np.uint16).reshape(h, w) / 256).astype(np.uint8)
        else:
            return
        self.conf = arr

_ros_inited = False
_ros_ctx: Optional[_DepthSub] = None

@create_op("Ros2DepthSubscribeOp")
def ros2_depth_subscribe(depth_topic: str = "/camera/depth/image_rect_raw",
                         conf_topic: str = "/camera/confidence/image_rect",
                         conf_min: int = 0) -> Output:
    """Source op: emits {'ts','depth','meta'} from ROS depth/conf topics."""
    global _ros_inited, _ros_ctx
    if not ROS_OK:
        return None
    if not _ros_inited:
        rclpy.init(args=None)
        _ros_ctx = _DepthSub(depth_topic, conf_topic or None)
        _ros_inited = True

    rclpy.spin_once(_ros_ctx, timeout_sec=0.0)
    if _ros_ctx.depth is None:
        return None

    depth = _ros_ctx.depth
    meta = {}
    if conf_topic and _ros_ctx.conf is not None and conf_min > 0:
        m = (_ros_ctx.conf >= int(conf_min)).astype(depth.dtype)
        depth = depth * m
        meta['conf'] = _ros_ctx.conf

    if HAS_CUPY:
        return {'ts': _ros_ctx.stamp_ns or 0, 'depth': cp.asarray(depth, dtype=cp.float32), 'meta': meta}
    else:
        return {'ts': _ros_ctx.stamp_ns or 0, 'depth': np.asarray(depth, dtype=np.float32), 'meta': meta}
