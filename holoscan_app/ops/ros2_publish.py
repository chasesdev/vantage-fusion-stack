from holoscan.decorator import create_op, Input, Output

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    ROS_OK = True
except Exception:
    ROS_OK = False
    Node = object

_HAS_NITROS = False
try:
    from isaac_ros_managed_nitros import ManagedNitrosPublisher
    from isaac_ros_nitros import NitrosImageBuilder
    _HAS_NITROS = True
except Exception:
    _HAS_NITROS = False

try:
    import numpy as np
    import cupy as cp
    HAS_CUPY = True
except Exception:
    import numpy as np
    HAS_CUPY = False

class _Ros2Ctx(Node):
    def __init__(self, node_name, topic_name, width, height, use_nitros=False):
        super().__init__(node_name)
        self.topic = topic_name
        self.use_nitros = use_nitros and _HAS_NITROS
        if self.use_nitros:
            self.pub = ManagedNitrosPublisher(self, self.topic, "nitros_image")
        else:
            self.pub = self.create_publisher(Image, self.topic, 10)
        self.width = width; self.height = height

    def publish_rgb8(self, img_rgb):
        if self.use_nitros:
            if not HAS_CUPY or not hasattr(img_rgb, "data"):
                self.get_logger().warn("NITROS requested but CuPy image not provided; falling back to CPU")
                return self._publish_cpu(img_rgb)
            try:
                builder = NitrosImageBuilder()
                builder.with_dimensions(self.height, self.width).with_encoding("rgb8")
                builder.with_gpu_data(img_rgb.data.ptr)
                self.pub.publish(builder.build()); return
            except Exception as e:
                self.get_logger().warn(f"NITROS publish failed: {e}; falling back to CPU")
                return self._publish_cpu(img_rgb)
        else:
            return self._publish_cpu(img_rgb)

    def _publish_cpu(self, img_rgb):
        if HAS_CUPY and hasattr(img_rgb, "get"):  # CuPy array
            img_np = img_rgb.get()
        else:
            img_np = np.asarray(img_rgb)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = img_np.shape[0], img_np.shape[1]
        msg.encoding = "rgb8"; msg.step = img_np.shape[1]*3
        msg.data = img_np.tobytes(order='C')
        self.pub.publish(msg)

_ros_node = None; _ros_inited = False

@create_op("Ros2PublishOp")
def ros2_publish(frame: Input, topic_name: str="/vantage/fused/image",
                 node_name: str="vantage_ros_pub", use_nitros: bool=False) -> Output:
    global _ros_node, _ros_inited
    if not ROS_OK or frame is None or 'img' not in frame:
        return frame
    img = frame['img']
    if not _ros_inited:
        rclpy.init(args=None)
        h, w = (img.shape[0], img.shape[1]) if hasattr(img, "shape") else (0, 0)
        _ros_node = _Ros2Ctx(node_name, topic_name, w, h, use_nitros=use_nitros)
        _ros_inited = True
    _ros_node.publish_rgb8(img)
    rclpy.spin_once(_ros_node, timeout_sec=0.0)
    return frame
