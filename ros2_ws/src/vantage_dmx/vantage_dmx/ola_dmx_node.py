import rclpy
from rclpy.node import Node
from ola.ClientWrapper import ClientWrapper
from std_msgs.msg import String

UNIVERSE = 1
class DmxNode(Node):
    def __init__(self):
        super().__init__('dmx')
        self.wrapper = ClientWrapper()
        self.client = self.wrapper.Client()
        self.sub = self.create_subscription(String, '/vantage/lighting/cue', self.on_cue, 10)
        self.dmx = bytearray([0]*512)

    def on_cue(self, msg: String):
        try:
            kv = dict(p.split('=') for p in msg.data.split(';') if '=' in p)
            self.dmx[0] = max(0, min(255, int(kv.get('intensity', 0))))
            self.dmx[1] = max(0, min(255, int(int(kv.get('color_temp', 4000)) / 40)))
            self.client.SendDmx(UNIVERSE, self.dmx, lambda state: None)
        except Exception as e:
            self.get_logger().error(f"Failed to parse cue: {e}")

def main():
    rclpy.init()
    node = DmxNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.wrapper.RunOnce()
    finally:
        rclpy.shutdown()
