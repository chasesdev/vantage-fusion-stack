import rclpy, subprocess, os, re, time
from rclpy.node import Node
from std_msgs.msg import String

TC_RE = re.compile(r'TC:\s*(\d\d:\d\d:\d\d:\d\d)')
class LtcNode(Node):
    def __init__(self):
        super().__init__('ltc_decoder')
        self.pub = self.create_publisher(String, '/vantage/timecode', 10)
        device = os.environ.get('LTC_AUDIO_DEVICE', 'hw:0,0')
        cmd = f"arecord -f S16_LE -r 48000 -c 1 -D {device} -q | ltcdump -"
        self.get_logger().info(f"Spawning: {cmd}")
        self.proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

    def spin_once(self):
        if self.proc.stdout is None: return
        line = self.proc.stdout.readline()
        if not line: time.sleep(0.01); return
        m = TC_RE.search(line)
        if m:
            tc = m.group(1)
            msg = String(); msg.data = f"{tc} epoch_ns={time.time_ns()}"
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = LtcNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.spin_once()
    finally:
        rclpy.shutdown()
