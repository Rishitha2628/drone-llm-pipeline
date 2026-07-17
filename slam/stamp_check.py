"""Stamp checker — prints the header stamps of /odom and /cloud_clean side by
side with /clock, once per second. This is the ground truth the approx-sync
filter sees; if the two stamps differ by more than ~0.2 s, sync can never match.

Run:  python3 -m slam.stamp_check
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock


def _t(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class StampCheck(Node):
    def __init__(self):
        super().__init__("stamp_check")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.odom_t = None
        self.cloud_t = None
        self.clock_t = None
        self.odom_n = 0
        self.cloud_n = 0
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.create_subscription(PointCloud2, "/cloud_clean", self.on_cloud, qos)
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.create_timer(1.0, self.report)
        print("listening on /odom, /cloud_clean, /clock ... Ctrl+C to stop")

    def on_odom(self, msg):
        self.odom_t = _t(msg.header.stamp)
        self.odom_n += 1

    def on_cloud(self, msg):
        self.cloud_t = _t(msg.header.stamp)
        self.cloud_n += 1

    def on_clock(self, msg):
        self.clock_t = _t(msg.clock)

    def report(self):
        o = f"{self.odom_t:.3f}" if self.odom_t is not None else "NONE"
        c = f"{self.cloud_t:.3f}" if self.cloud_t is not None else "NONE"
        k = f"{self.clock_t:.3f}" if self.clock_t is not None else "NONE"
        diff = (f"{abs(self.odom_t - self.cloud_t):.3f}"
                if self.odom_t is not None and self.cloud_t is not None else "n/a")
        print(f"clock={k} | odom={o} (n={self.odom_n}) | "
              f"cloud={c} (n={self.cloud_n}) | |odom-cloud|={diff}")


def main():
    rclpy.init()
    node = StampCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
