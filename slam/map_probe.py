import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
n = Node("map_probe")
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
def cb(m):
    occ = sum(1 for c in m.data if c > 50)
    free = sum(1 for c in m.data if c == 0)
    print(f"MAP {m.info.width}x{m.info.height} @ {m.info.resolution} m  "
          f"occupied={occ} free={free} stamp={m.header.stamp.sec}")
n.create_subscription(OccupancyGrid, "/map", cb, qos)
end = time.time() + 6
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.3)
n.destroy_node()
rclpy.shutdown()
