import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

OUT = "/home/rishi/challenge2-evidence/walls_map"

rclpy.init()
n = Node("map_saver_direct")
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
done = []

def cb(m):
    if done:
        return
    done.append(True)
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    # PGM: unknown(-1)->205, free(0..~25)->254, occupied(>65)->0
    with open(OUT + ".pgm", "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        for row in range(h - 1, -1, -1):          # pgm is top-down, grid is bottom-up
            for col in range(w):
                c = m.data[row * w + col]
                f.write(bytes([205 if c < 0 else (0 if c > 65 else (254 if c < 25 else 205))]))
    with open(OUT + ".yaml", "w") as f:
        f.write(f"image: walls_map.pgm\nresolution: {res}\n"
                f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    occ = sum(1 for c in m.data if c > 65)
    print(f"saved {OUT}.pgm/.yaml  ({w}x{h} @ {res:.2f} m, occupied={occ})")

n.create_subscription(OccupancyGrid, "/map", cb, qos)
end = time.time() + 6
while time.time() < end and not done:
    rclpy.spin_once(n, timeout_sec=0.3)
n.destroy_node(); rclpy.shutdown()
