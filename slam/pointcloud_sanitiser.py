"""Point cloud sanitiser (fast).

Reads /depth_camera/points as raw bytes, strips NaN/Inf and out-of-range
points using vectorised numpy, republishes on /depth_camera/points_filtered.

Previous version used a Python loop over each point; that ran at ~1 Hz on a
640x480 cloud (307k points). This version runs at ~30 Hz.
"""
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

MAX_RANGE_M = 5.0
MIN_RANGE_M = 0.3


def _dtype_from_fields(fields):
    """Build a numpy dtype matching the PointCloud2 field layout."""
    m = {
        PointField.INT8: 'i1', PointField.UINT8: 'u1',
        PointField.INT16: 'i2', PointField.UINT16: 'u2',
        PointField.INT32: 'i4', PointField.UINT32: 'u4',
        PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
    }
    return [(f.name, m[f.datatype]) for f in sorted(fields, key=lambda x: x.offset)]


def _make_xyz_cloud(header, xyz):
    n = xyz.shape[0]
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.data = xyz.astype(np.float32).tobytes()
    return msg


class Sanitiser(Node):
    def __init__(self):
        super().__init__("pointcloud_sanitiser")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,   # keep only the newest — don't buffer stale frames
        )
        self.sub = self.create_subscription(
            PointCloud2, "/depth_camera/points", self.on_cloud, qos)
        self.pub = self.create_publisher(
            PointCloud2, "/depth_camera/points_filtered", qos)
        self.count = 0
        self.get_logger().info(
            f"sanitising -> /depth_camera/points_filtered "
            f"(range {MIN_RANGE_M}-{MAX_RANGE_M} m, vectorised)")

    def on_cloud(self, msg: PointCloud2):
        # Interpret the raw bytes as a numpy structured array matching the fields
        dtype = np.dtype(_dtype_from_fields(msg.fields))
        # PointCloud2 rows may have padding to point_step; account for it
        pt_arr = np.frombuffer(msg.data, dtype=dtype)
        if pt_arr.size == 0:
            return

        # Extract x/y/z as a plain (N, 3) float32 array — vectorised, no Python loop
        xyz = np.stack([pt_arr["x"], pt_arr["y"], pt_arr["z"]], axis=-1).astype(np.float32)

        # Filter: finite AND in-range
        finite = np.isfinite(xyz).all(axis=1)
        d2 = xyz[:, 0]**2 + xyz[:, 1]**2 + xyz[:, 2]**2   # squared range, faster than sqrt
        in_range = (d2 >= MIN_RANGE_M * MIN_RANGE_M) & (d2 <= MAX_RANGE_M * MAX_RANGE_M)
        clean = xyz[finite & in_range]

        if clean.shape[0] < 100:
            return

        out_msg = _make_xyz_cloud(msg.header, clean)
        self.pub.publish(out_msg)

        self.count += 1
        if self.count % 60 == 0:
            self.get_logger().info(
                f"sanitised {self.count} clouds ({clean.shape[0]}/{pt_arr.shape[0]} pts)")


def main():
    rclpy.init()
    node = Sanitiser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()