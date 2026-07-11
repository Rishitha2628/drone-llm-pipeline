"""Odometry -> TF bridge for RTAB-Map. Simpler & louder-debug version.

Reads PX4 local NED position via MAVSDK, publishes:
  1. odom -> base_link TF at ~20 Hz
  2. base_link -> camera_link static TF
  3. nav_msgs/Odometry on /odom
"""
import asyncio
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavsdk import System


def euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.n = self.e = self.d = 0.0
        self.yaw_deg = 0.0
        self.have_pos = False
        self.have_att = False
        self.pos_count = 0
        self.att_count = 0


S = State()


async def mavsdk_main():
    drone = System()
    print("[odom_bridge] connecting MAVSDK to udpin://0.0.0.0:14540")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for st in drone.core.connection_state():
        if st.is_connected:
            print("[odom_bridge] MAVSDK connected")
            break

    async def pos_loop():
        print("[odom_bridge] starting position_velocity_ned stream")
        async for pv in drone.telemetry.position_velocity_ned():
            with S.lock:
                S.n = pv.position.north_m
                S.e = pv.position.east_m
                S.d = pv.position.down_m
                S.have_pos = True
                S.pos_count += 1

    async def att_loop():
        print("[odom_bridge] starting attitude_euler stream")
        async for a in drone.telemetry.attitude_euler():
            with S.lock:
                S.yaw_deg = a.yaw_deg
                S.have_att = True
                S.att_count += 1

    await asyncio.gather(pos_loop(), att_loop())


class OdomBridge(Node):
    def __init__(self):
        super().__init__("px4_odom_bridge")
        self.tfb = TransformBroadcaster(self)
        self.static_tfb = StaticTransformBroadcaster(self)
        # from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.15
        t.transform.rotation = euler_to_quat(0.0, 0.35, 0.0)
        self.static_tfb.sendTransform(t)

        self.create_timer(0.05, self.publish_state)   # 20 Hz
        self.create_timer(2.0,  self.print_status)    # heartbeat
        self.get_logger().info("odom bridge ready. static TF sent.")

    def print_status(self):
        with S.lock:
            print(f"[odom_bridge] pos_count={S.pos_count} att_count={S.att_count} "
                  f"have_pos={S.have_pos} have_att={S.have_att}")

    def publish_state(self):
        with S.lock:
            if not S.have_pos:
                return
            n, e, d, yaw_deg = S.n, S.e, S.d, S.yaw_deg

        # PX4 NED -> ROS ENU
        x_enu = e
        y_enu = n
        z_enu = -d
        yaw_rad = math.radians(yaw_deg)

        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x_enu
        t.transform.translation.y = y_enu
        t.transform.translation.z = z_enu
        t.transform.rotation = euler_to_quat(0.0, 0.0, yaw_rad)
        self.tfb.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x_enu
        odom.pose.pose.position.y = y_enu
        odom.pose.pose.position.z = z_enu
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = OdomBridge()

    def runner():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(mavsdk_main())
        except Exception as e:
            print(f"[odom_bridge] MAVSDK thread crashed: {e!r}")

    t = threading.Thread(target=runner, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
