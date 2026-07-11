"""RTAB-Map launcher using explicit remappings.

The rtabmap_launch.launch.py in ROS 2 Humble ignores certain topic
argument names (scan_cloud_topic, etc.) — the launch API uses launch
description composition rather than argument substitution. Building
the nodes directly lets us set remaps that actually apply.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Delete any prior map so we start fresh
    os.system("rm -f ~/.ros/rtabmap.db")

    common_params = [{
        "frame_id": "base_link",
        "use_sim_time": True,
        "subscribe_scan_cloud": True,
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "approx_sync": True,
        "approx_sync_max_interval": 0.1,
        "qos": 2,
        "qos_scan_cloud": 2,
        "qos_odom": 2,
        "Reg/Strategy": "1",             # ICP registration for point clouds
        "Grid/RangeMax": "5.0",
        "Icp/PointToPlaneRadius": "0",
    }]

    remaps = [
        ("scan_cloud", "/depth_camera/points"),
        ("odom", "/odom"),
    ]

    return LaunchDescription([
        Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            output="screen",
            parameters=common_params,
            remappings=remaps + [("odom", "/rtabmap/odom")],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=common_params,
            arguments=["-d"],
            remappings=remaps,
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            output="screen",
            parameters=common_params,
            remappings=remaps,
        ),
    ])
