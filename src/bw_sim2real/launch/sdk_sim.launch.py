"""
纯仿真 Launch 文件
用于 SDK 仿真预览模式，不启动 input_router/panel 等控制节点

使用方法:
    ros2 launch bw_sim2real sdk_sim.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_rviz = LaunchConfiguration("enable_rviz")
    
    # 1. 定义包名和文件名
    description_package = "mantis_description"
    urdf_file = "mantis.urdf"
    
    # 2. 定位文件路径
    pkg_share = FindPackageShare(description_package)
    urdf_model_path = PathJoinSubstitution([pkg_share, "urdf", urdf_file])

    # 3. 读取 URDF 内容
    robot_description_content = ParameterValue(
        Command(['cat ', urdf_model_path]),
        value_type=str
    )

    # 4. Robot State Publisher
    # 直接订阅 /joint_states（由 sdk_bridge_node 发布）
    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }],
    )
    
    # 5. SDK Bridge 节点
    # 接收 SDK 的 JSON 消息，转发到 ROS2 /joint_states
    node_sdk_bridge = Node(
        package="bw_sim2real",
        executable="sdk_bridge_node",
        name="sdk_bridge_node",
        output="screen",
    )

    # 6. RViz2
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "mantis.rviz"])],
        condition=IfCondition(enable_rviz),
    )

    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Start RViz2",
    ))
    
    ld.add_action(node_rsp)
    ld.add_action(node_sdk_bridge)
    ld.add_action(node_rviz)
    
    return ld
