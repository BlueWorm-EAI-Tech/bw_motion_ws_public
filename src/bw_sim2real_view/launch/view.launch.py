from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_router = LaunchConfiguration('enable_router')
    enable_panel = LaunchConfiguration('enable_panel')
    enable_playback = LaunchConfiguration('enable_playback')

    default_mode = LaunchConfiguration('default_mode')
    record_source = LaunchConfiguration('record_source')

    # 仅负责“话题控制/模式控制/录制回放”等视图相关节点。
    # 默认不启动 rviz / bridge / IK（它们在 phase1 launch 里做可选组合）。

    node_router = Node(
        package="bw_sim2real",
        executable="input_router",
        name="input_router",
        output="screen",
        parameters=[{'default_mode': default_mode}],
        condition=IfCondition(enable_router),
    )

    # 控制面板（统一UI入口）
    node_panel = Node(
        package="bw_sim2real_view",
        executable="mantis_control_panel",
        name="mantis_control_panel",
        output="screen",
        condition=IfCondition(enable_panel),
    )

    # 关键帧录制/回放（此节点已不再负责启动UI，只负责数据与播放线程）
    node_playback = Node(
        package="bw_sim2real",
        executable="mantis_playback_node",
        name="mantis_playback",
        output="screen",
        parameters=[{'record_source': record_source}],
        condition=IfCondition(enable_playback),
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_router', default_value='true', description='Start input_router'),
        DeclareLaunchArgument('enable_panel', default_value='true', description='Start Qt control panel'),
        DeclareLaunchArgument('enable_playback', default_value='true', description='Start keyframe playback node'),
        DeclareLaunchArgument('default_mode', default_value='gui', description='input_router default_mode'),
        DeclareLaunchArgument('record_source', default_value='control', description='mantis_playback_node record_source'),
        node_router,
        node_playback,
        node_panel,
    ])
