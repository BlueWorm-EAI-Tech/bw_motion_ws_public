import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    enable_view = LaunchConfiguration("enable_view")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_bridge = LaunchConfiguration("enable_bridge")
    enable_ik = LaunchConfiguration("enable_ik")

    # 1. 定义包名和文件名
    description_package = "mantis_description"
    urdf_file = "mantis.urdf"  # 请确保这个文件就在 urdf 文件夹下
    
    # 2. 定位文件路径
    pkg_share = FindPackageShare(description_package)
    urdf_model_path = PathJoinSubstitution([pkg_share, "urdf", urdf_file])

    # 3. Robot State Publisher
    # 对于静态 URDF，直接通过 arguments 传入路径即可，不需要读取内容传 parameters
    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=[urdf_model_path],  # <--- 直接传路径
        remappings=[("/joint_states", "/ctrl/joint_target")] 
    )

    # 4. View launch：只负责控制相关节点（router/playback/panel），不启动 RViz
    view_pkg = FindPackageShare("bw_sim2real_view")
    view_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([view_pkg, "launch", "view.launch.py"])
        ),
        condition=IfCondition(enable_view),
    )

    # 5. RViz2
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "mantis.rviz"])]
        condition=IfCondition(enable_rviz),
    )

    # 6. 功能节点：Bridge（phase1 启动时包含实机桥接）
    node_bridge = Node(
        package="bw_sim2real",
        executable="sim_to_real_bridge",
        name="sim_to_real_bridge",
        output="screen",
        condition=IfCondition(enable_bridge),
    )

    # 7. 功能节点：IK（注意：该节点较重，后续可改为参数化可选启动）
    node_casadi_ik = Node(
        package="bw_sim2real",
        executable="mantis_casadi_node", # 记得在 setup.py/CMakeLists 注册
        name="mantis_casadi_node",
        output="screen",
        condition=IfCondition(enable_ik),
    )


    ld = LaunchDescription()

    # Startup load switches
    ld.add_action(DeclareLaunchArgument(
        "enable_view",
        default_value="true",
        description="Start view/control layer (router + panel + playback) via bw_sim2real_view/view.launch.py",
    ))
    ld.add_action(DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Start RViz2",
    ))
    ld.add_action(DeclareLaunchArgument(
        "enable_bridge",
        default_value="false",
        description="Start sim_to_real_bridge (heavy / hardware-related)",
    ))
    ld.add_action(DeclareLaunchArgument(
        "enable_ik",
        default_value="false",
        description="Start mantis_casadi_node IK (heavy)",
    ))

    ld.add_action(node_rsp)
    ld.add_action(view_launch)
    ld.add_action(node_rviz)
    ld.add_action(node_bridge)
    ld.add_action(node_casadi_ik)

    return ld