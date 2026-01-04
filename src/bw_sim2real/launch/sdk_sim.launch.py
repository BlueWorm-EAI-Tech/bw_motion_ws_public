"""
SDK 仿真 Launch 文件
===================

用于 SDK 仿真预览模式，支持 RViz 或 Gazebo 两种可视化方式。

参数:
    use_gazebo (bool): 是否使用 Gazebo 物理仿真，默认 false
    use_rviz (bool): 是否使用 RViz 可视化，默认 true（使用 Gazebo 时自动禁用）
    world_name (str): Gazebo 世界文件名，默认 empty.world

使用方法:
    # 仅 RViz（默认，快速预览）
    ros2 launch bw_sim2real sdk_sim.launch.py
    
    # 使用 Gazebo（物理仿真，底盘移动）
    ros2 launch bw_sim2real sdk_sim.launch.py use_gazebo:=true
    
    # 自定义 Gazebo 世界
    ros2 launch bw_sim2real sdk_sim.launch.py use_gazebo:=true world_name:=empty.world

然后使用 SDK:
    from mantis import Mantis
    with Mantis(sim=True) as robot:
        robot.left_arm.set_shoulder_pitch(-0.5)
        robot.chassis.forward(0.2)  # 仅 Gazebo 模式下底盘会移动
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def replace_package_path_in_urdf(urdf_path, package_path):
    """将 URDF 中的 package:// 替换为绝对路径（Gazebo 需要）"""
    with open(urdf_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 替换 package:// 路径
    content = content.replace('package://mantis_description', package_path)
    
    # 移除 XML 声明（spawn_entity 不支持带声明的 Unicode 字符串）
    if content.startswith('<?xml'):
        end_of_declaration = content.find('?>') + 2
        content = content[end_of_declaration:].lstrip()
    
    temp_path = '/tmp/mantis_gazebo_temp.urdf'
    with open(temp_path, 'w', encoding='utf-8') as f:
        f.write(content)
    return temp_path


def launch_setup(context, *args, **kwargs):
    """运行时配置"""
    # 获取参数
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    world_name = LaunchConfiguration('world_name').perform(context)
    
    # 使用 Gazebo 时强制禁用 RViz
    if use_gazebo:
        use_rviz = False
        print("[sdk_sim] Gazebo 模式: RViz 已禁用")
    
    # 包路径
    pkg_description = FindPackageShare('mantis_description').find('mantis_description')
    
    # URDF 路径选择
    if use_gazebo:
        urdf_path = os.path.join(pkg_description, 'urdf', 'mantis_gazebo.urdf')
        if not os.path.exists(urdf_path):
            urdf_path = os.path.join(pkg_description, 'urdf', 'mantis.urdf')
            print(f"[sdk_sim] 警告: mantis_gazebo.urdf 不存在，使用 mantis.urdf")
    else:
        urdf_path = os.path.join(pkg_description, 'urdf', 'mantis.urdf')
    
    # 读取 URDF 内容
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    nodes = []
    
    # ==================== 通用节点 ====================
    
    # 1. Robot State Publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_gazebo  # Gazebo 使用仿真时间
        }]
    ))
    
    # 2. SDK Bridge Node（接收 Zenoh 消息，发布 /sdk/joint_states）
    nodes.append(Node(
        package='bw_sim2real',
        executable='sdk_bridge_node',
        name='sdk_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_gazebo}]
    ))
    
    # ==================== Gazebo 模式 ====================
    
    if use_gazebo:
        # Gazebo 需要绝对路径的 URDF
        temp_urdf_path = replace_package_path_in_urdf(urdf_path, pkg_description)
        
        # 世界文件
        world_path = None
        if world_name:
            search_paths = [
                os.path.join(pkg_description, 'worlds', world_name),
                f'/usr/share/gazebo-11/worlds/{world_name}',
                world_name
            ]
            for path in search_paths:
                if os.path.exists(path):
                    world_path = path
                    break
        
        # 3a. Gazebo 进程
        gazebo_cmd = [
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ]
        if world_path:
            gazebo_cmd.append(world_path)
            print(f"[sdk_sim] 使用世界: {world_path}")
        
        gazebo_env = {
            'GAZEBO_MODEL_PATH': os.environ.get('GAZEBO_MODEL_PATH', ''),
            'GAZEBO_MODEL_DATABASE_URI': ''  # 禁用在线模型查询
        }
        
        nodes.append(ExecuteProcess(
            cmd=gazebo_cmd,
            output='screen',
            additional_env=gazebo_env
        ))
        
        # 3b. Spawn Robot
        nodes.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'mantis',
                '-file', temp_urdf_path,
                '-x', '0', '-y', '0', '-z', '0'
            ],
            output='screen'
        ))
        
        # 3c. Gazebo Bridge Node（转发关节/底盘命令到 Gazebo）
        nodes.append(Node(
            package='bw_sim2real',
            executable='gazebo_bridge_node',
            name='gazebo_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ))
        
        print("[sdk_sim] Gazebo 模式启动")
    
    # ==================== RViz 模式 ====================
    
    if use_rviz:
        rviz_config = os.path.join(pkg_description, 'rviz', 'mantis.rviz')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ))
        print("[sdk_sim] RViz 模式启动")
    
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='是否使用 Gazebo 物理仿真（启用后自动禁用 RViz）'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否使用 RViz 可视化（Gazebo 模式下自动禁用）'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='empty.world',
            description='Gazebo 世界文件名'
        ),
        OpaqueFunction(function=launch_setup)
    ])
