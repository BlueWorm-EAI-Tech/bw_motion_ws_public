from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 构建配置 (和原来一样)
    moveit_config = MoveItConfigsBuilder("mantis", package_name="mantis_moveit_config").to_moveit_configs()

    # 2. 定义 MoveGroup 节点 (核心后端)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": False},
        ],
    )

    # 3. 定义 Robot State Publisher (发布 TF 树)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 4. 定义 ROS 2 Control 节点 (虚拟控制器管理器)
    # 如果你是连接真机，这一步可能不需要，取决于你的架构
    # 但如果是纯仿真 demo，必须要有这个来加载 Fake Hardware
    ros2_controllers_path = moveit_config.package_path / "config" / "ros2_controllers.yaml"
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    # 5. 定义控制器 Spawner (自动加载控制器)
    # 这里我们读取 moveit_config 里的控制器列表
    # 通常 demo.launch.py 会自动加载 joint_state_broadcaster 和 具体的 arm controller
    
    # 5.1 加载 joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 5.2 加载我们在 moveit_controllers.yaml 里定义的控制器
    # 这里我们手动写出你的控制器名字，确保稳定
    # 根据你之前的配置，你有4个控制器
    controllers_to_spawn = [
        "arm_l_controller", 
        "arm_r_controller", 
        "gripper_l_controller", 
        "gripper_r_controller"
    ]
    
    controller_spawners = []
    for controller in controllers_to_spawn:
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
        )
        controller_spawners.append(spawner)

    # 6. 定义静态 TF 发布 (World -> Base Link)
    # 否则机器人在 TF 树里是悬空的
    virtual_joint_name = "virtual_joint" # 请确认你的 SRDF 里虚拟关节的名字
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # 7. 组装启动描述
    nodes_to_start = [
        robot_state_publisher_node,
        static_tf_node,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
    ] + controller_spawners

    return LaunchDescription(nodes_to_start)