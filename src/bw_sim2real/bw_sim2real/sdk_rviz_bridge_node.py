#!/usr/bin/env python3
"""
SDK 仿真桥接节点

接收 Zenoh 发来的关节数据，转发到 ROS2 /joint_states 话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import threading

try:
    import zenoh
except ImportError:
    print("请安装 zenoh: pip install eclipse-zenoh")
    raise


class SDKBridgeNode(Node):
    """SDK 桥接节点"""
    
    # SDK 发布的 Zenoh 话题（JSON 格式）
    ZENOH_TOPIC = "sdk/joint_states"
    
    def __init__(self):
        super().__init__('sdk_bridge_node')
        
        # ROS2 发布者
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Zenoh 会话
        self.zenoh_session = None
        self.zenoh_subscriber = None
        
        # 启动 Zenoh 订阅
        self._start_zenoh()
        
        self.get_logger().info("SDK Bridge 节点已启动，等待 SDK 连接...")
    
    def _start_zenoh(self):
        """启动 Zenoh 订阅"""
        try:
            config = zenoh.Config()
            self.zenoh_session = zenoh.open(config)
            
            self.zenoh_subscriber = self.zenoh_session.declare_subscriber(
                self.ZENOH_TOPIC,
                self._on_zenoh_message
            )
            self.get_logger().info(f"已订阅 Zenoh 话题: {self.ZENOH_TOPIC}")
        except Exception as e:
            self.get_logger().error(f"Zenoh 初始化失败: {e}")
    
    def _on_zenoh_message(self, sample):
        """处理 Zenoh 消息"""
        try:
            # 解析 JSON
            data = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            # 构建 JointState 消息
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = data.get('name', [])
            msg.position = data.get('position', [])
            msg.velocity = data.get('velocity', [])
            msg.effort = data.get('effort', [])
            
            # 发布到 ROS2
            self.joint_pub.publish(msg)
            
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON 解析失败: {e}")
        except Exception as e:
            self.get_logger().error(f"处理消息失败: {e}")
    
    def destroy_node(self):
        """清理资源"""
        if self.zenoh_subscriber:
            self.zenoh_subscriber.undeclare()
        if self.zenoh_session:
            self.zenoh_session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SDKBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
