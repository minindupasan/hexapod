#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class HexapodTestController(Node):
    def __init__(self):
        super().__init__('hexapod_test_controller')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Extract exact joint names from your URDF
        self.joint_names = [
            'femur_1_joint', 'tibia_1_joint', 'coxa_1_joint',
            'femur_2_joint', 'tibia_2_joint', 'coxa_2_joint',
            'femur_3_joint', 'tibia_3_joint', 'coxa_3_joint',
            'femur_4_joint', 'tibia_4_joint', 'coxa_4_joint',
            'femur_5_joint', 'tibia_5_joint', 'coxa_5_joint',
            'femur_6_joint', 'tibia_6_joint', 'coxa_6_joint'
        ]
        
        # Test with simple movement
        self.timer = self.create_timer(0.1, self.test_joints)
        self.start_time = time.time()
        
        self.get_logger().info(f'Test Controller started with {len(self.joint_names)} joints')
        for i, name in enumerate(self.joint_names):
            self.get_logger().info(f'Joint {i}: {name}')
        
    def test_joints(self):
        """Test each joint individually"""
        current_time = time.time() - self.start_time
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = []
        
        # Test specific joint movement
        for i, joint_name in enumerate(self.joint_names):
            if 'femur_4_joint' in joint_name:
                # Make femur_4_joint oscillate more obviously
                angle = 0.5 * math.sin(current_time * 2.0)
                msg.position.append(angle)
                if abs(angle) > 0.4:  # Log when moving significantly
                    self.get_logger().info(f'{joint_name} angle: {angle:.3f}')
            elif 'femur' in joint_name:
                # Small movement for other femur joints
                msg.position.append(0.1 * math.sin(current_time + i))
            elif 'tibia' in joint_name:
                # Different movement for tibia joints
                msg.position.append(-0.3 + 0.2 * math.sin(current_time + i))
            elif 'coxa' in joint_name:
                # Small rotation for coxa joints
                msg.position.append(0.05 * math.sin(current_time * 0.5 + i))
            else:
                msg.position.append(0.0)
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = HexapodTestController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
