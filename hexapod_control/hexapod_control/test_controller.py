#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class TestController(Node):
    def __init__(self):
        super().__init__('test_controller')
        
        # Publisher to the joint position controller
        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            '/hexapod_position_controller/commands',
            10
        )
        
        # Timer for sending commands
        self.timer = self.create_timer(0.1, self.publish_commands)  # 10 Hz
        
        # Joint names for reference (matching controller config)
        self.joint_names = [
            'coxa_1_joint', 'femur_1_joint', 'tibia_1_joint',  # Leg 1
            'coxa_2_joint', 'femur_2_joint', 'tibia_2_joint',  # Leg 2
            'coxa_3_joint', 'femur_3_joint', 'tibia_3_joint',  # Leg 3
            'coxa_4_joint', 'femur_4_joint', 'tibia_4_joint',  # Leg 4
            'coxa_5_joint', 'femur_5_joint', 'tibia_5_joint',  # Leg 5
            'coxa_6_joint', 'femur_6_joint', 'tibia_6_joint'   # Leg 6
        ]
        
        self.start_time = time.time()
        self.get_logger().info('Test Controller started!')
    
    def publish_commands(self):
        msg = Float64MultiArray()
        current_time = time.time() - self.start_time
        
        # Create simple sine wave movements for testing
        positions = []
        for i in range(18):  # 18 joints total
            # Different frequency for each joint type
            if i % 3 == 0:  # Coxa joints
                amplitude = 0.3
                frequency = 0.5
            elif i % 3 == 1:  # Femur joints
                amplitude = 0.5
                frequency = 0.3
            else:  # Tibia joints
                amplitude = 0.7
                frequency = 0.4
            
            position = amplitude * math.sin(frequency * current_time + i * 0.5)
            positions.append(position)
        
        msg.data = positions
        self.position_publisher.publish(msg)
        
        # Log periodically
        if int(current_time) % 5 == 0 and int(current_time * 10) % 50 == 0:
            self.get_logger().info(f'Publishing test positions at t={current_time:.1f}s')

def main(args=None):
    rclpy.init(args=args)
    test_controller = TestController()
    
    try:
        rclpy.spin(test_controller)
    except KeyboardInterrupt:
        pass
    finally:
        test_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()