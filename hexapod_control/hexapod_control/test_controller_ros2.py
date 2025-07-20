#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class HexapodTestController(Node):
    def __init__(self):
        super().__init__('hexapod_test_controller')
        
        # Publisher for hexapod position controller
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/hexapod_position_controller/commands', 
            10
        )
        
        # Timer for test movements
        self.timer = self.create_timer(0.1, self.test_movement)  # 10 Hz
        self.start_time = time.time()
        
        self.get_logger().info('Hexapod Test Controller started')
    
    def test_movement(self):
        """Simple test movement - sine wave on all joints"""
        current_time = time.time() - self.start_time
        
        # Create command message
        msg = Float64MultiArray()
        
        # Generate sine wave positions for all 18 joints
        positions = []
        for i in range(18):
            # Different frequency and phase for each joint
            freq = 0.5 + (i % 3) * 0.2  # Different freq for coxa/femur/tibia
            phase = (i // 3) * math.pi / 3  # Different phase for each leg
            amplitude = 0.3 + (i % 3) * 0.2  # Different amplitude
            
            position = amplitude * math.sin(freq * current_time + phase)
            positions.append(position)
        
        msg.data = positions
        self.position_pub.publish(msg)
        
        if int(current_time) % 5 == 0 and int(current_time * 10) % 50 == 0:
            self.get_logger().info(f'Test running... time: {current_time:.1f}s')


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
