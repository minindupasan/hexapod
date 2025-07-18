#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class SimpleWalkingController(Node):
    def __init__(self):
        super().__init__('simple_walking_controller')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Joint names matching your URDF exactly (order from actual joint_states output)
        self.joint_names = [
            'femur_1_joint', 'tibia_1_joint', 'coxa_1_joint',
            'femur_2_joint', 'tibia_2_joint', 'coxa_2_joint', 
            'femur_3_joint', 'tibia_3_joint', 'coxa_3_joint',
            'femur_4_joint', 'tibia_4_joint', 'coxa_4_joint',
            'femur_5_joint', 'tibia_5_joint', 'coxa_5_joint',
            'femur_6_joint', 'tibia_6_joint', 'coxa_6_joint'
        ]
        
        # Resting positions from your actual joint_states output (default spider pose)
        self.rest_femur_angle = 0.5   # Femur joint resting position
        self.rest_tibia_angle = -1.5  # Tibia joint resting position  
        self.rest_coxa_angle = 0.0    # Coxa joint resting position
        
        # Walking parameters
        self.walking = True
        self.step_cycle = 0.0
        self.step_speed = 0.5  # cycles per second
        
        # Walking amplitudes (how much joints move from resting position)
        self.step_amplitude_femur = 0.4   # How much femur moves during walking
        self.step_amplitude_tibia = 0.3   # How much tibia moves during walking
        self.step_amplitude_coxa = 0.1    # How much coxa rotates during walking
        
        # Leg base angles for radial arrangement
        self.leg_base_angles = [0, 45, 135, 180, 225, 315]  # degrees
        
        # Timer
        self.timer = self.create_timer(0.1, self.walk_cycle)
        self.start_time = time.time()
        
        self.get_logger().info('Simple Walking Controller started')
        
    def walk_cycle(self):
        """Simple walking cycle with tripod gait using correct resting positions"""
        current_time = time.time() - self.start_time
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = []
        
        # Tripod gait pattern
        for leg in range(6):
            # Tripod grouping: Group A (legs 0,2,4) vs Group B (legs 1,3,5)  
            group_a = leg in [0, 2, 4]
            
            # Phase offset for tripod gait
            phase_offset = 0 if group_a else math.pi
            
            # Calculate walking phases
            walk_phase = current_time * self.step_speed * 2 * math.pi + phase_offset
            
            # Femur (first joint) - lifting movement based on resting position (0.5)
            if math.sin(walk_phase) > 0:  # Swing phase - lift leg
                femur_angle = self.rest_femur_angle + self.step_amplitude_femur * math.sin(walk_phase)
            else:  # Stance phase - leg supports body
                femur_angle = self.rest_femur_angle - 0.1 * math.sin(walk_phase)
                
            # Tibia (second joint) - extending/retracting movement based on resting position (-1.5)
            if math.sin(walk_phase) > 0:  # Swing phase - extend leg forward
                tibia_angle = self.rest_tibia_angle + self.step_amplitude_tibia * math.sin(walk_phase)
            else:  # Stance phase - retract leg backward
                tibia_angle = self.rest_tibia_angle - 0.2 * math.sin(walk_phase)
            
            # Coxa (third joint) - small rotational movement based on resting position (0.0)
            coxa_angle = self.rest_coxa_angle + self.step_amplitude_coxa * math.sin(walk_phase)
            
            # Joint order: femur, tibia, coxa (as per your joint_states output)
            msg.position.extend([femur_angle, tibia_angle, coxa_angle])
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller = SimpleWalkingController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
