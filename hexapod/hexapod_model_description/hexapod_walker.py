#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class HexapodWalker(Node):
    def __init__(self):
        super().__init__('hexapod_walker')
        
        # Create publishers for each joint
        self.joint_publishers = {}
        
        # Joint names for all 6 legs
        legs = ['1', '2', '3', '4', '5', '6']
        joints = ['coxa', 'femur', 'tibia']
        
        for leg in legs:
            for joint in joints:
                topic_name = f'/{joint}_{leg}_position_controller/command'
                self.joint_publishers[f'{joint}_{leg}'] = self.create_publisher(
                    Float64, topic_name, 10)
        
        # Walking parameters
        self.step_height = 0.5  # How high to lift legs
        self.step_frequency = 1.0  # Steps per second
        self.stride_length = 0.3  # Forward reach
        
        # Timer for walking gait
        self.timer = self.create_timer(0.05, self.walking_gait)  # 20Hz
        self.start_time = time.time()
        
        self.get_logger().info('Hexapod Walker Node Started - Spider Walking Mode!')
        
    def walking_gait(self):
        """Tripod gait - typical spider walking pattern"""
        current_time = time.time() - self.start_time
        phase = (current_time * self.step_frequency) % 1.0
        
        # Tripod 1: Legs 1, 3, 5 (right front, left middle, right rear)
        # Tripod 2: Legs 2, 4, 6 (left front, right middle, left rear)
        
        # Define leg positions for tripod gait
        tripod1_legs = ['1', '3', '5']
        tripod2_legs = ['2', '4', '6']
        
        for leg in tripod1_legs:
            self.move_leg(leg, phase)
            
        for leg in tripod2_legs:
            # Phase shift of 0.5 for opposite tripod
            opposite_phase = (phase + 0.5) % 1.0
            self.move_leg(leg, opposite_phase)
    
    def move_leg(self, leg_num, phase):
        """Move a single leg through its walking cycle"""
        
        # Basic leg positions (neutral stance)
        coxa_neutral = 0.0
        femur_neutral = -0.5
        tibia_neutral = 1.0
        
        if phase < 0.5:
            # Stance phase - leg on ground, body moves forward
            progress = phase * 2.0  # 0 to 1
            
            # Coxa sweeps backward (body moves forward relative to leg)
            coxa_angle = coxa_neutral + self.stride_length * (0.5 - progress)
            
            # Femur and tibia maintain ground contact
            femur_angle = femur_neutral
            tibia_angle = tibia_neutral
            
        else:
            # Swing phase - leg lifts and moves forward
            progress = (phase - 0.5) * 2.0  # 0 to 1
            
            # Coxa moves forward for next step
            coxa_angle = coxa_neutral + self.stride_length * (progress - 0.5)
            
            # Lift leg (arc motion)
            lift_height = self.step_height * math.sin(progress * math.pi)
            femur_angle = femur_neutral - lift_height
            tibia_angle = tibia_neutral + lift_height * 0.5
        
        # Apply leg-specific adjustments based on position
        coxa_angle = self.adjust_for_leg_position(leg_num, coxa_angle)
        
        # Publish joint commands
        self.publish_joint_command(f'coxa_{leg_num}', coxa_angle)
        self.publish_joint_command(f'femur_{leg_num}', femur_angle)
        self.publish_joint_command(f'tibia_{leg_num}', tibia_angle)
    
    def adjust_for_leg_position(self, leg_num, base_angle):
        """Adjust coxa angle based on leg position on body"""
        # Leg positions (approximate angles from body center)
        leg_positions = {
            '1': 45,    # Right front
            '2': 135,   # Left front  
            '3': 0,     # Right middle
            '4': 180,   # Left middle
            '5': -45,   # Right rear
            '6': -135   # Left rear
        }
        
        body_angle = leg_positions.get(leg_num, 0)
        return base_angle + math.radians(body_angle * 0.1)  # Small adjustment
    
    def publish_joint_command(self, joint_name, angle):
        """Publish command to a joint"""
        if joint_name in self.joint_publishers:
            msg = Float64()
            msg.data = float(angle)
            self.joint_publishers[joint_name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    hexapod_walker = HexapodWalker()
    
    try:
        rclpy.spin(hexapod_walker)
    except KeyboardInterrupt:
        pass
    
    hexapod_walker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
