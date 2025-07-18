#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time
import numpy as np

class HexapodJointController(Node):
    def __init__(self):
        super().__init__('hexapod_joint_controller')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Joint names matching your URDF exactly
        self.joint_names = [
            'coxa_1_joint', 'tibia_1_joint', 'femur_1_joint',
            'coxa_2_joint', 'tibia_2_joint', 'femur_2_joint', 
            'coxa_3_joint', 'tibia_3_joint', 'femur_3_joint',
            'coxa_4_joint', 'tibia_4_joint', 'femur_4_joint',
            'coxa_5_joint', 'tibia_5_joint', 'femur_5_joint',
            'coxa_6_joint', 'tibia_6_joint', 'femur_6_joint'
        ]
        
        # Leg positions based on your URDF coxa joint origins
        self.leg_positions = [
            (-0.0265, 0.0694),    # Leg 1
            (-0.107782, 0.103068), # Leg 2  
            (-0.107782, 0.265632), # Leg 3
            (-0.0265, 0.2993),     # Leg 4
            (0.054782, 0.265632),  # Leg 5
            (0.054782, 0.103068)   # Leg 6
        ]
        
        # Movement parameters
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.cycle_time = 2.0  # seconds per step cycle
        self.step_height = 0.03  # meters
        self.step_length = 0.08   # meters
        
        # Leg geometry based on your URDF measurements
        self.coxa_length = 0.05929   # From origin offsets in tibia joints
        self.femur_length = 0.0954   # From femur joint origins
        self.tibia_length = 0.1267   # Estimated from mesh scale
        
        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        self.time_start = time.time()
        
        self.get_logger().info('Hexapod Joint Controller started')
        
    def cmd_callback(self, msg):
        """Handle velocity commands"""
        self.linear_x = msg.linear.x * 0.1  # Scale down for safety
        self.angular_z = msg.angular.z * 0.1
        
    def control_loop(self):
        """Main control loop"""
        t = time.time() - self.time_start
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.calculate_joint_positions(t)
        
        self.joint_pub.publish(msg)
        
    def calculate_joint_positions(self, t):
        """Calculate joint positions for tripod gait matching your URDF structure"""
        positions = []
        
        # Adjust cycle time based on speed
        actual_cycle_time = self.cycle_time / (abs(self.linear_x) + 0.1)
        
        for leg in range(6):
            # Tripod gait grouping
            group_a = leg in [0, 2, 4]  # legs 1, 3, 5
            
            # Phase calculation
            phase = (t / actual_cycle_time) % 1.0
            if not group_a:
                phase = (phase + 0.5) % 1.0
            
            # Get leg base position
            base_x, base_y = self.leg_positions[leg]
            
            # Calculate leg end effector position
            x, y, z = self.calculate_leg_position(leg, phase, base_x, base_y)
            
            # Apply body movement
            if phase < 0.5:  # Stance phase
                x += self.linear_x * (0.5 - phase) * 2
            
            # Inverse kinematics
            coxa_angle, tibia_angle, femur_angle = self.leg_ik(x, y, z, leg)
            
            # Joint order in your URDF: coxa, tibia, femur per leg
            positions.extend([coxa_angle, tibia_angle, femur_angle])
        
        return positions
        
    def calculate_leg_position(self, leg_num, phase, base_x, base_y):
        """Calculate leg tip position for given phase relative to leg base"""
        # Default leg extension from coxa joint
        default_reach = 0.08  # Distance from coxa joint
        default_z = -0.1      # Below coxa joint
        
        # Leg angle relative to body (based on leg placement)
        leg_angles = [0, 45, 135, 180, 225, 315]  # degrees for legs 1-6
        leg_angle_rad = math.radians(leg_angles[leg_num])
        
        if phase < 0.5:  # Stance phase - leg on ground
            # Leg moves backward relative to body
            step_progress = phase * 2
            x_offset = default_reach * math.cos(leg_angle_rad) + self.step_length * (0.5 - step_progress)
            y_offset = default_reach * math.sin(leg_angle_rad)
            z = default_z
        else:  # Swing phase - leg in air
            swing_phase = (phase - 0.5) * 2
            x_offset = default_reach * math.cos(leg_angle_rad) + self.step_length * (swing_phase - 0.5)
            y_offset = default_reach * math.sin(leg_angle_rad)
            z = default_z + self.step_height * math.sin(swing_phase * math.pi)
        
        # Convert to global coordinates relative to base_link
        x = base_x + x_offset
        y = base_y + y_offset
        
        return x, y, z
        
    def leg_ik(self, target_x, target_y, target_z, leg_num):
        """Inverse kinematics for 3-DOF leg matching your URDF joint hierarchy"""
        # Get leg base position
        base_x, base_y = self.leg_positions[leg_num]
        
        # Calculate relative position from coxa joint
        rel_x = target_x - base_x
        rel_y = target_y - base_y
        rel_z = target_z - 0.013762  # Height offset from coxa joint
        
        # Coxa angle (horizontal rotation around Z)
        coxa_angle = math.atan2(rel_y, rel_x)
        
        # Distance from coxa joint to target in XY plane
        xy_distance = math.sqrt(rel_x*rel_x + rel_y*rel_y)
        
        # Remaining distance after coxa offset
        remaining_distance = xy_distance - self.coxa_length
        
        # 3D distance from femur joint to target
        target_distance_3d = math.sqrt(remaining_distance*remaining_distance + rel_z*rel_z)
        
        # Tibia and femur angles using 2-DOF IK
        if target_distance_3d > (self.femur_length + self.tibia_length):
            # Target too far, extend fully
            femur_angle = math.atan2(-rel_z, remaining_distance)  # Negative because Z points down
            tibia_angle = 0
        else:
            # Use cosine law for 2-link IK
            # Tibia angle (knee joint)
            cos_tibia = (self.femur_length*self.femur_length + 
                        self.tibia_length*self.tibia_length - 
                        target_distance_3d*target_distance_3d) / (2 * self.femur_length * self.tibia_length)
            cos_tibia = max(-1, min(1, cos_tibia))  # Clamp to valid range
            
            tibia_angle = math.acos(cos_tibia) - math.pi  # Bend backward
            
            # Femur angle (shoulder joint)
            alpha = math.atan2(-rel_z, remaining_distance)  # Angle to target
            cos_alpha = (self.femur_length*self.femur_length + 
                        target_distance_3d*target_distance_3d - 
                        self.tibia_length*self.tibia_length) / (2 * self.femur_length * target_distance_3d)
            cos_alpha = max(-1, min(1, cos_alpha))
            beta = math.acos(cos_alpha)
            
            femur_angle = alpha - beta  # Adjust for proper femur orientation
        
        return coxa_angle, tibia_angle, femur_angle

def main(args=None):
    rclpy.init(args=args)
    
    controller = HexapodJointController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()