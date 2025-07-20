#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time


class HexapodROS2ControlWalkingController(Node):
    def __init__(self):
        super().__init__('hexapod_ros2_control_walking_controller')
        
        # Publisher for hexapod position controller
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/hexapod_position_controller/commands', 
            10
        )
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscriber for joint states feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint names in the order expected by the controller
        self.joint_names = [
            'coxa_1_joint', 'femur_1_joint', 'tibia_1_joint',
            'coxa_2_joint', 'femur_2_joint', 'tibia_2_joint',
            'coxa_3_joint', 'femur_3_joint', 'tibia_3_joint',
            'coxa_4_joint', 'femur_4_joint', 'tibia_4_joint',
            'coxa_5_joint', 'femur_5_joint', 'tibia_5_joint',
            'coxa_6_joint', 'femur_6_joint', 'tibia_6_joint'
        ]
        
        # Initialize joint positions (18 joints)
        self.current_positions = [0.0] * 18
        self.target_positions = [0.0] * 18
        
        # Walking parameters
        self.step_height = 0.05
        self.step_length = 0.1
        self.body_height = 0.15
        self.walking_speed = 1.0
        
        # Tripod gait timing
        self.gait_phase = 0.0
        self.gait_period = 2.0  # seconds per complete gait cycle
        
        # Control variables
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.is_walking = False
        
        # Timer for walking control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info('Hexapod ROS2 Control Walking Controller initialized')
        
        # Initialize to standing position
        self.set_standing_position()
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands from user input"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        
        # Start walking if there's any velocity command
        self.is_walking = abs(self.linear_velocity) > 0.01 or abs(self.angular_velocity) > 0.01
        
        if not self.is_walking:
            self.set_standing_position()
            
        self.get_logger().info(f'Velocity command: linear={self.linear_velocity:.2f}, angular={self.angular_velocity:.2f}')
    
    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
    
    def set_standing_position(self):
        """Set robot to neutral standing position"""
        # Basic standing pose - all legs extended downward
        for leg in range(6):
            base_idx = leg * 3
            # Coxa (hip) - slight outward angle
            self.target_positions[base_idx] = 0.0
            # Femur (thigh) - angle to position leg
            self.target_positions[base_idx + 1] = -0.5
            # Tibia (shin) - angle to reach ground
            self.target_positions[base_idx + 2] = 1.0
        
        self.publish_joint_commands()
    
    def control_loop(self):
        """Main control loop for walking gait"""
        if not self.is_walking:
            return
        
        # Update gait phase
        dt = 0.05  # Timer period
        self.gait_phase += dt / self.gait_period
        if self.gait_phase >= 1.0:
            self.gait_phase -= 1.0
        
        # Generate tripod gait
        self.generate_tripod_gait()
        
        # Publish commands
        self.publish_joint_commands()
    
    def generate_tripod_gait(self):
        """Generate tripod gait pattern"""
        # Tripod 1: legs 1, 4, 5 (front-right, back-left, back-right)
        # Tripod 2: legs 2, 3, 6 (front-left, middle-right, middle-left)
        
        tripod1_legs = [0, 3, 4]  # Legs 1, 4, 5 (0-indexed)
        tripod2_legs = [1, 2, 5]  # Legs 2, 3, 6 (0-indexed)
        
        for leg in range(6):
            base_idx = leg * 3
            
            # Determine which tripod this leg belongs to
            if leg in tripod1_legs:
                phase = self.gait_phase
            else:
                phase = (self.gait_phase + 0.5) % 1.0
            
            # Generate leg positions based on phase
            coxa_pos, femur_pos, tibia_pos = self.calculate_leg_position(leg, phase)
            
            self.target_positions[base_idx] = coxa_pos
            self.target_positions[base_idx + 1] = femur_pos
            self.target_positions[base_idx + 2] = tibia_pos
    
    def calculate_leg_position(self, leg, phase):
        """Calculate individual leg joint positions"""
        # Leg angles for hexapod (6 legs in circle)
        leg_angles = [0, math.pi/3, 2*math.pi/3, math.pi, 4*math.pi/3, 5*math.pi/3]
        leg_angle = leg_angles[leg]
        
        # Step cycle: 0-0.5 = stance phase, 0.5-1.0 = swing phase
        if phase < 0.5:
            # Stance phase - leg on ground, body moves over leg
            stance_progress = phase * 2.0
            x_offset = self.step_length * (0.5 - stance_progress) * self.linear_velocity
            y_offset = 0.0
            z_offset = 0.0
        else:
            # Swing phase - leg lifts and moves forward
            swing_progress = (phase - 0.5) * 2.0
            x_offset = self.step_length * (swing_progress - 0.5) * self.linear_velocity
            y_offset = 0.0
            z_offset = self.step_height * math.sin(swing_progress * math.pi)
        
        # Apply angular velocity (turning)
        angular_offset = self.angular_velocity * math.sin(leg_angle) * phase
        
        # Convert to joint angles (simplified inverse kinematics)
        coxa_angle = angular_offset
        
        # Simplified leg positioning
        femur_angle = -0.5 + z_offset * 2.0  # Lift leg for swing
        tibia_angle = 1.0 - z_offset * 1.5   # Compensate for femur
        
        return coxa_angle, femur_angle, tibia_angle
    
    def publish_joint_commands(self):
        """Publish joint position commands"""
        msg = Float64MultiArray()
        msg.data = self.target_positions
        self.position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    controller = HexapodROS2ControlWalkingController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
