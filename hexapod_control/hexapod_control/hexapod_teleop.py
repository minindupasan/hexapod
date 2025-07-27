#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import threading
import sys
import select
import termios
import tty
import math

class HexapodSpiderTeleop(Node):
    def __init__(self):
        super().__init__('hexapod_spider_teleop')
        self.get_logger().info('Initializing Spider Hexapod Controller...')

        # Publishers for each leg controller
        self.trajectory_publishers = {
            1: self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', 10),
            2: self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', 10),
            3: self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', 10),
            4: self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', 10),
            5: self.create_publisher(JointTrajectory, '/leg5_controller/joint_trajectory', 10),
            6: self.create_publisher(JointTrajectory, '/leg6_controller/joint_trajectory', 10),
        }

        # Joint names
        self.joint_names = {
            1: ['coxa_joint_1', 'femur_joint_1', 'tibia_joint_1'],
            2: ['coxa_joint_2', 'femur_joint_2', 'tibia_joint_2'], 
            3: ['coxa_joint_3', 'femur_joint_3', 'tibia_joint_3'],
            4: ['coxa_joint_4', 'femur_joint_4', 'tibia_joint_4'],
            5: ['coxa_joint_5', 'femur_joint_5', 'tibia_joint_5'],
            6: ['coxa_joint_6', 'femur_joint_6', 'tibia_joint_6'],
        }

        # Physical robot parameters
        self.l1 = 0.05811  # Coxa length
        self.l2 = 0.1208   # Femur length  
        self.l3 = 0.209    # Tibia length

        # Initial stance angles from YAML configuration
        self.neutral_angles = {
            1: [0.0, 0.4, -1.7],   # coxa, femur, tibia
            2: [0.0, 0.4, -1.7],
            3: [0.0, 0.4, -1.7],
            4: [0.0, 0.4, -1.7],
            5: [0.0, 0.4, -1.7],
            6: [0.0, 0.4, -1.7]
        }

        # Current joint angles (will be updated during movement)
        self.current_angles = {}
        for leg in range(1, 7):
            self.current_angles[leg] = self.neutral_angles[leg].copy()

        # Joint limits (from URDF)
        self.joint_limits = {
            'coxa': [-1.5708, 1.5708],    # ±90°
            'femur': [-1.5708, 1.5708],   # ±90° 
            'tibia': [-2.35619, 0.0]      # -135° to 0°
        }

        # Spider body dimensions and stance
        self.body_height = 0.08    # Target body height above ground
        
        # Spider leg base positions (relative to body center)
        self.leg_base_positions = {
            # LEFT SIDE legs (positive Y)
            1: np.array([0.08, 0.12, 0.014]),    # Front-left
            2: np.array([0.0, 0.14, 0.014]),     # Middle-left  
            3: np.array([-0.08, 0.12, 0.014]),   # Back-left
            
            # RIGHT SIDE legs (negative Y)  
            4: np.array([0.08, -0.12, 0.014]),   # Front-right
            5: np.array([0.0, -0.14, 0.014]),    # Middle-right
            6: np.array([-0.08, -0.12, 0.014])   # Back-right
        }

        # FIXED: Leg mounting angles corrected for proper forward direction
        # Forward is now correctly toward legs 1 and 4 (positive X direction)
        # 0° = positive X axis (toward legs 1&4), positive = counterclockwise
        self.leg_angles = {
            1: 45.0,    # Front-left: 45° from X axis
            2: 90.0,    # Middle-left: 90° from X axis (pointing in Y direction)
            3: 135.0,   # Back-left: 135° from X axis
            4: -45.0,   # Front-right: -45° from X axis
            5: -90.0,   # Middle-right: -90° from X axis (pointing in -Y direction)
            6: -135.0   # Back-right: -135° from X axis
        }

        # Movement parameters
        self.step_height = 5        # Foot lift height during swing (default 1)
        self.base_step_angle = 2     # Base coxa rotation for each step (radians) 
        self.lift_femur_offset = 0.25   # Femur adjustment for smooth lifting (default 0.25 radians)
        self.lift_tibia_offset = 0.35   # Tibia adjustment for smooth lifting (default 0.35 radians)
        self.base_cycle_time = 1.5      # Base time for complete gait cycle (default 1.5 seconds)
        self.points_per_phase = 40      # More trajectory points for smoothness (default 20)
        
        # Extended speed control with more profiles
        self.speed_levels = {
            'crawl': 0.3,      # 30% speed - very slow
            'slow': 0.5,       # 50% speed
            'normal': 1.0,     # 100% speed
            'fast': 1.5,       # 150% speed
            'turbo': 2.0,      # 200% speed
            'sprint': 2.5,     # 250% speed
            'blitz': 3.0,      # 300% speed
            'hyper': 5.0,      # 500% speed
            'max': 20.0         # 2000% speed - maximum
        }
        self.current_speed = 'normal'
        self.speed_multiplier = self.speed_levels[self.current_speed]
        
        # Spider gait: alternating tripods
        self.tripod_1 = [1, 3, 5]  # Left-front, Left-back, Right-middle
        self.tripod_2 = [2, 4, 6]  # Left-middle, Right-front, Right-back
        
        # Movement state
        self.is_moving = False
        self.movement_thread = None
        
        # Smooth transition parameters
        self.use_smooth_transitions = True
        self.transition_smoothness = 0.8  # Smoothing factor (0-1)
        
        # Terminal setup
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        self.print_instructions()

    def print_instructions(self):
        instructions = """
        ╔═══════════════════════════════════════════════════════════════╗
        ║                SPIDER HEXAPOD LOCOMOTION CONTROLLER           ║
        ║              (Joint-Based Tripod Gait Control)                ║
        ╠═══════════════════════════════════════════════════════════════╣
        ║ Spider Layout (Forward = toward legs 1&4):                    ║
        ║            Forward ↑                                          ║
        ║       ┌─────────────────┐                                     ║
        ║  Left │ 1           4   │ Right                               ║
        ║       │ 2           5   │                                     ║  
        ║       │ 3           6   │                                     ║
        ║       └─────────────────┘                                     ║
        ║            Backward ↓                                         ║
        ║                                                               ║
        ║ Movement Controls:                                            ║
        ║   w/W : Walk forward (spider tripod gait)                     ║
        ║   s/S : Walk backward (spider tripod gait)                    ║
        ║   a/A : Turn left (body rotation)                             ║
        ║   d/D : Turn right (body rotation)                            ║
        ║   q/Q : Diagonal forward-left                                 ║
        ║   e/E : Diagonal forward-right                                ║
        ║                                                               ║
        ║ Speed Controls:                                               ║
        ║   1 : Crawl speed (30%)                                       ║
        ║   2 : Slow speed (50%)                                        ║
        ║   3 : Normal speed (100%) [Default]                           ║
        ║   4 : Fast speed (150%)                                       ║
        ║   5 : Turbo speed (200%)                                      ║
        ║   6 : Sprint speed (250%)                                     ║
        ║   7 : Blitz speed (300%)                                      ║
        ║   8 : Hyper speed (500%)                                      ║
        ║   9 : Max speed (1000%)                                       ║
        ║                                                               ║
        ║ Control Commands:                                             ║
        ║   x/X : Stop movement                                         ║
        ║   r/R : Reset to neutral stance                               ║
        ║   SPACE : Emergency stop                                      ║
        ║   ESC/Ctrl+C : Exit program                                   ║
        ║                                                               ║
        ║ Status: Ready for joint-based gait control                    ║
        ║ Current Speed: {:<8} ({:>3}%)                                 ║
        ╚═══════════════════════════════════════════════════════════════╝
        
        Spider ready to walk...
        """.format(self.current_speed.upper(), int(self.speed_levels[self.current_speed] * 100))
        print(instructions)

    def get_current_movement_params(self):
        """Get current movement parameters based on speed setting."""
        speed_mult = self.speed_levels[self.current_speed]
        
        # Higher speed = larger steps and faster cycle
        step_angle = self.base_step_angle * speed_mult
        cycle_time = self.base_cycle_time / speed_mult
        
        # Clamp values to safe ranges
        step_angle = min(step_angle, 0.5)  # Max step angle for safety
        cycle_time = max(cycle_time, 0.5)  # Min cycle time for stability
        
        return step_angle, cycle_time

    def set_speed(self, speed_name):
        """Set movement speed."""
        if speed_name in self.speed_levels:
            self.current_speed = speed_name
            self.speed_multiplier = self.speed_levels[speed_name]
            self.get_logger().info(f'Speed set to: {speed_name} ({int(self.speed_multiplier*100)}%)')
        else:
            self.get_logger().warn(f'Invalid speed: {speed_name}. Available: {list(self.speed_levels.keys())}')

    def clamp_joint_angle(self, angle, joint_type):
        """Clamp joint angle within limits."""
        limits = self.joint_limits[joint_type]
        return max(limits[0], min(limits[1], angle))

    def smooth_trajectory(self, start_val, end_val, phase, total_phases):
        """Generate smooth trajectory between two values using cosine interpolation."""
        if total_phases <= 1:
            return end_val
        
        # Cosine interpolation for smoother motion
        t = phase / (total_phases - 1)
        cos_val = (1 - math.cos(t * math.pi)) / 2
        
        return start_val + (end_val - start_val) * cos_val

    def generate_step_angles(self, leg_num, direction, step_phase, total_phases):
        """Generate joint angles for a specific step phase."""
        base_angles = self.neutral_angles[leg_num].copy()
        coxa_base, femur_base, tibia_base = base_angles

        # Get current movement parameters based on speed
        step_angle, _ = self.get_current_movement_params()

        # Get leg mounting angle
        leg_angle = math.radians(self.leg_angles[leg_num])

        # Calculate coxa rotation based on direction and leg mounting angle
        coxa_rotation = 0.0

        # --- DIRECTION LOGIC SWAP ---
        if direction == "forward":
            # Now forward moves in the direction of previous 'turn right' (i.e., rotate coxa for all legs as in turn right)
            tangent_component = -1.0 if leg_num in [1, 2, 3] else 1.0
            coxa_rotation = step_angle * 0.8 * tangent_component

        elif direction == "backward":
            # Opposite of new forward
            tangent_component = 1.0 if leg_num in [1, 2, 3] else -1.0
            coxa_rotation = step_angle * 0.8 * tangent_component

        elif direction == "turn_left":
            # True rotation: left turn = counterclockwise
            # All legs rotate coxa in + direction
            coxa_rotation = step_angle * 1.0

        elif direction == "turn_right":
            # True rotation: right turn = clockwise
            # All legs rotate coxa in - direction
            coxa_rotation = -step_angle * 1.0

        elif direction == "forward_left":
            # Diagonal: blend new forward and left
            tangent_component = -1.0 if leg_num in [1, 2, 3] else 1.0
            coxa_rotation = step_angle * (0.7 * tangent_component + 0.3 * 1.0)

        elif direction == "forward_right":
            # Diagonal: blend new forward and right
            tangent_component = -1.0 if leg_num in [1, 2, 3] else 1.0
            coxa_rotation = step_angle * (0.7 * tangent_component - 0.3 * 1.0)

        # Use smooth trajectory for step motion
        if self.use_smooth_transitions:
            coxa_angle = self.smooth_trajectory(coxa_base, coxa_base + coxa_rotation, 
                                              step_phase, total_phases)
        else:
            if total_phases > 1:
                phase_ratio = step_phase / (total_phases - 1)
                smooth_ratio = phase_ratio * phase_ratio * (3.0 - 2.0 * phase_ratio)
            else:
                smooth_ratio = 1.0
            coxa_angle = coxa_base + coxa_rotation * smooth_ratio

        # Apply joint limits
        coxa_angle = self.clamp_joint_angle(coxa_angle, 'coxa')

        return [coxa_angle, femur_base, tibia_base]

    def generate_lift_angles(self, leg_num, lift_phase, total_phases):
        """Generate joint angles for lifting leg during swing phase with smooth motion (improved tibia smoothness)."""
        base_angles = self.neutral_angles[leg_num].copy()
        coxa_base, femur_base, tibia_base = base_angles

        # Create smooth lift trajectory using sine wave for natural motion
        if total_phases > 1:
            phase_ratio = lift_phase / (total_phases - 1)
        else:
            phase_ratio = 1.0

        # Use sine wave for smooth lifting and lowering
        lift_factor = math.sin(phase_ratio * math.pi)
        lift_factor = max(0.0, min(1.0, lift_factor))  # Clamp for safety

        # Apply smoothing factor to reduce jerkiness
        lift_factor *= self.transition_smoothness

        # Improved: Use cosine interpolation for both femur and tibia, but for tibia, blend more gently
        if self.use_smooth_transitions:
            femur_target = femur_base + self.lift_femur_offset
            tibia_target = tibia_base + self.lift_tibia_offset

            femur_angle = self.smooth_trajectory(femur_base, femur_target, lift_phase, total_phases)
            # For tibia, use a double-smooth blend to avoid glitches
            tibia_mid = (tibia_base + tibia_target) / 2
            tibia_angle = self.smooth_trajectory(tibia_base, tibia_mid, lift_phase, total_phases)
            tibia_angle = self.smooth_trajectory(tibia_angle, tibia_target, int(lift_phase/2), int(total_phases/2) if total_phases>2 else 1)

            # Apply lift factor for arc motion
            femur_angle = femur_base + (femur_angle - femur_base) * lift_factor
            tibia_angle = tibia_base + (tibia_angle - tibia_base) * lift_factor
        else:
            femur_angle = femur_base + self.lift_femur_offset * lift_factor
            tibia_angle = tibia_base + self.lift_tibia_offset * lift_factor

        # Apply joint limits
        femur_angle = self.clamp_joint_angle(femur_angle, 'femur')
        tibia_angle = self.clamp_joint_angle(tibia_angle, 'tibia')

        return [coxa_base, femur_angle, tibia_angle]

    def execute_tripod_gait_step(self, direction):
        """Execute one complete tripod gait cycle with smooth transitions."""
        # Get current speed parameters
        _, cycle_time = self.get_current_movement_params()
        phase_duration = cycle_time / (2 * self.points_per_phase)  # Time per trajectory point
        
        self.get_logger().info(f'Executing tripod gait: {direction} at {self.current_speed} speed')
        
        # Phase 1: Tripod 1 swings, Tripod 2 supports
        self.get_logger().debug(f'Phase 1: Tripod 1 ({self.tripod_1}) swings, Tripod 2 ({self.tripod_2}) supports')
        for phase in range(self.points_per_phase):
            if not self.is_moving:
                return
                
            for leg in range(1, 7):
                if leg in self.tripod_1:
                    # Swing phase: lift and step with smooth transition
                    if phase < self.points_per_phase // 2:
                        # First half: smooth lift
                        angles = self.generate_lift_angles(leg, phase, self.points_per_phase // 2)
                        if phase == 0:  # Debug first step
                            self.get_logger().debug(f'Leg {leg} lifting: {[round(a, 3) for a in angles]}')
                    else:
                        # Second half: step forward while lowering smoothly
                        step_phase = phase - self.points_per_phase // 2
                        lift_phase = self.points_per_phase // 2 - step_phase - 1
                        
                        # Generate smooth combined motion
                        lift_angles = self.generate_lift_angles(leg, lift_phase, self.points_per_phase // 2)
                        step_angles = self.generate_step_angles(leg, direction, step_phase, 
                                                              self.points_per_phase // 2)
                        
                        # Smoothly blend lift and step motions
                        blend_factor = step_phase / (self.points_per_phase // 2 - 1) if self.points_per_phase > 2 else 1.0
                        angles = [
                            step_angles[0],  # Coxa follows step trajectory
                            lift_angles[1] * (1 - blend_factor) + step_angles[1] * blend_factor,  # Smooth femur blend
                            lift_angles[2] * (1 - blend_factor) + step_angles[2] * blend_factor   # Smooth tibia blend
                        ]
                        
                        if step_phase == 0:  # Debug first step
                            leg_angle_deg = self.leg_angles[leg]
                            self.get_logger().debug(f'Leg {leg} (angle {leg_angle_deg}°) stepping: {[round(a, 3) for a in angles]}')
                else:
                    # Stance phase: support body (maintain neutral)
                    angles = self.neutral_angles[leg].copy()
                
                self.send_joint_command(leg, angles, phase_duration)
            
            time.sleep(phase_duration)
        
        # Phase 2: Tripod 2 swings, Tripod 1 supports
        self.get_logger().debug(f'Phase 2: Tripod 2 ({self.tripod_2}) swings, Tripod 1 ({self.tripod_1}) supports')
        for phase in range(self.points_per_phase):
            if not self.is_moving:
                return
                
            for leg in range(1, 7):
                if leg in self.tripod_2:
                    # Swing phase: lift and step with smooth transition
                    if phase < self.points_per_phase // 2:
                        # First half: smooth lift
                        angles = self.generate_lift_angles(leg, phase, self.points_per_phase // 2)
                        if phase == 0:  # Debug first step
                            self.get_logger().debug(f'Leg {leg} lifting: {[round(a, 3) for a in angles]}')
                    else:
                        # Second half: step forward while lowering smoothly
                        step_phase = phase - self.points_per_phase // 2
                        lift_phase = self.points_per_phase // 2 - step_phase - 1
                        
                        # Generate smooth combined motion
                        lift_angles = self.generate_lift_angles(leg, lift_phase, self.points_per_phase // 2)
                        step_angles = self.generate_step_angles(leg, direction, step_phase, 
                                                              self.points_per_phase // 2)
                        
                        # Smoothly blend lift and step motions
                        blend_factor = step_phase / (self.points_per_phase // 2 - 1) if self.points_per_phase > 2 else 1.0
                        angles = [
                            step_angles[0],  # Coxa follows step trajectory
                            lift_angles[1] * (1 - blend_factor) + step_angles[1] * blend_factor,  # Smooth femur blend
                            lift_angles[2] * (1 - blend_factor) + step_angles[2] * blend_factor   # Smooth tibia blend
                        ]
                        
                        if step_phase == 0:  # Debug first step
                            leg_angle_deg = self.leg_angles[leg]
                            self.get_logger().debug(f'Leg {leg} (angle {leg_angle_deg}°) stepping: {[round(a, 3) for a in angles]}')
                else:
                    # Stance phase: support body (maintain neutral)
                    angles = self.neutral_angles[leg].copy()
                
                self.send_joint_command(leg, angles, phase_duration)
            
            time.sleep(phase_duration)

    def execute_continuous_gait(self, direction):
        """Execute continuous tripod gait."""
        self.is_moving = True
        
        while self.is_moving:
            self.execute_tripod_gait_step(direction)

    def send_joint_command(self, leg_num, joint_angles, duration_sec):
        """Send joint trajectory command with velocity and acceleration profiles."""
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names[leg_num]
        
        point = JointTrajectoryPoint()
        point.positions = list(map(float, joint_angles))
        
        # Calculate smooth velocities based on position changes
        if leg_num in self.current_angles:
            velocities = []
            for i in range(3):
                vel = (joint_angles[i] - self.current_angles[leg_num][i]) / duration_sec
                # Apply velocity smoothing
                vel *= 0.8  # Reduce velocity for smoother motion
                velocities.append(vel)
            point.velocities = velocities
        else:
            point.velocities = [0.0] * 3
        
        point.accelerations = [0.0] * 3
        point.time_from_start = duration
        
        traj_msg.points.append(point)
        self.trajectory_publishers[leg_num].publish(traj_msg)
        
        # Update current angles
        self.current_angles[leg_num] = joint_angles.copy()

    def reset_to_neutral_stance(self):
        """Move to neutral spider stance with smooth transition."""
        self.get_logger().info('Moving to neutral spider stance...')
        self.is_moving = False
        
        duration = Duration()
        duration.sec = 2
        duration.nanosec = 0
        
        for leg_num in range(1, 7):
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names[leg_num]
            
            point = JointTrajectoryPoint()
            point.positions = list(map(float, self.neutral_angles[leg_num]))
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = duration
            
            traj_msg.points.append(point)
            self.trajectory_publishers[leg_num].publish(traj_msg)
            
            # Update current angles
            self.current_angles[leg_num] = self.neutral_angles[leg_num].copy()
        
        time.sleep(2.5)

    def emergency_stop(self):
        """Stop all movement immediately."""
        self.get_logger().warn('SPIDER EMERGENCY STOP!')
        self.is_moving = False
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1.0)

    def get_key(self):
        """Get keyboard input without blocking."""
        if select.select([sys.stdin], [], [], 0.05) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def process_key(self, key):
        """Process keyboard commands."""
        if key is None:
            return True

        key_lower = key.lower()
        
        # Stop current movement
        if self.is_moving:
            self.emergency_stop()
            time.sleep(0.2)

        if key_lower == 'w':  # Forward
            self.get_logger().info('Spider walking forward...')
            self.movement_thread = threading.Thread(target=self.execute_continuous_gait, args=("forward",))
            self.movement_thread.start()
            
        elif key_lower == 's':  # Backward
            self.get_logger().info('Spider walking backward...')
            self.movement_thread = threading.Thread(target=self.execute_continuous_gait, args=("backward",))
            self.movement_thread.start()
            
        elif key_lower == 'a':  # Turn left
            self.get_logger().info('Spider turning left...')
            self.movement_thread = threading.Thread(target=self.execute_continuous_gait, args=("turn_left",))
            self.movement_thread.start()
            
        elif key_lower == 'd':  # Turn right
            self.get_logger().info('Spider turning right...')
            self.movement_thread = threading.Thread(target=self.execute_continuous_gait, args=("turn_right",))
            self.movement_thread.start()
            
        elif key_lower == 'q':  # Forward-left
            self.get_logger().info('Spider walking forward-left...')
            self.movement_thread = threading.Thread(target=self.execute_continuous_gait, args=("forward_left",))
            self.movement_thread.start()
            
        elif key_lower == 'e':  # Forward-right
            self.get_logger().info('Spider walking forward-right...')
            self.movement_thread = threading.Thread(target=self.execute_continuous_gait, args=("forward_right",))
            self.movement_thread.start()
            
        elif key == '1':  # Crawl speed
            self.set_speed('crawl')
            
        elif key == '2':  # Slow speed
            self.set_speed('slow')
            
        elif key == '3':  # Normal speed
            self.set_speed('normal')
            
        elif key == '4':  # Fast speed
            self.set_speed('fast')
            
        elif key == '5':  # Turbo speed
            self.set_speed('turbo')
            
        elif key == '6':  # Sprint speed
            self.set_speed('sprint')
        
        elif key == '7':  # Blitz speed
            self.set_speed('blitz')

        elif key == '8':  # Hyper speed
            self.set_speed('hyper')

        elif key == '9':  # Max speed
            self.set_speed('max')
            
        elif key_lower == 'x':  # Stop
            self.get_logger().info('Stopping spider...')
            self.emergency_stop()
            
        elif key_lower == 'r':  # Reset stance
            self.get_logger().info('Resetting to neutral stance...')
            self.emergency_stop()
            threading.Thread(target=self.reset_to_neutral_stance).start()
            
        elif key == ' ':  # Emergency stop
            self.emergency_stop()
            
        elif key == '\x1b' or ord(key) == 3:  # ESC or Ctrl+C
            self.get_logger().info('Exiting spider controller...')
            return False
            
        else:
            if ord(key) > 31:
                self.get_logger().info(f'Unknown command: "{key}"')

        return True

    def run(self):
        """Main control loop."""
        try:
            # Move to neutral stance first
            self.reset_to_neutral_stance()
            
            self.get_logger().info('Spider controller ready! Use WASD for movement.')
            
            while rclpy.ok():
                key = self.get_key()
                if not self.process_key(key):
                    break
                    
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt')
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources."""
        self.emergency_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
        self.get_logger().info('Spider controller shutdown complete')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        spider_teleop = HexapodSpiderTeleop()
        spider_teleop.run()
    except Exception as e:
        print(f'Spider Controller Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()