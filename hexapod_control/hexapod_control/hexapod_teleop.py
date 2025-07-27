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

        # Leg mounting angles (angle from body centerline to leg)
        # These determine the natural direction each leg should step
        self.leg_angles = {
            1: 60.0,   # Front-left: 60° from forward
            2: 90.0,   # Middle-left: 90° from forward (pure sideways)
            3: 120.0,  # Back-left: 120° from forward
            4: -60.0,  # Front-right: -60° from forward
            5: -90.0,  # Middle-right: -90° from forward (pure sideways)
            6: -120.0  # Back-right: -120° from forward
        }

        # Movement parameters
        self.step_height = 0.1        # Foot lift height during swing
        self.step_angle = 0.8          # Coxa rotation for each step (radians)
        self.lift_femur_offset = 0.2   # Femur adjustment for lifting
        self.lift_tibia_offset = 0.3   # Tibia adjustment for lifting
        self.cycle_time = 2.0          # Time for complete gait cycle
        self.points_per_phase = 10     # Trajectory points per phase
        
        # Spider gait: alternating tripods
        self.tripod_1 = [1, 3, 5]  # Left-front, Left-back, Right-middle
        self.tripod_2 = [2, 4, 6]  # Left-middle, Right-front, Right-back
        
        # Movement state
        self.is_moving = False
        self.movement_thread = None
        
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
        ║ Spider Layout:                                                ║
        ║   LEFT:  1(front) - 2(middle) - 3(back)                       ║
        ║   RIGHT: 4(front) - 5(middle) - 6(back)                       ║
        ║                                                               ║
        ║ Movement Controls:                                            ║
        ║   w/W : Walk forward (spider tripod gait)                     ║
        ║   s/S : Walk backward (spider tripod gait)                    ║
        ║   a/A : Turn left (body rotation)                             ║
        ║   d/D : Turn right (body rotation)                            ║
        ║   q/Q : Diagonal forward-left                                 ║
        ║   e/E : Diagonal forward-right                                ║
        ║                                                               ║
        ║ Control Commands:                                             ║
        ║   x/X : Stop movement                                         ║
        ║   r/R : Reset to neutral stance                               ║
        ║   SPACE : Emergency stop                                      ║
        ║   ESC/Ctrl+C : Exit program                                   ║
        ║                                                               ║
        ║ Status: Ready for joint-based gait control                    ║
        ╚═══════════════════════════════════════════════════════════════╝
        
        Spider ready to walk...
        """
        print(instructions)

    def clamp_joint_angle(self, angle, joint_type):
        """Clamp joint angle within limits."""
        limits = self.joint_limits[joint_type]
        return max(limits[0], min(limits[1], angle))

    def generate_step_angles(self, leg_num, direction, step_phase, total_phases):
        """Generate joint angles for a specific step phase."""
        base_angles = self.neutral_angles[leg_num].copy()
        coxa_base, femur_base, tibia_base = base_angles
        
        # Get leg mounting angle
        leg_angle = math.radians(self.leg_angles[leg_num])
        
        # Calculate coxa rotation based on direction and leg mounting angle
        coxa_rotation = 0.0
        
        if direction == "forward":
            # For forward motion, each leg needs to step in the direction that
            # contributes to forward body movement based on its mounting angle
            forward_component = math.cos(leg_angle)  # How much this leg contributes to forward motion
            coxa_rotation = self.step_angle * forward_component
            
        elif direction == "backward":
            # Opposite of forward
            forward_component = math.cos(leg_angle)
            coxa_rotation = -self.step_angle * forward_component
            
        elif direction == "turn_left":
            # For turning, all legs rotate in same direction relative to their mounting
            coxa_rotation = self.step_angle * 0.6
            
        elif direction == "turn_right":
            coxa_rotation = -self.step_angle * 0.6
            
        elif direction == "forward_left":
            # Combine forward and left turn components
            forward_component = math.cos(leg_angle)
            side_component = math.sin(leg_angle)
            coxa_rotation = self.step_angle * (0.7 * forward_component + 0.3 * side_component)
                
        elif direction == "forward_right":
            # Combine forward and right turn components
            forward_component = math.cos(leg_angle)
            side_component = math.sin(leg_angle)
            coxa_rotation = self.step_angle * (0.7 * forward_component - 0.3 * side_component)

        # Apply step phase (smooth transition)
        phase_ratio = step_phase / (total_phases - 1)
        smooth_ratio = 0.5 * (1 - math.cos(phase_ratio * math.pi))  # Smooth S-curve
        
        coxa_angle = coxa_base + coxa_rotation * smooth_ratio
        
        # Apply joint limits
        coxa_angle = self.clamp_joint_angle(coxa_angle, 'coxa')
        
        return [coxa_angle, femur_base, tibia_base]

    def generate_lift_angles(self, leg_num, lift_phase, total_phases):
        """Generate joint angles for lifting leg during swing phase."""
        base_angles = self.neutral_angles[leg_num].copy()
        coxa_base, femur_base, tibia_base = base_angles
        
        # Create smooth lift trajectory
        phase_ratio = lift_phase / (total_phases - 1)
        
        # Use sine wave for smooth lift and lower
        lift_factor = math.sin(phase_ratio * math.pi)
        
        # Adjust femur and tibia for lifting
        femur_angle = femur_base + self.lift_femur_offset * lift_factor
        tibia_angle = tibia_base + self.lift_tibia_offset * lift_factor
        
        # Apply joint limits
        femur_angle = self.clamp_joint_angle(femur_angle, 'femur')
        tibia_angle = self.clamp_joint_angle(tibia_angle, 'tibia')
        
        return [coxa_base, femur_angle, tibia_angle]

    def execute_tripod_gait_step(self, direction):
        """Execute one complete tripod gait cycle."""
        phase_duration = self.cycle_time / (2 * self.points_per_phase)  # Time per trajectory point
        
        self.get_logger().info(f'Executing tripod gait: {direction}')
        
        # Phase 1: Tripod 1 swings, Tripod 2 supports
        self.get_logger().debug(f'Phase 1: Tripod 1 ({self.tripod_1}) swings, Tripod 2 ({self.tripod_2}) supports')
        for phase in range(self.points_per_phase):
            if not self.is_moving:
                return
                
            for leg in range(1, 7):
                if leg in self.tripod_1:
                    # Swing phase: lift and step
                    if phase < self.points_per_phase // 2:
                        # First half: lift
                        angles = self.generate_lift_angles(leg, phase, self.points_per_phase // 2)
                        if phase == 0:  # Debug first step
                            self.get_logger().debug(f'Leg {leg} lifting: {[round(a, 3) for a in angles]}')
                    else:
                        # Second half: step forward and lower
                        step_phase = phase - self.points_per_phase // 2
                        lift_angles = self.generate_lift_angles(leg, 
                                                              self.points_per_phase // 2 - step_phase - 1, 
                                                              self.points_per_phase // 2)
                        step_angles = self.generate_step_angles(leg, direction, step_phase, 
                                                              self.points_per_phase // 2)
                        # Combine lift and step
                        angles = [step_angles[0], lift_angles[1], lift_angles[2]]
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
                    # Swing phase: lift and step
                    if phase < self.points_per_phase // 2:
                        # First half: lift
                        angles = self.generate_lift_angles(leg, phase, self.points_per_phase // 2)
                        if phase == 0:  # Debug first step
                            self.get_logger().debug(f'Leg {leg} lifting: {[round(a, 3) for a in angles]}')
                    else:
                        # Second half: step forward and lower
                        step_phase = phase - self.points_per_phase // 2
                        lift_angles = self.generate_lift_angles(leg, 
                                                              self.points_per_phase // 2 - step_phase - 1, 
                                                              self.points_per_phase // 2)
                        step_angles = self.generate_step_angles(leg, direction, step_phase, 
                                                              self.points_per_phase // 2)
                        # Combine lift and step
                        angles = [step_angles[0], lift_angles[1], lift_angles[2]]
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
        """Send joint trajectory command."""
        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names[leg_num]
        
        point = JointTrajectoryPoint()
        point.positions = list(map(float, joint_angles))
        point.velocities = [0.0] * 3
        point.accelerations = [0.0] * 3
        point.time_from_start = duration
        
        traj_msg.points.append(point)
        self.trajectory_publishers[leg_num].publish(traj_msg)
        
        # Update current angles
        self.current_angles[leg_num] = joint_angles.copy()

    def reset_to_neutral_stance(self):
        """Move to neutral spider stance."""
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