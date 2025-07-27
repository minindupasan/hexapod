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
        self.get_logger().info('Initializing Hexapod Spider Gait Controller...')

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

        # Physical robot parameters (from your original code)
        self.l1 = 0.05811  # Coxa length: 0.17995 - 0.12184
        self.l2 = 0.1208   # Femur length: 0.30075 - 0.17995  
        self.l3 = 0.209    # Tibia length: 0.50975 - 0.30075

        # Initial joint positions from your YAML (radians)
        self.initial_angles = {
            1: [0.0, 0, -1.7],
            2: [0.0, 0, -1.7],
            3: [0.0, 0, -1.7],
            4: [0.0, 0, -1.7],
            5: [0.0, 0, -1.7],
            6: [0.0, 0, -1.7]
        }

        # Leg base positions relative to body center (hexagon layout)
        self.leg_base_positions = {
            1: [0.12184, -0.12184 * math.tan(math.pi/6), 0],    # Front-right (30°)
            2: [0.0, -0.12184, 0],                              # Middle-right (90°)
            3: [-0.12184, -0.12184 * math.tan(math.pi/6), 0],  # Rear-right (150°)
            4: [-0.12184, 0.12184 * math.tan(math.pi/6), 0],   # Rear-left (210°)
            5: [0.0, 0.12184, 0],                               # Middle-left (270°)
            6: [0.12184, 0.12184 * math.tan(math.pi/6), 0]     # Front-left (330°)
        }

        # Calculate neutral foot positions using forward kinematics
        self.neutral_positions = {}
        for leg in range(1, 7):
            pos = self.forward_kinematics(self.initial_angles[leg], leg)
            self.neutral_positions[leg] = pos
            self.get_logger().info(f'Leg {leg} neutral position: {pos}')

        # Spider gait parameters
        self.step_length = 0.1        # Step distance (increased for more movement)
        self.step_height = 0.08        # Lift height during swing
        self.cycle_time = 1.2          # Total cycle time (slower for stability)
        self.duty_factor = 0.6         # Fraction of cycle in stance
        self.points_per_cycle = 30     # Trajectory resolution
        
        # Movement parameters for better coxa engagement
        self.stride_multiplier = 1.5   # Multiply step length for better reach
        
        # Tripod groups (classic spider gait)
        self.tripod_1 = [1, 3, 5]  # Right-front, Right-rear, Left-middle
        self.tripod_2 = [2, 4, 6]  # Right-middle, Left-rear, Left-front
        
        # Movement state
        self.is_moving = False
        self.movement_thread = None
        self.current_phase = 0
        
        # Terminal setup
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        self.print_instructions()

    def print_instructions(self):
        instructions = """
        ╔══════════════════════════════════════════════════════════════╗
        ║                 HEXAPOD SPIDER GAIT CONTROLLER               ║
        ╠══════════════════════════════════════════════════════════════╣
        ║ Movement Controls:                                           ║
        ║   w/W : Walk forward with spider gait                       ║
        ║   s/S : Walk backward with spider gait                      ║
        ║   a/A : Turn left (rotate body)                             ║
        ║   d/D : Turn right (rotate body)                            ║
        ║   q/Q : Walk forward-left diagonal                          ║
        ║   e/E : Walk forward-right diagonal                         ║
        ║                                                              ║
        ║ Control Commands:                                            ║
        ║   x/X : Stop movement and return to neutral                 ║
        ║   r/R : Reset to initial standing position                  ║
        ║   SPACE : Emergency stop                                     ║
        ║   ESC/Ctrl+C : Exit program                                  ║
        ║                                                              ║
        ║ Spider Gait: Alternating tripod pattern (3 legs at a time)  ║
        ╚══════════════════════════════════════════════════════════════╝
        
        Ready for commands...
        """
        print(instructions)

    def forward_kinematics(self, joint_angles, leg_num):
        """Calculate foot position from joint angles in global coordinates."""
        alpha1, alpha2, alpha3 = joint_angles
        
        # Local leg coordinates
        x_local = self.l1 + self.l2 * math.cos(alpha2) + self.l3 * math.cos(alpha2 + alpha3)
        y_local = 0
        z_local = -self.l2 * math.sin(alpha2) - self.l3 * math.sin(alpha2 + alpha3)
        
        # Rotate by coxa angle
        x_rotated = x_local * math.cos(alpha1) - y_local * math.sin(alpha1)
        y_rotated = x_local * math.sin(alpha1) + y_local * math.cos(alpha1)
        
        # Transform to global coordinates
        base_pos = self.leg_base_positions[leg_num]
        global_x = base_pos[0] + x_rotated
        global_y = base_pos[1] + y_rotated  
        global_z = base_pos[2] + z_local
        
        return np.array([global_x, global_y, global_z])

    def inverse_kinematics(self, target_pos, leg_num):
        """Calculate joint angles from target foot position."""
        # Transform to leg base coordinates
        base_pos = self.leg_base_positions[leg_num]
        local_x = target_pos[0] - base_pos[0]
        local_y = target_pos[1] - base_pos[1]
        local_z = target_pos[2] - base_pos[2]
        
        # Coxa angle (horizontal rotation) - this is crucial for movement!
        alpha1 = math.atan2(local_y, local_x)
        
        # Distance from coxa joint axis to target (in horizontal plane)
        horizontal_distance = math.sqrt(local_x**2 + local_y**2)
        
        # Distance from coxa end to target
        r = horizontal_distance - self.l1
        h = -local_z  # Height (positive up)
        
        # Distance from femur joint to foot
        d = math.sqrt(r**2 + h**2)
        
        # Check if target is reachable
        max_reach = self.l2 + self.l3
        min_reach = abs(self.l2 - self.l3)
        
        if d > max_reach:
            # Scale down if too far
            scale = max_reach * 0.95 / d
            r *= scale
            h *= scale
            d = math.sqrt(r**2 + h**2)
            self.get_logger().debug(f'Leg {leg_num}: Scaled target to reachable distance')
        elif d < min_reach:
            # Scale up if too close  
            scale = min_reach * 1.05 / d
            r *= scale
            h *= scale
            d = math.sqrt(r**2 + h**2)
            
        # Knee angle (using law of cosines)
        cos_alpha3 = (self.l2**2 + self.l3**2 - d**2) / (2 * self.l2 * self.l3)
        cos_alpha3 = max(-1, min(1, cos_alpha3))  # Clamp to valid range
        alpha3 = -math.acos(cos_alpha3)  # Negative for knee bend
        
        # Hip angle
        angle1 = math.atan2(h, r) if r != 0 else 0
        angle2 = math.acos((self.l2**2 + d**2 - self.l3**2) / (2 * self.l2 * d)) if d != 0 else 0
        alpha2 = angle1 + angle2
        
        return [alpha1, alpha2, alpha3]

    def generate_step_trajectory(self, start_pos, end_pos, is_swing_phase, num_points):
        """Generate trajectory for one step."""
        trajectory = []
        
        if is_swing_phase:
            # Swing phase: lift foot and move through air
            for i in range(num_points):
                t = i / (num_points - 1)
                
                # Linear interpolation in X and Y
                x = start_pos[0] + t * (end_pos[0] - start_pos[0])
                y = start_pos[1] + t * (end_pos[1] - start_pos[1])
                
                # Parabolic arc in Z (lift foot)
                z_base = start_pos[2] + t * (end_pos[2] - start_pos[2])
                z_lift = self.step_height * 4 * t * (1 - t)  # Parabola
                z = z_base + z_lift
                
                trajectory.append(np.array([x, y, z]))
        else:
            # Stance phase: keep foot on ground, move body over foot
            for i in range(num_points):
                t = i / (num_points - 1)
                x = start_pos[0] + t * (end_pos[0] - start_pos[0])
                y = start_pos[1] + t * (end_pos[1] - start_pos[1])
                z = start_pos[2] + t * (end_pos[2] - start_pos[2])
                trajectory.append(np.array([x, y, z]))
                
        return trajectory

    def execute_spider_gait(self, direction_vector):
        """Execute continuous spider gait in given direction."""
        self.is_moving = True
        
        # Scale direction vector for better coxa movement
        scaled_direction = direction_vector * self.stride_multiplier
        
        # Calculate step parameters
        swing_points = int(self.points_per_cycle * (1 - self.duty_factor))
        stance_points = int(self.points_per_cycle * self.duty_factor) 
        step_time = self.cycle_time / self.points_per_cycle
        
        phase = 0  # 0 = tripod_1 swings, 1 = tripod_2 swings
        
        self.get_logger().info(f'Starting gait with direction: {scaled_direction}')
        
        while self.is_moving:
            # Generate trajectories for current phase
            leg_trajectories = {}
            
            for leg in range(1, 7):
                neutral_pos = self.neutral_positions[leg]
                
                # Determine if this leg is in swing or stance phase
                in_tripod_1 = leg in self.tripod_1
                is_swing = (phase == 0 and in_tripod_1) or (phase == 1 and not in_tripod_1)
                
                if is_swing:
                    # Swing phase: move from rear to front position
                    # Start from a position behind neutral
                    start_pos = neutral_pos - scaled_direction * 0.6
                    # End at a position ahead of neutral  
                    end_pos = neutral_pos + scaled_direction * 0.6
                    trajectory = self.generate_step_trajectory(start_pos, end_pos, True, swing_points)
                    self.get_logger().debug(f'Leg {leg} SWING: {start_pos} -> {end_pos}')
                else:
                    # Stance phase: move from front to rear position (ground contact)
                    # Start from ahead of neutral
                    start_pos = neutral_pos + scaled_direction * 0.6
                    # End behind neutral
                    end_pos = neutral_pos - scaled_direction * 0.6  
                    trajectory = self.generate_step_trajectory(start_pos, end_pos, False, stance_points)
                    self.get_logger().debug(f'Leg {leg} STANCE: {start_pos} -> {end_pos}')
                
                leg_trajectories[leg] = trajectory
            
            # Execute trajectories synchronously
            max_points = max(len(leg_trajectories[leg]) for leg in range(1, 7))
            
            for step_idx in range(max_points):
                if not self.is_moving:
                    return
                
                # Send commands to all legs for this time step
                for leg in range(1, 7):
                    if step_idx < len(leg_trajectories[leg]):
                        target_pos = leg_trajectories[leg][step_idx]
                        joint_angles = self.inverse_kinematics(target_pos, leg)
                        
                        if joint_angles is not None:
                            # Log coxa movement for debugging
                            if step_idx % 10 == 0:  # Log every 10th step
                                self.get_logger().debug(f'Leg {leg} coxa angle: {math.degrees(joint_angles[0]):.1f}°')
                            self.send_joint_command(leg, joint_angles, step_time)
                        else:
                            self.get_logger().warn(f'IK failed for leg {leg} at step {step_idx}')
                
                time.sleep(step_time)
            
            # Switch to other tripod
            phase = 1 - phase
            self.get_logger().debug(f'Switching to phase {phase}')

    def execute_turn_gait(self, angular_velocity):
        """Execute turning gait with proper coxa movement."""
        self.is_moving = True
        
        swing_points = int(self.points_per_cycle * (1 - self.duty_factor))
        stance_points = int(self.points_per_cycle * self.duty_factor)
        step_time = self.cycle_time / self.points_per_cycle
        
        phase = 0
        
        # Calculate turn step size
        turn_step = angular_velocity * 0.15  # Increased turn step
        
        while self.is_moving:
            leg_trajectories = {}
            
            for leg in range(1, 7):
                neutral_pos = self.neutral_positions[leg]
                base_pos = np.array(self.leg_base_positions[leg])
                
                # Calculate tangential movement for rotation around body center
                radius_vector = base_pos[:2]  # Vector from center to leg base
                radius = np.linalg.norm(radius_vector)
                
                if radius > 0:
                    # Perpendicular vector for rotation (tangent)
                    tangent_unit = np.array([-radius_vector[1], radius_vector[0]]) / radius
                    # Scale by radius and turn rate
                    turn_vector = np.array([tangent_unit[0] * turn_step * radius, 
                                          tangent_unit[1] * turn_step * radius, 0])
                else:
                    turn_vector = np.array([0, 0, 0])
                
                in_tripod_1 = leg in self.tripod_1
                is_swing = (phase == 0 and in_tripod_1) or (phase == 1 and not in_tripod_1)
                
                if is_swing:
                    start_pos = neutral_pos - turn_vector * 0.8
                    end_pos = neutral_pos + turn_vector * 0.8
                    trajectory = self.generate_step_trajectory(start_pos, end_pos, True, swing_points)
                    self.get_logger().debug(f'Turn leg {leg} SWING: vector={turn_vector}')
                else:
                    start_pos = neutral_pos + turn_vector * 0.8
                    end_pos = neutral_pos - turn_vector * 0.8
                    trajectory = self.generate_step_trajectory(start_pos, end_pos, False, stance_points)
                
                leg_trajectories[leg] = trajectory
            
            # Execute trajectories synchronously
            max_points = max(len(leg_trajectories[leg]) for leg in range(1, 7))
            
            for step_idx in range(max_points):
                if not self.is_moving:
                    return
                
                for leg in range(1, 7):
                    if step_idx < len(leg_trajectories[leg]):
                        target_pos = leg_trajectories[leg][step_idx]
                        joint_angles = self.inverse_kinematics(target_pos, leg)
                        
                        if joint_angles is not None:
                            self.send_joint_command(leg, joint_angles, step_time)
                
                time.sleep(step_time)
            
            phase = 1 - phase

    def send_joint_command(self, leg_num, joint_angles, duration_sec):
        """Send joint command to specific leg."""
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

    def reset_to_initial_pose(self):
        """Reset robot to initial standing pose."""
        self.get_logger().info('Resetting to initial pose...')
        self.is_moving = False
        
        duration = Duration()
        duration.sec = 2
        duration.nanosec = 0
        
        for leg_num in range(1, 7):
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names[leg_num]
            
            point = JointTrajectoryPoint()
            point.positions = list(map(float, self.initial_angles[leg_num]))
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = duration
            
            traj_msg.points.append(point)
            self.trajectory_publishers[leg_num].publish(traj_msg)
        
        time.sleep(2.0)

    def emergency_stop(self):
        """Stop all movement immediately."""
        self.get_logger().warn('EMERGENCY STOP!')
        self.is_moving = False
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1.0)

    def get_key(self):
        """Non-blocking keyboard input."""
        if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def process_key(self, key):
        """Process keyboard commands."""
        if key is None:
            return True

        key = key.lower()
        
        # Stop any current movement
        if self.is_moving:
            self.emergency_stop()
            time.sleep(0.2)

        if key == 'w':  # Forward
            self.get_logger().info('Walking forward...')
            direction = np.array([self.step_length, 0, 0])
            self.movement_thread = threading.Thread(target=self.execute_spider_gait, args=(direction,))
            self.movement_thread.start()
            
        elif key == 's':  # Backward  
            self.get_logger().info('Walking backward...')
            direction = np.array([-self.step_length, 0, 0])
            self.movement_thread = threading.Thread(target=self.execute_spider_gait, args=(direction,))
            self.movement_thread.start()
            
        elif key == 'a':  # Turn left
            self.get_logger().info('Turning left...')
            self.movement_thread = threading.Thread(target=self.execute_turn_gait, args=(-1.0,))
            self.movement_thread.start()
            
        elif key == 'd':  # Turn right
            self.get_logger().info('Turning right...')
            self.movement_thread = threading.Thread(target=self.execute_turn_gait, args=(1.0,))
            self.movement_thread.start()
            
        elif key == 'q':  # Forward-left diagonal
            self.get_logger().info('Walking forward-left...')
            direction = np.array([self.step_length*0.7, self.step_length*0.7, 0])
            self.movement_thread = threading.Thread(target=self.execute_spider_gait, args=(direction,))
            self.movement_thread.start()
            
        elif key == 'e':  # Forward-right diagonal
            self.get_logger().info('Walking forward-right...')
            direction = np.array([self.step_length*0.7, -self.step_length*0.7, 0])
            self.movement_thread = threading.Thread(target=self.execute_spider_gait, args=(direction,))
            self.movement_thread.start()
            
        elif key == 'x':  # Stop and reset
            self.get_logger().info('Stopping movement...')
            self.emergency_stop()
            
        elif key == 'r':  # Reset pose
            self.get_logger().info('Resetting pose...')
            self.emergency_stop()
            threading.Thread(target=self.reset_to_initial_pose).start()
            
        elif key == ' ':  # Emergency stop
            self.emergency_stop()
            
        elif key == '\x1b' or ord(key) == 3:  # ESC or Ctrl+C
            self.get_logger().info('Exiting...')
            return False
            
        else:
            if ord(key) > 31:  # Printable character
                self.get_logger().info(f'Unknown command: "{key}" - See instructions above')

        return True

    def run(self):
        """Main control loop."""
        try:
            # Initialize to standing pose
            self.reset_to_initial_pose()
            
            self.get_logger().info('Spider gait controller ready! Use WASD for movement.')
            
            while rclpy.ok():
                key = self.get_key()
                if not self.process_key(key):
                    break
                    
                time.sleep(0.01)  # Small delay to prevent CPU overload
                
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received')
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources."""
        self.emergency_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
        self.get_logger().info('Hexapod controller shutdown complete')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop = HexapodSpiderTeleop()
        teleop.run()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()