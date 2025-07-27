#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import argparse

# --- Parse argument for full rotation angle ---
parser = argparse.ArgumentParser(description='Rotation for hexapod')
parser.add_argument('--angle', type=float, default=90.0,
                    help='Full rotation angle in degrees')
args = parser.parse_args()

def turn_hexapod(radius, angle, x_start, z):
    x_new = (x_start + radius) * np.cos(angle) - radius
    y_new = (x_start + radius) * np.sin(angle)
    return np.array([x_new, y_new, z])

def rectangular_trajectory(start, goal, height, num_points):
    num_points += 3
    start_up = start + np.array([0, 0, height])
    goal_up = goal + np.array([0, 0, height])

    phase1 = np.linspace(start, start_up, num_points // 3)
    phase2 = np.linspace(start_up, goal_up, num_points // 3)
    phase3 = np.linspace(goal_up, goal, num_points - len(phase1) - len(phase2))

    points = np.concatenate([phase1[1:], phase2[1:], phase3[1:]], axis=0)
    return points

def servo_angles(point, l1, l2, l3):
    """
    Calculates servo angles (inverse kinematics) for hexapod leg.
    """
    alpha1 = np.arctan2(point[1], point[0])
    P1 = np.array([l1 * np.cos(alpha1), l1 * np.sin(alpha1), 0])
    distance = np.linalg.norm(point - P1)

    cos_fi = (l2**2 + l3**2 - distance**2) / (2 * l2 * l3)
    cos_fi = np.clip(cos_fi, -1.0, 1.0)  # Safety clipping
    fi = np.arccos(cos_fi)
    alpha3 = np.deg2rad(180) - fi

    epsilon = np.arcsin(np.sin(fi) * l3 / distance)
    tau = np.arctan2(point[2] - P1[2], np.linalg.norm(point[:2] - P1[:2]))

    alpha2 = -(epsilon + tau)
    return [alpha1, alpha2, alpha3]

# Leg segment lengths (meters)
l1 = 0.17995 - 0.12184
l2 = 0.30075 - 0.17995
l3 = 0.50975 - 0.30075

# Initial joint angles in radians (adjust if needed)
alpha1_init = 0.0
alpha2_init = np.deg2rad(10)
alpha3_init = np.deg2rad(80)

correction_factor = 1

total_angle = np.radians(args.angle)

joint_offset_from_center = 0.1218
rotation_angle_cycle = np.radians(20)
rotation_angle = rotation_angle_cycle / 2 * correction_factor

# Rotation direction
direction = np.sign(total_angle)
rotation_angle *= direction

# Starting foot position in leg coordinate frame
x_start = l1 + l2 * np.cos(alpha2_init) + l3 * np.sin(np.deg2rad(90) - alpha2_init - alpha3_init)
z_start = -(l2 * np.sin(alpha2_init) + l3 * np.cos(np.deg2rad(90) - alpha2_init - alpha3_init))

step_height = 0.1
points_per_phase = 30

start_point = np.array([x_start, 0, z_start])
point_P1 = turn_hexapod(joint_offset_from_center, rotation_angle, x_start, z_start)
point_P2 = turn_hexapod(joint_offset_from_center, -rotation_angle, x_start, z_start)

# Peak points of the step
peak_phase1 = (start_point + point_P1) / 2 + np.array([0, 0, step_height])
peak_phase5 = (start_point + point_P2) / 2 + np.array([0, 0, step_height])

# Generate trajectories for phases
phase_1 = rectangular_trajectory(start_point, point_P1, step_height, points_per_phase)
phase_2 = np.linspace(point_P1, start_point, points_per_phase)[1:]
phase_3 = np.linspace(start_point, point_P2, points_per_phase)[1:]
phase_5 = rectangular_trajectory(point_P2, start_point, step_height, points_per_phase)

num_cycles = int(np.abs(total_angle) // rotation_angle_cycle)
remaining_angle = (np.abs(total_angle) % rotation_angle_cycle) / 2 * correction_factor * direction

cycle_legs_1_3_5 = np.concatenate([phase_1, phase_2])
cycle_legs_2_4_6 = np.concatenate([phase_3, phase_5])

for _ in range(num_cycles - 1):
    cycle_legs_1_3_5 = np.concatenate([cycle_legs_1_3_5, phase_1, phase_2])
    cycle_legs_2_4_6 = np.concatenate([cycle_legs_2_4_6, phase_3, phase_5])

if np.abs(remaining_angle) > np.radians(1):
    point_P1 = turn_hexapod(joint_offset_from_center, remaining_angle, x_start, z_start)
    point_P2 = turn_hexapod(joint_offset_from_center, -remaining_angle, x_start, z_start)
    phase_1 = rectangular_trajectory(start_point, point_P1, step_height, points_per_phase)
    phase_2 = np.linspace(point_P1, start_point, points_per_phase)[1:]
    phase_3 = np.linspace(start_point, point_P2, points_per_phase)[1:]
    phase_5 = rectangular_trajectory(point_P2, start_point, step_height, points_per_phase)
    cycle_legs_1_3_5 = np.concatenate([cycle_legs_1_3_5, phase_1, phase_2])
    cycle_legs_2_4_6 = np.concatenate([cycle_legs_2_4_6, phase_3, phase_5])

# Calculate servo angles for the trajectories
servo_angles_legs_1_3_5 = np.array([servo_angles(p, l1, l2, l3) for p in cycle_legs_1_3_5])
servo_angles_legs_2_4_6 = np.array([servo_angles(p, l1, l2, l3) for p in cycle_legs_2_4_6])


class LegSequencePlayer(Node):
    def __init__(self):
        super().__init__('leg_sequence_player')
        self.get_logger().info('Initializing leg sequence player...')

        # Publishers for each leg controller
        self.trajectory_publishers = {
            1: self.create_publisher(JointTrajectory, '/leg1_controller/joint_trajectory', 10),
            2: self.create_publisher(JointTrajectory, '/leg2_controller/joint_trajectory', 10),
            3: self.create_publisher(JointTrajectory, '/leg3_controller/joint_trajectory', 10),
            4: self.create_publisher(JointTrajectory, '/leg4_controller/joint_trajectory', 10),
            5: self.create_publisher(JointTrajectory, '/leg5_controller/joint_trajectory', 10),
            6: self.create_publisher(JointTrajectory, '/leg6_controller/joint_trajectory', 10),
        }

        # Joint names matching your URDF
        self.joint_names = {
            1: ['coxa_joint_1', 'femur_joint_1', 'tibia_joint_1'],
            2: ['coxa_joint_2', 'femur_joint_2', 'tibia_joint_2'],
            3: ['coxa_joint_3', 'femur_joint_3', 'tibia_joint_3'],
            4: ['coxa_joint_4', 'femur_joint_4', 'tibia_joint_4'],
            5: ['coxa_joint_5', 'femur_joint_5', 'tibia_joint_5'],
            6: ['coxa_joint_6', 'femur_joint_6', 'tibia_joint_6'],
        }

    def send_trajectory_to_all_legs_at_step(self, step_index, duration_sec=0.2):
        if step_index >= len(servo_angles_legs_1_3_5) or step_index >= len(servo_angles_legs_2_4_6):
            self.get_logger().error(f'Step index {step_index} out of range')
            return False

        duration = Duration()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        for leg_num in range(1, 7):
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names[leg_num]
            point = JointTrajectoryPoint()

            if leg_num in [1, 3, 5]:
                joint_positions = servo_angles_legs_1_3_5[step_index]
            else:
                joint_positions = servo_angles_legs_2_4_6[step_index]

            point.positions = list(map(float, joint_positions))
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            point.time_from_start = duration

            traj_msg.points.append(point)
            self.trajectory_publishers[leg_num].publish(traj_msg)

        self.get_logger().info(f'Published trajectories for step {step_index}')
        return True

    def execute_sequence(self, start_step=0, end_step=None, step_duration=0.2):
        self.get_logger().info('Starting leg movement sequence...')
        max_steps = min(len(servo_angles_legs_1_3_5), len(servo_angles_legs_2_4_6))

        if end_step is None or end_step > max_steps:
            end_step = max_steps

        # Move to initial position
        self.send_trajectory_to_all_legs_at_step(start_step, duration_sec=step_duration)
        time.sleep(step_duration)

        for step in range(start_step + 1, end_step):
            self.send_trajectory_to_all_legs_at_step(step, duration_sec=step_duration)
            time.sleep(step_duration)

        self.get_logger().info('Leg movement sequence completed.')

def main(args=None):
    rclpy.init(args=args)
    node = LegSequencePlayer()

    try:
        node.execute_sequence()
        time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
