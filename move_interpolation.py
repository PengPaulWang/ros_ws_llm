import rospy
import intera_interface
from intera_interface import Limb
import numpy as np

# Initialize the ROS node and limb interface
rospy.init_node('Sawyer_Smooth_Movement')
limb = Limb('right')

def move_to_neutral():
    """
    Move the arm to a neutral pose.
    """
    print("Moving to neutral pose...")
    limb.move_to_neutral()

def interpolate_movement(start_movement, end_movement, steps):
    """
    Interpolate between two movements to create a smooth motion.
    
    :param start_movement: Starting joint angles for the movement
    :param end_movement: Ending joint angles for the movement
    :param steps: Number of interpolation steps
    :return: List of joint angles for the smooth motion
    """
    smooth_motion = []
    for i in range(steps):
        alpha = float(i) / (steps - 1)  # Interpolation factor
        smooth_step = {}
        for joint, start_angle in start_movement.items():
            end_angle = end_movement[joint]
            interpolated_angle = start_angle + alpha * (end_angle - start_angle)
            smooth_step[joint] = interpolated_angle
        smooth_motion.append(smooth_step)
    return smooth_motion

def perform_smooth_movement(start_movement, end_movement, smoothness=50):
    """
    Perform a smooth movement by interpolating joint angles between start and end positions.
    
    :param start_movement: Starting joint angles
    :param end_movement: Ending joint angles
    :param smoothness: Number of interpolation steps for smooth motion (default: 50)
    """
    # Interpolate between start and end movements for smoother motion
    smooth_motions = interpolate_movement(start_movement, end_movement, smoothness)
    
    for smooth_step in smooth_motions:
        limb.move_to_joint_positions(smooth_step)
        rospy.sleep(0.01)  # Adjust sleep duration for smoother motion

# Define the joint angles for movement_4 and movement_5
movement_4 = {'right_j6': -1.0, 'right_j5': -0.4, 'right_j4': 1.0, 'right_j3': -1.0, 'right_j2': -0.5, 'right_j1': 0.2, 'right_j0': -0.3}
movement_5 = {'right_j6': 3.42, 'right_j5': 1.02, 'right_j4': 0.67, 'right_j3': -0.22, 'right_j2': -0.84, 'right_j1': 0.43, 'right_j0': -0.09}

# Main program
if __name__ == '__main__':
    try:
        move_to_neutral()
        perform_smooth_movement(movement_4, movement_5)
        move_to_neutral()
    except rospy.ROSInterruptException:
        pass
