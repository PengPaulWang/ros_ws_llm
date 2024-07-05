import rospy
import intera_interface
from intera_interface import Limb
import time
import numpy as np

# Initialize the ROS node and limb interface
rospy.init_node('Sawyer_Multiple_Movements')
limb = Limb('right')

def move_to_neutral():
    """
    Move the arm to a neutral pose.
    """
    print("Moving to neutral pose...")
    limb.move_to_neutral()

def interpolate_movement(start_movement, end_movement):
    """
    Interpolate between two movements to create a smooth motion.
    
    :param start_movement: Starting joint angles for the movement
    :param end_movement: Ending joint angles for the movement
    :return: List of joint angles for the smooth motion
    """
    smooth_motion = []
    steps = 50  # Number of interpolation steps
    for i in range(steps):
        alpha = float(i) / (steps - 1)  # Interpolation factor
        smooth_step = {}
        for joint, start_angle in start_movement.items():
            end_angle = end_movement[joint]
            interpolated_angle = start_angle + alpha * (end_angle - start_angle)
            smooth_step[joint] = interpolated_angle
        smooth_motion.append(smooth_step)
    return smooth_motion

def interpolate_wave(wave_start, wave_end, steps):
    """
    Interpolate between two wave positions to create a smooth wave motion.
    
    :param wave_start: Starting wave joint angles
    :param wave_end: Ending wave joint angles
    :param steps: Number of interpolation steps
    :return: List of joint angles for the smooth wave motion
    """
    smooth_wave = []
    for i in range(steps):
        alpha = float(i) / (steps - 1)  # Interpolation factor
        smooth_step = {}
        for joint, start_angle in wave_start.items():
            end_angle = wave_end[joint]
            interpolated_angle = start_angle + alpha * (end_angle - start_angle)
            smooth_step[joint] = interpolated_angle
        smooth_wave.append(smooth_step)
    return smooth_wave

def perform_complex_motion(smoothness=150):
    """
    Perform a complex motion by sequencing predefined movements.
    """
    movement_1 = {'right_j6': -1.5126, 'right_j5': -0.3438, 'right_j4': 1.5126, 'right_j3': -1.3833, 'right_j2': 0.03726, 'right_j1': 0.3526, 'right_j0': -0.4259}
    movement_2 = {'right_j6': -1.5101, 'right_j5': -0.3806, 'right_j4': 1.5103, 'right_j3': -1.4038, 'right_j2': -0.2609, 'right_j1': 0.3940, 'right_j0': -0.4281}
    
    # Define additional movements
    movement_3 = {'right_j6': -1.5, 'right_j5': -0.5, 'right_j4': 1.5, 'right_j3': -1.4, 'right_j2': 0.0, 'right_j1': 0.3, 'right_j0': -0.4}
    movement_4 = {'right_j6': -1.0, 'right_j5': -0.4, 'right_j4': 1.0, 'right_j3': -1.0, 'right_j2': -0.5, 'right_j1': 0.2, 'right_j0': -0.3}
    
    movement_5 = {'right_j6': 3.42, 'right_j5': 1.02, 'right_j4': 0.67, 'right_j3': -0.22, 'right_j2': -0.84, 'right_j1': 0.43, 'right_j0': -0.09}
    # movement_5 = {'right_j0': -0.09, 'right_j1': 0.43, 'right_j2': -0.84, 'right_j3': -0.22, 'right_j4': 0.67, 'right_j5': 1.02, 'right_j6': 3.42}

    # Perform a sequence of movements
    movements_sequence = [movement_1, movement_2, movement_3, movement_4, movement_5]
    
    for i in range(5-1):
        # Interpolate between movements for smoother motion
        current_move = movements_sequence[i]
        next_move = movements_sequence[i+1]


        # smooth_motion = interpolate_movement(limb.joint_angles(), movement)
        smooth_motion = interpolate_wave(current_move, next_move, smoothness)
        # smooth_motion = interpolate_movement()
        
        for smooth_step in smooth_motion:
            limb.move_to_joint_positions(smooth_step)
            print(f'current move is {smooth_step}')
            rospy.sleep(0.01)  # Adjust sleep duration for smoother motion

        rospy.sleep(0.5)

# Main program
if __name__ == '__main__':
    try:
        # move_to_neutral()
        # perform_complex_motion()
        move_to_neutral()
    except rospy.ROSInterruptException:
        pass

