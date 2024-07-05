import rospy
import intera_interface
from intera_interface import Limb
import time
import numpy as np

# Initialize the ROS node and limb interface
rospy.init_node('Sawyer_Smooth_Waves')
limb = Limb('right')

def move_to_neutral():
    """
    Move the arm to a neutral pose.
    """
    print("Moving to neutral pose...")
    limb.move_to_neutral()

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

def wave_smoothly(times=3, smoothness=50):
    """
    Make the robot smoothly wave its arm.
    
    :param times: Number of times to wave (default: 3)
    :param smoothness: Number of interpolation steps for smooth waves (default: 50)
    """
    wave_1 = {'right_j6': -1.5126, 'right_j5': -0.3438, 'right_j4': 1.5126, 'right_j3': -1.3833, 'right_j2': 0.03726, 'right_j1': 0.3526, 'right_j0': -0.4259}
    wave_2 = {'right_j6': -1.5101, 'right_j5': -0.3806, 'right_j4': 1.5103, 'right_j3': -1.4038, 'right_j2': -0.2609, 'right_j1': 0.3940, 'right_j0': -0.4281}
    
    for _move in range(times):
        # Interpolate between wave_1 and wave_2 for smoother motion
        smooth_waves = interpolate_wave(wave_1, wave_2, smoothness)
        
        for smooth_step in smooth_waves:
            limb.move_to_joint_positions(smooth_step)
            rospy.sleep(0.01)  # Adjust sleep duration for smoother motion

        rospy.sleep(0.5)

# Main program
if __name__ == '__main__':
    try:
        move_to_neutral()
        wave_smoothly(times=3, smoothness=50)
        move_to_neutral()
    except rospy.ROSInterruptException:
        pass
