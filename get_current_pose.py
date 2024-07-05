import rospy
import intera_interface
from intera_interface import Limb

# Initialize the ROS node and limb interface
rospy.init_node('Get_Sawyer_Joint_Angles')
limb = Limb('right')

def get_current_joint_angles():
    """
    Get the current joint angles of the right arm.
    """
    joint_angles = limb.joint_angles()
    return joint_angles

# Main program
if __name__ == '__main__':
    try:
        current_joint_angles = get_current_joint_angles()
        print("Current Joint Angles:")
        print(current_joint_angles)
    except rospy.ROSInterruptException:
        pass
