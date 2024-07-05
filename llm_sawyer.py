#!/usr/bin/env python3

import rospy
import intera_interface
from intera_interface import Limb, RobotEnable
from intera_core_msgs.msg import EndpointState
import json
from groq import Groq
from termcolor import colored
import os

# Groq client
client = Groq(
    api_key=os.getenv("GROQ_API_KEY"),
)
MODEL = "llama3-70b-8192"

# groq API Key: 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

#export GROQ_API_KEY='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'


# Global variables
arm = None
arm_pose = None
arm_updated = False

def enable_robot():
    robot_state = RobotEnable()
    robot_state.enable()

# Callback function for the arm pose subscriber
def arm_callback(data):
    global arm_pose
    global arm_updated

    arm_pose = data
    arm_updated = True

def joint_angles_to_dict(joint_angles):
    return joint_angles

def move_to_neutral():
    """
    Move the arm to a neutral pose.
    """
    print("Moving to neutral pose...")
    arm.move_to_neutral()

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

def perform_complex_motion(smoothness=150):
    """
    Perform a complex motion by sequencing predefined movements.
    """
    movement_1 = {'right_j6': -1.5126, 'right_j5': -0.3438, 'right_j4': 1.5126, 'right_j3': -1.3833, 'right_j2': 0.03726, 'right_j1': 0.3526, 'right_j0': -0.4259}
    movement_2 = {'right_j6': -1.5101, 'right_j5': -0.3806, 'right_j4': 1.5103, 'right_j3': -1.4038, 'right_j2': -0.2609, 'right_j1': 0.3940, 'right_j0': -0.4281}
    movement_3 = {'right_j6': -1.5, 'right_j5': -0.5, 'right_j4': 1.5, 'right_j3': -1.4, 'right_j2': 0.0, 'right_j1': 0.3, 'right_j0': -0.4}
    movement_4 = {'right_j6': -1.0, 'right_j5': -0.4, 'right_j4': 1.0, 'right_j3': -1.0, 'right_j2': -0.5, 'right_j1': 0.2, 'right_j0': -0.3}
    movement_5 = {'right_j6': 3.42, 'right_j5': 1.02, 'right_j4': 0.67, 'right_j3': -0.22, 'right_j2': -0.84, 'right_j1': 0.43, 'right_j0': -0.09}

    movements_sequence = [movement_1, movement_2, movement_3, movement_4, movement_5]

    for i in range(len(movements_sequence) - 1):
        current_move = movements_sequence[i]
        next_move = movements_sequence[i + 1]
        smooth_motion = interpolate_movement(current_move, next_move)

        for smooth_step in smooth_motion:
            arm.move_to_joint_positions(smooth_step)
            rospy.sleep(0.01)  # Adjust sleep duration for smoother motion

        rospy.sleep(0.5)

def run_conversation(user_prompt):
    messages = [
        {
            "role": "user",
            "content": user_prompt,
        },
    ]
    tools = [
        {
            "type": "function",
            "function": {
                "name": "move_arm_to_joint_positions",
                "description": "Move the Sawyer arm to specific joint positions. Returns the joint angles of the arm after the movement.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "joint_positions": {
                            "type": "object",
                            "description": "Dictionary of joint angles for the Sawyer arm",
                        },
                        "duration": {
                            "type": "number",
                            "description": "The duration of the movement in seconds",
                        },
                    },
                    "required": ["joint_positions", "duration"],
                },
            },
        }
    ]

    response = client.chat.completions.create(
        model=MODEL, 
        messages=messages, 
        tools=tools, 
        tool_choice="auto", 
        max_tokens=4096
    )

    response_message = response.choices[0].message
    tool_calls = response_message.tool_calls

    while tool_calls:
        available_functions = {
            "move_arm_to_joint_positions": move_arm_to_joint_positions,
        }

        rospy.loginfo(
            colored(f"Response message: {response_message}", "green")
        )
        messages.append(response_message)  # extend conversation with assistant's reply

        for tool_call in tool_calls:
            function_name = tool_call.function.name
            function_to_call = available_functions[function_name]
            function_args = json.loads(tool_call.function.arguments)
            rospy.loginfo(
                colored(
                    f"Calling function {function_name} with arguments {function_args}",
                    "yellow",
                )
            )

            function_response = function_to_call(
                joint_positions=function_args["joint_positions"],
                duration=function_args["duration"],
            )

            rospy.loginfo(colored(f"Function response: {function_response}", "yellow"))

            messages.append(
                {
                    "tool_call_id": tool_call.id,
                    "role": "tool",
                    "name": function_name,
                    "content": function_response,
                }
            )  # extend conversation with function response
        second_response = client.chat.completions.create(
            model=MODEL, 
            messages=messages
        )  # get a new response from the model where it can see the function response
        response_message = second_response.choices[0].message
        rospy.loginfo(
            colored(f"Response message: {response_message}", "green")
        )
        tool_calls = response_message.tool_calls

    return response_message.content

def move_arm_to_joint_positions(joint_positions, duration):
    global arm_updated

    arm.set_joint_position_speed(0.3)
    arm.move_to_joint_positions(joint_positions)

    while not arm_updated:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    arm_updated = False  # Reset the flag

    joint_angles_dict = joint_angles_to_dict(arm.joint_angles())

    return json.dumps(joint_angles_dict)

def main():
    global arm

    rospy.init_node("robot_llm")

    enable_robot()
    arm = Limb('right')

    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, arm_callback)

    while arm_pose is None:
        rospy.sleep(0.1)  # Sleep for a short time to avoid busy waiting

    joint_angles = arm.joint_angles()
    print("Initial joint angles: {}".format(joint_angles))

    while not rospy.is_shutdown():
        user_prompt = input("> Enter prompt: ")
        response = run_conversation(user_prompt)
        print(colored(f"LLM message: {response}", "cyan"))

    # rospy.spin()

if __name__ == "__main__":
    main()

