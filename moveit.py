#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from math import pi
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# Define the initial position of the robot
INITIAL_POSITION = {
    'x': 1.18425,
    'y': 0.290699999951,
    'z': 0.0608499999404,
    'orientation': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'w': 1.0
    }
}

# Define the fabric position
FABRIC_POSITION = {
    'x': 0.5,
    'y': 0.5,
    'z': 0.2,
}

# Define the move_group globally
move_group = None
hand_command_pub = None

def move_to_target(target_pose):
    global move_group

    # Initialize MoveGroupCommander only if it hasn't been initialized
    if move_group is None:
        move_group = MoveGroupCommander("manipulator")

    # Set the target pose for the robot arm
    moved_pose = Pose()
    moved_pose.position.x = target_pose['x']
    moved_pose.position.y = target_pose['y']
    moved_pose.position.z = target_pose['z']

    # Set the orientation to look straight down
    quaternion = quaternion_from_euler(0.0, pi, 0.0)  # Roll, Pitch, Yaw
    moved_pose.orientation.x = quaternion[0]
    moved_pose.orientation.y = quaternion[1]
    moved_pose.orientation.z = quaternion[2]
    moved_pose.orientation.w = quaternion[3]

    # Set the target pose and execute the movement
    move_group.set_pose_target(moved_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def move_home():
    global move_group

    # Initialize MoveGroupCommander only if it hasn't been initialized
    if move_group is None:
        move_group = MoveGroupCommander("manipulator")

    group_name = move_group.get_name()
    rospy.loginfo("Moving {} to home position...".format(group_name))

    # Set the home position
    pose_goal = Pose()
    pose_goal.position.x = INITIAL_POSITION['x']
    pose_goal.position.y = INITIAL_POSITION['y']
    pose_goal.position.z = INITIAL_POSITION['z']
    pose_goal.orientation.x = INITIAL_POSITION['orientation']['x']
    pose_goal.orientation.y = INITIAL_POSITION['orientation']['y']
    pose_goal.orientation.z = INITIAL_POSITION['orientation']['z']
    pose_goal.orientation.w = INITIAL_POSITION['orientation']['w']

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if plan:
        rospy.loginfo("Home position reached successfully for {}!".format(group_name))
    else:
        rospy.logerr("Failed to reach home position for {}.".format(group_name))

def move_to_fabric():
    global move_group

    rospy.loginfo("Moving manipulator to fabric position...")

    # Step 1: Move above the fabric (0.5 meters higher)
    above_fabric_position = FABRIC_POSITION.copy()
    above_fabric_position['z'] += 0.5  # Move up by 0.5 meters
    move_to_target(above_fabric_position)

    # Step 2: Move down to the fabric position
    move_to_target(FABRIC_POSITION)

    # Step 3: Close the gripper to grab the fabric
    hand_command_pub.publish("close")
    rospy.sleep(2)

    # Step 4: Move half a meter forward while maintaining the same orientation
    new_pose = FABRIC_POSITION.copy()
    new_pose['x'] += 0.5  # Move forward by 0.5 meters
    move_to_target(new_pose)

    # Wait for 2 seconds
    rospy.sleep(2)

    # Step 5: Open the gripper to release the fabric
    hand_command_pub.publish("open")
    rospy.sleep(2)

    # Step 6: Return to the home position without rotation
    move_home()

def callback_fn(msg):
    global move_group, hand_command_pub
    if msg.data == "home":
        move_home()
    elif msg.data == "danai":
        move_to_fabric()

def main():
    global move_group, hand_command_pub

    # Initialize MoveIt and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_tutorial", anonymous=True)

    hand_command_pub = rospy.Publisher('/hand_command', String, queue_size=10)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Initialize the MoveGroupCommander for the manipulator
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    print("============ Printing robot state")
    print(robot.get_current_state())
    print(move_group.get_current_pose().pose)

    # Subscribe to the move_command topic
    rospy.Subscriber("/move_commands", String, callback_fn)

    move_home()  # Move to home position initially

    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    main()





moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit_tutorial", anonymous=True)

hand_command_pub = rospy.Publisher('/hand_command', String, queue_size=10)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander("manipulator")

# move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

# pub = rospy.Subscriber('/arm_commands', String, callback_fn)

print("============ Printing robot state")
print(robot.get_current_state())
print(move_group.get_current_pose().pose)
