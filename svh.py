#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header, Duration
from std_msgs.msg import String
from std_msgs.msg import Float64
import time


def close_hand():
    msg = FollowJointTrajectoryActionGoal()

    msg.goal.trajectory.joint_names = ['left_hand_Thumb_Opposition', 'left_hand_Thumb_Flexion', 'left_hand_Index_Finger_Proximal',
                                       'left_hand_Index_Finger_Distal', 'left_hand_Middle_Finger_Proximal', 'left_hand_Middle_Finger_Distal',
                                       'left_hand_Finger_Spread', 'left_hand_Pinky', 'left_hand_Ring_Finger']

    point = JointTrajectoryPoint()
    point.positions = [0.17, 0.93, 0.51, 0.80, 0.52, 0.36, 0.48, 0.54, 0.00]
    point.time_from_start = rospy.Duration(1)
    msg.goal.trajectory.points.append(point)
    pub.publish(msg)

    # point = JointTrajectoryPoint()
    # point.positions = [0.17, 0.93, 0.51, 0.80, 0.52, 0.36, 0.48, 0.54, 0.00]
    # point.velocities = [1, 1, 1, 1, 1, 1, 1, 1, 1]
    # point.accelerations = [1, 1, 1, 1, 1, 1, 1, 1, 1]
    # point.time_from_start = rospy.Duration(1)
    # msg.goal.trajectory.points.append(point)
    # pub.publish(msg)


def open_hand():
    msg = FollowJointTrajectoryActionGoal()

    msg.goal.trajectory.joint_names = ['left_hand_Thumb_Opposition', 'left_hand_Thumb_Flexion', 'left_hand_Index_Finger_Proximal',
                                       'left_hand_Index_Finger_Distal', 'left_hand_Middle_Finger_Proximal', 'left_hand_Middle_Finger_Distal',
                                       'left_hand_Finger_Spread', 'left_hand_Pinky', 'left_hand_Ring_Finger']

    point = JointTrajectoryPoint()
    point.positions = [-0.002, 0.0, -0.0002, -0.00025, -0.0003, -0.00025, -0.000004, -0.00047, -0.00077]
    point.time_from_start = rospy.Duration(1)
    msg.goal.trajectory.points.append(point)
    pub.publish(msg)



    # point = JointTrajectoryPoint()
    # point.positions = [-0.0022615329147619434, 1.556748161490873e-07, -0.00028867791577713575, -0.00025045624190855875,
    #                    -0.0003054057541946875, -0.0002573493614148603, -4.258850263560987e-06, -0.00047161395614292445,
    #                    -0.0007744146365586957]
    # point.velocities = [1, 1, 1, 1, 1, 1, 1, 1, 1]
    # point.accelerations = [1, 1, 1, 1, 1, 1, 1, 1, 1]
    # point.time_from_start = rospy.Duration(1)
    # msg.goal.trajectory.points.append(point)
    # pub.publish(msg)

    # point.positions = [0.0] * len(msg.goal.trajectory.joint_names) # Ανοιγμα χεριού
    # point.time_from_start = rospy.Duration(1)

def move_fingers():
    msg = FollowJointTrajectoryActionGoal()

    msg.goal.trajectory.joint_names = ['left_hand_Thumb_Opposition', 'left_hand_Thumb_Flexion', 'left_hand_Index_Finger_Proximal',
                                       'left_hand_Index_Finger_Distal', 'left_hand_Middle_Finger_Proximal', 'left_hand_Middle_Finger_Distal',
                                       'left_hand_Finger_Spread', 'left_hand_Pinky', 'left_hand_Ring_Finger']

    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.12, 0.74, 1.33, 0.0, 1.0, 0.12, 0.09, 0.02]
    point.velocities = [1, 1, 1, 1, 1, 1, 1, 1, 1]
    point.accelerations = [1, 1, 1, 1, 1, 1, 1, 1, 1]
    point.time_from_start = rospy.Duration(1)
    msg.goal.trajectory.points.append(point)
    pub.publish(msg)


# def hand_commands_callback(command):
#     if command.data == 'close':
#         print("now closing")
#         close_hand()
#     if command.data == 'open':
#         open_hand()
#     if command.data == 'move':
#         print("now close only the thumb and the index finger")
#         move_fingers()

def hand_commands_callback(command):
        if command.data == 'close':
            rospy.loginfo("Closing hand")
            close_hand()
        if command.data == 'open':
           rospy.loginfo("Opening hand")
           open_hand()
        # if command.data == 'move':
        #     rospy.loginfo("Moving fingers")
        #     move_fingers()






  # point.positions = [0.3, 0.8, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
  # point.velocities = [1, 1, 1, 1, 1, 1, 1, 1, 1]

# point.positions = [0.5, 1, 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0]
#  point.velocities = [1, 1, 1, 1, 1, 1, 1, 1, 1]

# def move_fingers(thumb_position, index_position):
#     rospy.init_node('move_fingers_node', anonymous=True)
#     rospy.Subscriber('hand_command', String, self.command_callback)
#
#     self.feedback_pub = rospy.Publisher('hand_feedback', String, queue_size=10 )
#
#     self.thumb_position = 0.0
#     self.index_position = 0.0
#
#     # thumb_pub = rospy.Publisher('/schunk_svh/thumb_position_controller/command', Float64, queue_size=10)
#     # index_pub = rospy.Publisher('/schunk_svh/index_position_controller/command', Float64, queue_size=10)
#
#     rate = rospy.Rate(10)
#     rospy.sleep(1)
#
#     # thumb_touch_position = 0.5
#     # index_touch_position = 0.5
#     # thumb_pub.publish(Float64(thumb_position))
#     # index_pub.publish(Float64(index_position))
#     #
#     # rospy.loginfo("Published thumb position: thumb_touch_position")
#     # rospy.loginfo("Published index position: index_touch_position")


# def closeup(self):
#     self.thumb_position = "40.0"
#     self.index_position = "40.0"
#     self.publish_positions()
#
#      # rospy.loginfo("O antixeiras kai o deikths enothikan")
#      # self.feedback_pub.publish("fingers_closed")
#
# def openup(self):
#     self.thumb_position = 0.0
#     self.index_position = 0.0
#     self.publish_positions()


rospy.init_node('trajectory_publisher', anonymous=True)
pub = rospy.Publisher('/left_hand_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
sub = rospy.Subscriber('/hand_command', String, hand_commands_callback)
hand_command_pub = rospy.Publisher('/hand_command', String, queue_size=10)



time.sleep(2)

rospy.spin()
