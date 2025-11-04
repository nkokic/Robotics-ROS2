#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import std_msgs.msg as signals
import copy
from math import pi
rospy.init_node("move_rob_manip")
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
penStatePub = rospy.Publisher('/pen_state', signals.Bool, queue_size=10)
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
pose_goal = Pose()

def GoToPose(position, orientation = [0.0, 3.14, 0.0]):
    pose_goal.position.x = position[0]
    pose_goal.position.y = position[1]
    pose_goal.position.z = position[2]
    q = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    move_group.set_pose_target(pose_goal, end_effector_link='hand')
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

penStatePub.publish(False)
GoToPose([0.3, 0.0, 0.2])
GoToPose([0.3, -0.1, 0.2])

penStatePub.publish(True)

GoToPose([0.3, 0.1, 0.2])
GoToPose([0.3, 0.1, 0.4])
GoToPose([0.3, -0.1, 0.4])
GoToPose([0.3, -0.1, 0.2])

penStatePub.publish(False)

GoToPose([0.35, -0.05, 0.35])

penStatePub.publish(True)

GoToPose([0.35, -0.05, 0.25])
GoToPose([0.35, 0.05, 0.25])
GoToPose([0.35, 0.05, 0.35])
GoToPose([0.35, -0.05, 0.35])

penStatePub.publish(False)

GoToPose([0.4, 0, 0.3])

penStatePub.publish(True)

GoToPose([0.1, 0, 0.3])
penStatePub.publish(False)
