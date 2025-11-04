#!/usr/bin/env python3 
import rospy 
import moveit_commander 
from geometry_msgs.msg import Pose 
from tf.transformations import quaternion_from_euler 
import std_msgs.msg as signals
import copy 
rospy.init_node("ros_node") 
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
GoToPose([0.3, 0.15, 0.4])

penStatePub.publish(True)
waypoints = []
wpose = move_group.get_current_pose().pose 
wpose.position.y -= 0.3
waypoints.append(copy.deepcopy(wpose))
wpose.position.z -= 0.3
waypoints.append(copy.deepcopy(wpose))
wpose.position.y += 0.3
waypoints.append(copy.deepcopy(wpose))
wpose.position.z += 0.3
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001)
move_group.execute(plan, wait=True)
move_group.stop() 
move_group.clear_pose_targets()

penStatePub.publish(False)
GoToPose([0.4, 0.075, 0.325])

penStatePub.publish(True)
waypoints = []
wpose = move_group.get_current_pose().pose 
wpose.position.y -= 0.15
waypoints.append(copy.deepcopy(wpose))
wpose.position.z -= 0.15
waypoints.append(copy.deepcopy(wpose))
wpose.position.y += 0.15
waypoints.append(copy.deepcopy(wpose))
wpose.position.z += 0.15
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001)
move_group.execute(plan, wait=True)
move_group.stop() 
move_group.clear_pose_targets()

penStatePub.publish(False)
GoToPose([0.5, 0, 0.25])

penStatePub.publish(True)
waypoints = []
wpose = move_group.get_current_pose().pose 
wpose.position.x -= 0.3
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.001)
move_group.execute(plan, wait=True)
move_group.stop() 
move_group.clear_pose_targets()

penStatePub.publish(False)