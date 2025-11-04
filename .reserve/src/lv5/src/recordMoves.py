#!/usr/bin/env python3
import rospy
import gazebo_msgs.msg as gazebo
import sensor_msgs.msg as sensors
import std_msgs.msg as signals

rospy.init_node("ros_node")

pointCoud = list()

lastSampling = rospy.Time.now().to_sec()
samplingTime = 0.1

shouldDraw = False

def getPositions(msgs: gazebo.LinkStates):
    global lastSampling
    global samplingTime
    global shouldDraw
    if(lastSampling + samplingTime > rospy.Time.now().to_sec() or not shouldDraw):
        return
    lastSampling = rospy.Time.now().to_sec()
    global pointCoud
    position = msgs.pose[-1].position
    pointCoud.append(position)
    msg = sensors.PointCloud()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.points = pointCoud
    display.publish(msg)

def setDrawStatus(msg: signals.Bool):
    global shouldDraw
    shouldDraw = msg.data

sensor = rospy.Subscriber('/gazebo/link_states', gazebo.LinkStates, getPositions)
drawStatus = rospy.Subscriber('/pen_state', signals.Bool, setDrawStatus)

display = rospy.Publisher('/sensor_msgs/PointCloud', sensors.PointCloud, queue_size=10)

rospy.spin()