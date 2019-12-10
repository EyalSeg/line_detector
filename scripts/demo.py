 #!/usr/bin/env python

import rospy

import tf2_ros

from line_detector.srv import NextPositionInLineService
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist
from tf2_geometry_msgs import PoseStamped


rospy.init_node('DEMO_NODE')

print("waiting for services")
rospy.wait_for_service('line_detection_service')
print("got line service")
rospy.wait_for_service('gazebo/set_model_state')
print("got gazebo service")

line_srv = rospy.ServiceProxy('line_detection_service', NextPositionInLineService)
teleport_srv = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

print("setting tf buffer")
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

print("submitting request")
result = line_srv(1.0, "ltr")
print("got it!")
print(result.next_position)

print("converting to map frame")

# Assuming the world is static, you could use this fix. Which will force the coordinates transformation to use the last data which there is on the frame, without respect to the time period.
result.next_position.header.stamp = rospy.Time(0)

input_pose = PoseStamped(result.next_position.header, result.next_position.pose)
converted_pose = tfBuffer.transform(input_pose, 'map').pose
converted_pose.position.z = 0 # ground the position

# A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,angle_temp[2]))

print(converted_pose)

twist = Twist()
model_name = "armadillo2"

desired_state = ModelState(model_name, converted_pose, twist, 'map')
print("teleporting! That probably messes with the transform tree...")
teleport_srv(desired_state)

pass


