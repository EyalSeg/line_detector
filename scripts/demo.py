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
input_pose = PoseStamped(result.next_position.header, result.next_position.pose)
converted_pose = tfBuffer.transform(input_pose, 'map').pose
converted_pose.position.z = 0 # ground the position

print(converted_pose)

twist = Twist()
model_name = "armadillo2"

desired_state = ModelState(model_name, converted_pose, twist, 'map')
print("teleporting! That probably messes with the transform tree...")
teleport_srv(desired_state)

pass


