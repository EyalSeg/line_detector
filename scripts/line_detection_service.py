#!/usr/bin/env python

import rospy

import tf2_ros
import numpy as np
import sys
import tf.transformations
import math

from scipy import optimize

from line_detector.srv import ObjectDetection
from geometry_msgs.msg import Point, PointStamped, Quaternion, Pose, PoseStamped

from line_detector.srv import NextPositionInLineService, NextPositionInLineServiceResponse


from math import copysign


service_name = 'line_detection_service'
node_name = 'line_detection_service'
object_scanning_service_name="object_scanning_service"
line_classes = ['person']


loss_magic_num = 4
max_line_threshold = 5
max_polynomial_degree = 3


def find_next_position_in_line(people_coordinates, distance, direction='ltr'):
    # note that this function ignores the z coordinate. 
    # assumption - the point's frame is parallel to the ground (we inted to convert it to the world frame and drop z to 0 later)
    points = stem_and_sort_line(people_coordinates)
    last_point = points[-1]
    last_yx = [last_point.y, last_point.x]
    weights = generate_weights(len(points))

    xs = [point.x for point in points]
    ys = [point.y for point in points]
    #zs = [point.x for point in points]

    # note that x is the front-distance, so we need p(y) = x
    line_polynomial = curve_fit(ys, xs, max_polynomial_degree, weights)
    next_y = find_next_position_in_polynomial(line_polynomial, last_yx, distance, direction)
    next_x = line_polynomial(next_y)
    next_z = last_point.z # or any other value, due to the assumption. Otherwise, we might have to curve fit the same way as for x.

    new_point = Point(next_x, next_y, next_z)
    orientation = get_rotation_between_points(new_point, last_point)

    new_pose = Pose(new_point, orientation)

    return new_pose

def get_rotation_between_points(origin, target):
    dx = target.x - origin.x
    dy = target.y - origin.y

    angle = math.atan2(dy, dx)# * 180 / np.pi
    #angle = -angle # right hand rule...

    q_array = tf.transformations.quaternion_from_euler(0, 0, angle)
    q = Quaternion(*q_array)

    return q
    

# find a u such that [u, poly(u)] is given-distance away from given-position 
# used to find a position in the polynomial ~distance away from the given position
# given position - [a, b] s.t p(a) ~= b
def find_next_position_in_polynomial(polynomial, position, distance, direction='ltr'):
    # calculates the distance between a and the given position
    norm_func = lambda a : np.linalg.norm(np.array([a, polynomial(a)]) - position)

    # a root of this function is distance away from the given position
    desired_distance_func = lambda a : norm_func(a) - distance

    # define where we would like our next position to intersect the polynomial
    # note that the higher the y value, the more the point is to the left
    if direction == "ltr":
        #window = get_bisection_window(desired_distance_func, position[0], -0.1)
        window = (position[0] - (distance * 2), position[0] + 0.01)
    else:
       # window = get_bisection_window(desired_distance_func, position[0], 0.1)
        window = (position[0] - 0.01, position[0] + distance * 2)

    # find a root for the above function. use any method you prefer.
    result_u = optimize.brentq(desired_distance_func, *window)

    return result_u

# returns [x1, x2] s.t either x1 or x2 equals x0 and sgn(f(x1)) != sgn(f(x2)) 
def get_bisection_window(func_to_solve, x0, step=0.1):
    x1 = x0

    y0 = func_to_solve(x0)
    y1 = func_to_solve(x1)         

    while copysign(1, y0) == copysign(1, y1):
        x1 = x1 + step

        y1 = func_to_solve(x1)         

    return [x0, x1] if step > 0 else [x1, x0]
    
# finds the best curve to fit the points (p(u) ~= v)
# prefers lower degree polynomials.
def curve_fit(u_axis, v_axis, max_degree = 3, weights = None):
    # the polynomial should estimate the distance in-front of the robot for a given distance to the size
    # therefore p(y) ~= x

    # I would suggest scipi's linear-regression, however I do not think we have enough datapoints

    best_loss = sys.maxsize
    best_curve = None

    for degree in range(1, max_degree + 1):
        # BUG HERE: residual is zero. maybe problem with the weights?
        curve, [resid, rank, sv, rcond] = \
            np.polynomial.polynomial.Polynomial.fit(u_axis, v_axis, degree, w=weights, full=True)

        loss = 0 if len(resid) == 0 else sum(resid)
        loss = loss * (loss_magic_num ** degree) # punishes higher-order polynomials

        if loss < best_loss:
            best_loss = loss
            best_curve = curve

    return best_curve

def generate_weights(num_of_weights):
    # dimish the weight of a point the further away it is from the line's ending
    weights = [1.0 / i for i in reversed(range(1, num_of_weights + 1))]

    # "encourage" the line to pass through the last point
    weights[-1] = num_of_weights

    return weights

def stem_and_sort_line(people_coordinates, dir="ltr"):
    # note that y=inf is at the left
    reversed = dir == 'ltr'

    # sort left-to-right: x - forward, y - left, z - up
    result = sorted(people_coordinates, key=lambda point: point.y, reverse=reversed)
    
    # if the line has a lot of people, ignore everyone but the last few
    if len(people_coordinates) > max_line_threshold:
        result = result[-max_line_threshold:]
    
    return result

def find_people():
    rospy.wait_for_service(object_scanning_service_name)
    object_detection_service = rospy.ServiceProxy(object_scanning_service_name, ObjectDetection)

    detection_results = object_detection_service(line_classes)

    coords = detection_results.object_coordinates
    
    return coords, detection_results.header

def init_server():
    rospy.init_node(node_name)
    service = rospy.Service(service_name, NextPositionInLineService, handle_request)
    print("Ready to find line positions!")

    rospy.spin()

    
def handle_request(request):
    people, header = find_people()
    new_pose = find_next_position_in_line(people, request.distance, direction=request.line_direction) 

    result = PoseStamped(header, new_pose)
    response = NextPositionInLineServiceResponse()
    response.next_position = result

    return response

if __name__ == "__main__":
    init_server()


