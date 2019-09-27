 #!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
import sys

from scipy import optimize

from line_detector.srv import ObjectDetection
from tf2_geometry_msgs import PointStamped


node_name = 'line_detection_service'
object_scanning_service_name="object_scanning_service"
world_frame = 'map'
line_classes = ['person']


distance_from_last_person = 0.5
loss_magic_num = 4
max_line_threshold = 5
max_polynomial_degree = 3


def find_next_position_in_line(people_coordinates, direction='ltr'):
    # note that this function ignores the z coordinate. 
    # assumption - the point's frame is parallel to the ground (we inted to convert it to the world frame and drop z to 0 later)
    points = stem_and_sort_line(people_coordinates)
    last_point = points[-1]
    last_point = [last_point.y, last_point.x]
    weights = generate_weights(len(points))

    xs = [point.x for point in points]
    ys = [point.y for point in points]
    #zs = [point.x for point in points]

    # note that x is the front-distance, so we need p(y) = x
    line_polynomial = curve_fit(ys, xs, max_polynomial_degree, weights)
    next_y = find_next_position_in_polynomial(line_polynomial, last_point, distance_from_last_person, direction)
    next_x = line_polynomial(next_y)
    next_z = last_point.z # or any other value, due to the assumption. Otherwise, we might have to curve fit the same way as for x.

    return Point(next_x, next_y, next_z)

# find a u such that [u, poly(u)] is given-distance away from given-position 
# given position - [a, b] s.t p(a) ~= b
def find_next_position_in_polynomial(polynomial, position, distance, direction='ltr'):
    # calculates the distance between a and the given position
    norm_func = lambda a : np.linalg.norm(np.array([a, polynomial(a)]) - position)

    # a root of this function is distance away from the given position
    desired_distance_func = lambda a : norm_func(a) - distance

    # define where we would like our next position to intersect the polynomial
    # note that the higher the y value, the more the point is to the left
    if direction == "ltr":
        window = (position[0] - 0.01, position[0] + distance * 2)
    else:
        window = (position[0] - (distance * 2), position[0] + 0.01)

    # find a root for the above function. use any method you prefer.
    result_u = optimize.bisect(desired_distance_func, *window)

    return result_u
    
# finds the best curve to fit the points (p(u) ~= v)
# prefers lower degree polynomials.
def curve_fit(u_axis, v_axis, max_degree = 3, weights = None):
    # the polynomial should estimate the distance in-front of the robot for a given distance to the size
    # therefore p(y) ~= x

    # I would suggest scipi's linear-regression, however I do not think we have enough datapoints

    best_loss = sys.maxsize
    best_curve = None

    for degree in range(1, max_degree + 1):
        curve, [resid, rank, sv, rcond] = \
            np.polynomial.polynomial.Polynomial.fit(u_axis, v_axis, degree, w=weights, full=True)

        loss = 0 if len(resid) == 0 else resid * (loss_magic_num ** degree) # punishes higher-order polynomials

        if loss < best_loss:
            best_loss = loss
            best_curve = curve

    return best_curve

def generate_weights(num_of_weights):
    # dimish the weight of a point the further away it is from the line's ending
    weights = [1 / i for i in reversed(range(1, num_of_weights + 1))]

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

pub = rospy.Publisher('clicked_point', PointStamped, queue_size=10)
rospy.init_node(node_name)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)


people, header = find_people()

next_point = find_next_position_in_line(people, direction='ltr') 
next_stamped = PointStamped(next_point, header)

result = tfBuffer.transform(next_stamped, world_frame)
result.z = 0


print('people!')
