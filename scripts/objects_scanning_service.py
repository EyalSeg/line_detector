#!/usr/bin/env python

import rospy

from line_detector.srv import ObjectDetection, ObjectDetectionResponse

from cv_bridge import CvBridge
import httplib
import json
import sys
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2

import cv2


pointcloud_topic = '/kinect2/qhd/points'
mrcnn_server_uri='localhost:8000'

camera_transform_name= "kinect2_depth_frame"
world_transform_name="map"

service_name = "object_scanning_service"
node_name= "object_scanning_service"


class objects_scanner:
    def __init__(self, mrcnn_server_uri):
        self.cv_bridge = CvBridge()
        self.mrcnn_server_uri = mrcnn_server_uri

    def msg_to_image(self, image_msg, encoding="bgr8"):
        return self.cv_bridge.imgmsg_to_cv2(image_msg, encoding)

    def detect(self, pc2_message, desired_classes=[]):
        image = self.pointcloud2_to_rgb(pc2_message)
        masks = self.find_objects_in_image(image, desired_classes)
        positions = [self.get_object_position(pc2_message, mask) for mask in masks]        

        return positions

    def find_objects_in_image(self, image, desired_classes = []):
        body_data = {"image_array": image.tolist(), "classes":desired_classes}
        json_data = json.dumps(body_data)

        conn = httplib.HTTPConnection(self.mrcnn_server_uri, timeout=sys.maxint)
        conn.request("POST", "/", body=json_data)
        res = conn.getresponse()
        response_data = json.loads(res.read().decode())

        masks_array = np.array(response_data['masks'])
        masks = [masks_array[:, :, i] for i in range(response_data['count'])]

        return masks

    def pointcloud2_to_rgb(self, pc):
        x = np.frombuffer(pc.data, 'uint8').reshape(-1, 8, 4)
        bgr = x[:pc.height*pc.width, 4, :3].reshape(pc.height, pc.width, 3)
        rgb = bgr[:,:,::-1]

        return rgb

    def get_object_position(self, cloud, object_mask):
        points = self.mask_to_points(cloud, object_mask)
        points = self.normalize_coordinates(points)

        # first axis is col axis -> left-right axis -> y axis
        # also ROS expects this axis to be where -inf is at the right, so flip the sign of the coordinate
        y = np.mean(points[:, 0])
        y = -y

        # second axis is row axis -> up-down axis -> z axis
        z = np.mean(points[:, 1])

        # third axis is depth axis -> forward-backward axis -> x axis
        x = np.mean(points[:, 2])

        point = [x, y, z]
        return point

    def mask_to_points(self, cloud, mask):
        pixles = np.argwhere(mask).tolist()
        pixles = [[v, u] for [u, v] in pixles] # the pixles are row-col but pc expects them as col-row
        generator = pc2.read_points(cloud, field_names=['x', 'y', 'z'], uvs=pixles, skip_nans=True)

        points = np.array(list(generator))
        return points

    def normalize_coordinates(self, coordinates):
        # remove (0,0,0)
        coordinates = coordinates[np.any(coordinates != 0, axis=1)]

        coordinates = self.normalize_coordinates_by_axis(coordinates, 0)
        coordinates = self.normalize_coordinates_by_axis(coordinates, 1)
        coordinates = self.normalize_coordinates_by_axis(coordinates, 2)

        return coordinates

    def normalize_coordinates_by_axis(self, coordinates, axis, m=1.8):
        mean = np.mean(coordinates[:,axis])
        sd = np.std(coordinates[:,axis])

        return coordinates[abs(coordinates[:, axis] - mean) < m * sd]


def detection_server():
    rospy.init_node(node_name, anonymous=True)
    service = rospy.Service(service_name, ObjectDetection, handle_detection_request)
    print("Ready to scan for objects!")

    rospy.spin()


def handle_detection_request(request):
    classes = request.classes_to_find
    scanner = objects_scanner(mrcnn_server_uri)

    pc2_msg = rospy.wait_for_message(pointcloud_topic, PointCloud2)

    # This might result in an error trying to transform the points later, if the detection takes too long.
    # Consider moving this line until _after_ the detection. 
    # However, it might retrun erronous results if anything in the scene moved during detection time - which should happen in real-world scenario
    msg_time = rospy.Time.now()

    coordinates_local = scanner.detect(pc2_msg)
    return genrerate_respone(coordinates_local, msg_time)


def genrerate_respone(coordinates, time):

    response = ObjectDetectionResponse()

    response.header.stamp = time
    response.header.frame_id = camera_transform_name

    response.object_coordinates = [Point(coordinate[0], coordinate[1], coordinate[2]) for coordinate in coordinates]

    return response


if __name__ == "__main__":
    detection_server()



                                                                                                                                                                                                                                                        