#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import httplib
import json
import sys
import numpy as np

from sensor_msgs.msg import PointCloud2, Image
from tf2_geometry_msgs import PointStamped
import sensor_msgs.point_cloud2 as pc2

import tf2_ros

import cv2


pointcloud_topic = '/kinect2/qhd/points'
mrcnn_server_uri='localhost:8000'

camera_transform_name= "kinect2_depth_optical_frame"
world_transform_name="map"



class objects_scanner:
    def __init__(self, mrcnn_server_uri):
        self.cv_bridge = CvBridge()
        self.mrcnn_server_uri = mrcnn_server_uri

    def msg_to_image(self, image_msg, encoding="bgr8"):
        return self.cv_bridge.imgmsg_to_cv2(image_msg, encoding)

    def detect(self, pc2_message):
        image = self.pointcloud2_to_rgb(pc2_message)
        masks = self.find_objects_in_image(image)
        positions = [self.get_object_position(pc2_message, mask) for mask in masks]        

        return positions

    def find_objects_in_image(self, image):
        body_data = {"image_array": image.tolist(), "classes":['person']}
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

        x = np.mean(points[:, 0])
        y = np.mean(points[:, 1])
        z = np.mean(points[:, 2])

        point = [x, y, z]

        return point

    def mask_to_points(self, cloud, mask):
        pixles = np.argwhere(mask).tolist()
        pixles = [[v, u] for [u, v] in pixles] # the pixles are row-col but pc expects them as col-row for some reason
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


def init_node():
    rospy.init_node("object_scanner", anonymous=True)
    rate = rospy.Rate(10)

def transform_point(buffer, point, time):
    stamped = PointStamped()
    stamped.header.stamp = time
    stamped.header.frame_id = camera_transform_name

    stamped.point.x = point[0]
    stamped.point.y = point[1]
    stamped.point.z = point[2]

    new_point = buffer.transform(stamped, world_transform_name)
    return new_point.point

if __name__ == "__main__":
    init_node()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    scanner = objects_scanner(mrcnn_server_uri)

    pc2_msg = rospy.wait_for_message(pointcloud_topic, PointCloud2)
    msg_time = rospy.Time.now()
    coordiantes_local = scanner.detect(pc2_msg)
    coordiantes_global = [transform_point(tfBuffer, point, msg_time) for point in coordiantes_local]

    a = 5


                                                                                                                                                                                                                                                        