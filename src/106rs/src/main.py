#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena


MODIFIED For EECS106A Project F'19 Group 22
"""

from __future__ import print_function
from collections import deque

import time

import rospy
import message_filters
import ros_numpy
import tf

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped

import numpy as np
import cv2

from cv_bridge import CvBridge

from image_segmentation import segment_image
from pointcloud_segmentation import segment_pointcloud
from pointcloud_segmentation import project_points

from sklearn.decomposition import PCA

from scipy.spatial.transform import Rotation as R


def get_camera_matrix(camera_info_msg):
    # TODO: Return the camera intrinsic matrix as a 3x3 numpy array
    # by retreiving information from the CameraInfo ROS message.
    # Hint: numpy.reshape may be useful here.
    return np.array(camera_info_msg.K).reshape(3, 3)

def isolate_object_of_interest(points, image, cam_matrix, trans, rot):
    segmented_image, found_box = segment_image(image)
    if not found_box:
        return None, found_box
    points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
    return points, found_box

def numpy_to_pc2_msg(points):
    return ros_numpy.msgify(PointCloud2, points, stamp=rospy.Time.now(),
        frame_id='camera_depth_optical_frame')

# Rodriguez code from https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
def rotation_matrix_from_vectors(vec1, vec2):
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    
    return rotation_matrix

class PointcloudProcess:
    """
    Wraps the processing of a pointcloud from an input ros topic and publishing
    to another PointCloud2 topic.

    """
    def __init__(self, points_sub_topic, 
                       image_sub_topic,
                       cam_info_topic,
                       points_pub_topic):

        self.num_steps = 0

        self.messages = deque([], 5)
        self.pointcloud_frame = None
        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        self.points_pub = rospy.Publisher(points_pub_topic, PointCloud2, queue_size=10)
        self.pose_pub = rospy.Publisher("stick_pose", PoseStamped, queue_size=10)
        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=10)

        self.br = tf.TransformBroadcaster()
        
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, points_msg, image, info):
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            points = ros_numpy.numpify(points_msg)
        except Exception as e:
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft((points, rgb_image, intrinsic_matrix))

    def publish_once_from_queue(self):
        if self.messages:
            points, image, info = self.messages.pop()
            try:
                trans, rot = self.listener.lookupTransform(
                                                       '/camera_color_optical_frame',
                                                       '/camera_depth_optical_frame',
                                                       rospy.Time(0))
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException):
                return

            points, found_box = isolate_object_of_interest(points, image, info, 
                np.array(trans), np.array(rot))
            if not found_box:
                return

            points_msg = numpy_to_pc2_msg(points)

            new_points = np.empty((len(points),3))
            for i, point in enumerate(points):
                new_points[i][0] = point[0]
                new_points[i][1] = point[1]
                new_points[i][2] = point[2]

            try:
                assert len(new_points) > 0

                pca = PCA(n_components=1)
                pca.fit(new_points)

                first_PC = pca.components_[0]

                pose_msg = PoseStamped()
                x, y, z = np.mean(new_points, axis=0)
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z

                # Create transform from x-axis in depth optical frame to stick frame
                x_axis = np.array([1, 0, 0])
                mat = rotation_matrix_from_vectors(x_axis, first_PC)
                vec1_rot = mat.dot(x_axis)

                # Sanity check R
                assert np.allclose(vec1_rot/np.linalg.norm(vec1_rot), first_PC/np.linalg.norm(first_PC))

                r = R.from_dcm(mat)
                x_theta, y_theta, z_theta, w = r.as_quat()

                pose_msg.pose.orientation.x = x_theta
                pose_msg.pose.orientation.y = y_theta
                pose_msg.pose.orientation.z = z_theta
                pose_msg.pose.orientation.w = w
                pose_msg.header.frame_id = '/camera_depth_optical_frame'
            except Exception as e:
                print(e)
                print("error")

                return


            self.br.sendTransform(
                (x, y, z),
                (x_theta, y_theta, z_theta, w),
                rospy.Time.now(),  
                "/stick_frame",
                "/camera_depth_optical_frame"
            )
            
            self.points_pub.publish(points_msg)
            self.pose_pub.publish(pose_msg)

def main():
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    POINTS_TOPIC = '/camera/depth/color/points'
    POINTS_PUB_TOPIC = 'segmented_points'

    rospy.init_node('realsense_listener')
    process = PointcloudProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':
    main()
