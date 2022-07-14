#!/usr/bin/env python

import rospy
import cv2 
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError
from skimage import io, exposure, img_as_uint, img_as_float
import imageio
import ros_numpy

class LineFollower(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.camera_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)
        self.point_cloud = rospy.Subscriber("/camera/depth/color/points",PointCloud2,self.point_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="rgb8")
            matplotlib.image.imsave("random.png",cv_image)
        except CvBridgeError as e:
            print(e)

    def depth_callback(self,data):
        try:
            depth_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="32FC1")
            imageio.imwrite("random_depth.png",depth_image.astype(np.uint16))
        except CvBridgeError as e:
            print(e)

    def point_callback(self,data):
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        x = np.array(xyz_array['x'])
        y = np.array(xyz_array['y'])
        z = np.array(xyz_array['z'])

        d = np.stack((x,y,z),axis=-1)
        
        # print(d.shape)
        np.save("possible_xyz_array",d)

        print("####")

        # r = np.array(xyz_array['r'])
        # g = np.array(xyz_array['g'])
        # b = np.array(xyz_array['b'])

        # e = np.stack((r,g,b),axis=-1)

        # rgb = ros_numpy.point_cloud2.split_rgb_field(xyz_array)
        # r = np.array(rgb['r'])
        # g = np.array(rgb['g'])
        # b = np.array(rgb['b'])

        # e = np.stack((r,g,b),axis=-1)
        # print(e.shape)
        # np.save("possible_rgb_array",e)


def main():
    line_follower_object = LineFollower()
    rospy.init_node('line_following_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
