# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from time import sleep
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import sys
sys.path.append("/home/sohn/anaconda3/lib/python3.7/")
sys.path.append("/home/sohn/anaconda3/lib/python3.7/site-packages/")
sys.path.append('/home/sohn/anaconda3/lib/python37.zip')
sys.path.append("/home/sohn/anaconda3/lib/python3.7")
sys.path.append("/home/sohn/anaconda3/lib/python3.7/lib-dynload")
import cv2

class RGBDSubscriber(Node):

    def __init__(self):
        super().__init__('realsense_rgbd_subscriber')
        print("__INIT__")

        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw')
        self.d_sub = message_filters.Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw')
        self.rgb_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')
        self.d_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/aligned_depth_to_color/camera_info')

    def listener_callback(self, msg_rgb, msg_d):
        logger = self.get_logger()
        self.get_logger().info("[Get RGB, Depth Images]")
        bridge = CvBridge()
        cv_rgb = bridge.imgmsg_to_cv2(msg_rgb, desired_encoding='passthrough')
        cv_depth = bridge.imgmsg_to_cv2(msg_d, desired_encoding='passthrough')
        cv_depth = cv_depth[..., np.newaxis]
        #logger.info(f"rgb.shape : {cv_rgb.shape}")
        #logger.info(f"depth.shape : {cv_depth.shape}")
        #logger.info(f"np.min(depth) : {np.min(cv_depth)}")
        #logger.info(f"np.max(depth) : {np.max(cv_depth)}")

        # preporcessing
        cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
        cv_depth = np.array(cv_depth, dtype=np.uint8)
        #cv_depth = cv2.equalizeHist(cv_depth)
        cv2.normalize(cv_depth, cv_depth, 0, 255, cv2.NORM_MINMAX)

        # stack for printing on one frame
        cv_depth_3channel = cv2.cvtColor(cv_depth, cv2.COLOR_GRAY2BGR)
        numpy_horizontal_concat = np.concatenate(
            (cv_rgb, cv_depth_3channel), axis=1)
        cv2.imshow('RGB, Depth Image', numpy_horizontal_concat)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    rgbd_subscriber = RGBDSubscriber()

    #ts = message_filters.TimeSynchronizer([rgbd_subscriber.rgb_sub, rgbd_subscriber.d_sub], qos_profile_sensor_data)
    #ts = message_filters.TimeSynchronizer([rgbd_subscriber.rgb_sub, rgbd_subscriber.d_sub], qos_profile_sensor_data)
    #ts = message_filters.ApproximateTimeSynchronizer([rgbd_subscriber.rgb_sub, rgbd_subscriber.d_sub], qos_profile_sensor_data, 0.1, allow_headerless=True)
    
    ts = message_filters.ApproximateTimeSynchronizer(
        [rgbd_subscriber.rgb_sub, rgbd_subscriber.d_sub], 10, 0.001, allow_headerless=True)
    ts.registerCallback(rgbd_subscriber.listener_callback)

    rclpy.spin(rgbd_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rgbd_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
