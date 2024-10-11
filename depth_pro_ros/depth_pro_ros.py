#!/usr/bin/env python

# Copyright (c) 2024 Daniel Eneh <danieleneh024@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Python libraries
import time
import numpy as np
import torch
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'depth_pro', 'src'))
print ("success")



# ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image

# Depth Pro library
from depth_pro.depth_pro import DEFAULT_MONODEPTH_CONFIG_DICT, create_model_and_transforms, DepthProConfig,DepthPro
print("success")

class DepthProRos(Node):
    """DepthProROS node

    This node subscribes to an image topic and publishes a depth image [in meters] and focal length.

    Parameters
    ----------
    image_topic : str
        Topic where the image will be subscribed.
    depth_image_topic : str
        Topic where the raw depth image will be published.
    device : str
        Device to use for the inference (cpu or cuda).
    checkpoint_file : str
        Path to the model.

    Subscribers
    ----------
    image_topic : sensor_msgs.msg.Image
        Image topic where the RGB image will be subscribed.

    Publishers
    ----------
    depth : sensor_msgs.msg.Image
        Image topic where the depth image will be published.

    Methods
    -------
    __init__(self)
        Initializes the node.
    get_params(self)
        Gets the ROS2 parameters.
    image_callback(self, image_msg: Image)
        Callback function for the image topic.
    """
    
    def __init__(self):
        """Initializes the DepthProROS node."""
        super().__init__('depth_pro_ros')
        self.get_params()
        self.bridge = CvBridge()

        if self.device != 'cpu':
            if not torch.cuda.is_available():
                self.get_logger().info(f'Device could not be set to: [{self.device}] ...')
                self.device = "cpu"
        self.get_logger().info(f'Setting device to: [{self.device}]')

        # Dynamically construct the path to the checkpoint file
        checkpoint_path = os.path.join(os.path.dirname(__file__), '..', 'checkpoints', 'depth_pro.pt')
        print (checkpoint_path)

        # Use the default configuration and update the checkpoint_uri dynamically
        custom_config = DEFAULT_MONODEPTH_CONFIG_DICT
        custom_config.checkpoint_uri = checkpoint_path

        # Load Depth Pro model and transforms
        self.model, self.transforms = create_model_and_transforms(
            config=custom_config,  # Use default config and updated checkpoint path
            device=self.device,
            precision=torch.float32
        )

        self.model.eval()

         # Create common publishers
        sensor_qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # Publisher for the depth images
        self.depth_pub = self.create_publisher(Image, self.depth_image_topic, sensor_qos_profile)

        # Subscriber to the image topic
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, sensor_qos_profile)

        self.get_logger().info('DepthProROS node has been initialized')

    def get_params(self):
        """Gets the ROS2 parameters."""
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('depth_image_topic', '/depth_image')
        self.declare_parameter('device', 'cpu')
        #self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('checkpoint_file', 'depth_pro.pt')

        self.image_topic = self.get_parameter('image_topic').value
        self.depth_image_topic = self.get_parameter('depth_image_topic').value
        self.device = torch.device(self.get_parameter('device').value)
        self.checkpoint_file = self.get_parameter('checkpoint_file').value

    def image_callback(self, image_msg: Image):
        """Callback function for the image topic.

        Parameters
        ----------
        image_msg : sensor_msgs.msg.Image
            Image message.
        """
        try:
            # Converts the ROS Image message to OpenCV formatiing
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

            # Converts the image to PyTorch tensor and apply transformations
            input_tensor = self.transforms(cv_image).unsqueeze(0)

            # depth estimation using DepthPro Inference
            with torch.no_grad():
                output = self.model.infer(input_tensor)

            # Extracts the depth map from the output
            depth_image = output["depth"].cpu().numpy()

            # Converts depth map to a ROS Image message
            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, '32FC1')
            depth_image_msg.header = image_msg.header

            # Publishes the depth image
            self.depth_pub.publish(depth_image_msg)

            # Logs out the focal length
            focal_length = output["focallength_px"]
            self.get_logger().info(f'Focal length: {focal_length}')

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    depth_pro_ros = DepthProRos()
    rclpy.spin(depth_pro_ros)
    depth_pro_ros.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()   

