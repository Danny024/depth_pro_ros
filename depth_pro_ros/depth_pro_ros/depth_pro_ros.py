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


#Python libraries
import time
import numpy as np
import torch
import cv2
from cv_bridge import CvBridge, CvBridgeError


#ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image


#Depth Pro library
import depth_pro


class DepthProRos(Node):
     """DepthProROS node

    This node subscribes to an image topic and publishes a depth image [in m ]and focal length

    Parameters
    ----------
        image_topic : str
            Topic where the image will be subscribed.
        depth_image_topic : str
            Topic where the raw depth image will be published.
        device : str
            Device to use for the inference (cpu or cuda).
        model_file : str
            Path to the model.
        

    Subscribers
    ----------
        image_topic : sensor_msgs.msg.Image
            Image topic where the rgb image will be subscribed.

    Publishers
    ----------
        depth : sensor_msgs.msg.Image
            Image topic where the depth image will be published.

    Methods
    -------
        __init__(self)
            Initializes the node.
        get_params(self)
            Gets the ros2 parameters.
        image_callback(self, image_msg: Image)
            Callback function for the image topic.
    """
