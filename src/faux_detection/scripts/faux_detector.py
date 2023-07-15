#!/usr/bin/env python
"""
Try to detect objects in the image stream
Relevant links:
https://docs.microsoft.com/en-us/azure/cognitive-services/custom-vision-service/export-model-python
https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/auto_examples/object_detection_camera.html
https://docs.luxonis.com/en/latest/pages/training/
"""
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
import numpy as np
import rospy
# from PIL import Image
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import threading
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D

def faux_detector():
    # Base params
    min_score_thresh = rospy.get_param('~min_score_thresh')
    interval = rospy.get_param('~interval')
    detection_topic = rospy.get_param('~detection_topic')
    model_type = rospy.get_param('~model_type')

    # Conditionally import model
    if model_type == "graphdef":
        from graphdef import Detector
        model_file = rospy.get_param('~model_file')
        label_file = rospy.get_param('~label_file')
        detector = Detector(model_file, label_file, min_score_thresh, interval, detection_topic)
    else:
        import os
        # Memory allocation
        os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "garbage_collection_threshold:0.5,max_split_size_mb:32"

        from torchhub import Detector
        model_path = rospy.get_param('~model_path')
        weights_file = rospy.get_param('~weights_file')
        subprocess_file = rospy.get_param('~subprocess_file')
        subprocess_python_file = rospy.get_param('~subprocess_python_file')
        detector = Detector(
            model_path, weights_file, subprocess_file, subprocess_python_file,
            min_score_thresh, interval, detection_topic
        )
    
    # Setup subscriber callback
    rospy.Subscriber(rospy.get_param('~image_topic'), Image, detector.image_callback)
    rospy.on_shutdown(detector.destructor)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    detector.running_detection()

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('faux_detector', anonymous=True)

    # Run faux camera
    faux_detector()