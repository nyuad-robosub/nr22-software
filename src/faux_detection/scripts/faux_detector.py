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
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
import rospy
# from PIL import Image
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import tensorflow as tf
import threading
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D

# Helper functions courtesy of:
# https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/Easy_Object_Detection_With_Custom_Data_Demo_Training.ipynb
# Getting number of classes
def get_num_classes(pbtxt_fname):
    from object_detection.utils import label_map_util
    label_map = label_map_util.load_labelmap(pbtxt_fname)
    categories = label_map_util.convert_label_map_to_categories(
        label_map, max_num_classes=90, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)
    return len(category_index.keys())

# Create detector courtesy of:
# https://docs.microsoft.com/en-us/azure/cognitive-services/custom-vision-service/export-model-python
class Detector():
    def __init__(self, model_file, label_file, min_score_thresh, interval, detection_topic):
        """ Initialize detector
        :param model_file: Path to model
        :param label_file: Path to label file
        :param min_score_thresh: How high score is considered acceptable
        :param interval: How much time (in milliseconds) to wait between inferences
        """
        # Import the TF graph
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            self.graph_def = tf.compat.v1.GraphDef()
            with tf.io.gfile.GFile(model_file, 'rb') as f:
                self.graph_def.ParseFromString(f.read())
                tf.import_graph_def(self.graph_def, name='')
        
        # Create a map of labels
        self.label_map = label_map_util.load_labelmap(label_file)
        self.num_classes = get_num_classes(label_file)
        categories = label_map_util.convert_label_map_to_categories(
            self.label_map, max_num_classes=self.num_classes, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        # Image feed
        self.br = CvBridge()
        self.new_frame = False
        self.image = Image()
        self.image_lock = threading.Lock()

        self.interval = interval
        self.last_frame_time = rospy.Time.now()
        self.frame_counter = 0
        self.terminate = False

        # Detection threshold, publisher & msg
        self.min_score_thresh = min_score_thresh
        self.detection_pub = rospy.Publisher(detection_topic, Detection2DArray, queue_size=5)
        self.detection_msg = Detection2DArray()
        
    def drop_frames_callback(self, drop_frames):
        """ Callback for number of frames to drop
        :param drop_frames: Int16
        """
        self.drop_frames = drop_frames.data

    def image_callback(self, image):
        """ Callback to receive images
        :param image: sensor_msgs.msg.Image, camera stream
        """
        # self.frame_counter += 1
        current_time = rospy.Time.now()
        diff = current_time - self.last_frame_time
        if diff > rospy.Duration(nsecs=self.interval * 1e6):
            # self.frame_counter = 0
            self.image_lock.acquire()
            self.image = image
            self.new_frame = True
            self.last_frame_time = current_time
            self.image_lock.release()
        
    def publish_detection(self, output_dict, image):
        """ Function to publish detections
        :param image: sensor_msgs.msg.Image, image message used to do detections on
        """
        image_height = image.height
        image_width = image.width

        # Forming message
        self.detection_msg.header.stamp = image.header.stamp
        self.detection_msg.detections = []
        for i in range(len(output_dict['detection_classes'])):
            if output_dict['detection_scores'][i] > self.min_score_thresh:
                # Detection id & scores
                result = ObjectHypothesisWithPose()
                result.id = output_dict['detection_classes'][i]
                result.score = output_dict['detection_scores'][i]

                # Bounding box
                ymin = output_dict['detection_boxes'][i][0] * image_height
                ymax = output_dict['detection_boxes'][i][2] * image_height
                xmin = output_dict['detection_boxes'][i][1] * image_width
                xmax = output_dict['detection_boxes'][i][3] * image_width

                # Creating message
                det = Detection2D()
                det.results.append(result)
                det.bbox = BoundingBox2D(
                    center = Pose2D(x = (xmin + xmax) / 2, y = (ymin + ymax) / 2, theta = 0),
                    size_x = xmax - xmin,
                    size_y = ymax - ymin
                )
                self.detection_msg.detections.append(det)

                # Notify
                # print(self.category_index[output_dict['detection_classes'][i]]['name'])
        
        self.detection_pub.publish(self.detection_msg)

    # Detecting objects courtesy of:
    # https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/Easy_Object_Detection_With_Custom_Data_Demo_Training.ipynb
    def running_detection(self):
        """ Function running continuously to detect objects
        """
        with self.detection_graph.as_default():
            with tf.Session() as sess:
                while not self.terminate:
                    # Wait for next frame or termination
                    while not self.new_frame and not self.terminate:
                        rospy.sleep(0.001)
                    if self.terminate:
                        break
                    # Get the image
                    self.image_lock.acquire()
                    image = self.image
                    self.new_frame = False
                    self.image_lock.release()

                    # Downscale image courtesy of:
                    # https://stackoverflow.com/questions/4195453/how-to-resize-an-image-with-opencv2-0-and-python2-6
                    image_np = np.asarray(cv2.resize(self.br.imgmsg_to_cv2(image), None, fx=0.25, fy=0.25))
                    # print(image_np.shape)

                    # BEGIN DETECTION
                    # Get handles to input and output tensors
                    ops = tf.compat.v1.get_default_graph().get_operations()
                    all_tensor_names = {
                        output.name for op in ops for output in op.outputs}
                    tensor_dict = {}
                    for key in [
                        'num_detections', 'detection_boxes', 'detection_scores',
                        'detection_classes', 'detection_masks'
                    ]:
                        tensor_name = key + ':0'
                        if tensor_name in all_tensor_names:
                            tensor_dict[key] = tf.compat.v1.get_default_graph().get_tensor_by_name(
                                tensor_name)
                    if 'detection_masks' in tensor_dict:
                        # The following processing is only for single image
                        detection_boxes = tf.squeeze(
                            tensor_dict['detection_boxes'], [0])
                        detection_masks = tf.squeeze(
                            tensor_dict['detection_masks'], [0])
                        # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                        real_num_detection = tf.cast(
                            tensor_dict['num_detections'][0], tf.int32)
                        detection_boxes = tf.slice(detection_boxes, [0, 0], [
                                                    real_num_detection, -1])
                        detection_masks = tf.slice(detection_masks, [0, 0, 0], [
                                                    real_num_detection, -1, -1])
                        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                            detection_masks, detection_boxes, image_np.shape[0], image_np.shape[1])
                        detection_masks_reframed = tf.cast(
                            tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                        # Follow the convention by adding back the batch dimension
                        tensor_dict['detection_masks'] = tf.expand_dims(
                            detection_masks_reframed, 0)
                    image_tensor = tf.compat.v1.get_default_graph().get_tensor_by_name('image_tensor:0')

                    # Run inference
                    output_dict = sess.run(tensor_dict,
                                            feed_dict={image_tensor: np.expand_dims(image_np, 0)})

                    # all outputs are float32 numpy arrays, so convert types as appropriate
                    output_dict['num_detections'] = int(
                        output_dict['num_detections'][0])
                    output_dict['detection_classes'] = output_dict[
                        'detection_classes'][0].astype(np.uint8)
                    output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
                    output_dict['detection_scores'] = output_dict['detection_scores'][0]
                    if 'detection_masks' in output_dict:
                        output_dict['detection_masks'] = output_dict['detection_masks'][0]
                        
                    # END DETECTION

                    # Output the detected objects courtesy of:
                    # https://stackoverflow.com/a/45880222
                    # print(rospy.Time.now() - image.header.stamp)
                    # print([[
                    #         output_dict['detection_boxes'][i]
                    #     ]
                    #     for i in range(len(output_dict['detection_classes']))
                    #     if output_dict['detection_scores'][i] > self.min_score_thresh])
                    self.publish_detection(output_dict, image)
        
    def destructor(self):
        self.terminate = True

def faux_detector():
    # Create model
    model_file = rospy.get_param('~model_file')
    label_file = rospy.get_param('~label_file')
    min_score_thresh = rospy.get_param('~min_score_thresh')
    interval = rospy.get_param('~interval')
    detection_topic = rospy.get_param('~detection_topic')
    detector = Detector(model_file, label_file, min_score_thresh, interval, detection_topic)

    # Base params
    rospy.Subscriber(rospy.get_param('~image_topic'), Image, detector.image_callback)
    rospy.on_shutdown(detector.destructor)
    # run_publisher = rospy.Publisher(rospy.get_param('~run_topic'), Bool, queue_size=1, latch=True)

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