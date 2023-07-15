import subprocess
import pickle
import PIL.Image
import struct
import io
import pandas
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
import numpy as np
import rospy
from sensor_msgs.msg import Image
import threading
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D

class Detector():
    # Helper functions to communicate with subprocess
    def send_data(self, data, output):
        """
        Send data to PIPE identified by output, only works reliably with bytes.
        """
        serialized_data = pickle.dumps(data, protocol=2)
        size = len(serialized_data)
        output.write(struct.pack('!I', size))
        output.write(serialized_data)
        output.flush()

    def receive_data(self, input):
        """
        Receive data to PIPE identified by input, only works reliably with bytes.
        """
        size_data = input.read(4)
        if not size_data:  # No more data, return None
            return None
        size = struct.unpack('!I', size_data)[0]
        data = input.read(size)
        return pickle.loads(data)

    def __init__(self, model_path, weights_file, subprocess_file, subprocess_python_file, min_score_thresh, interval, detection_topic):
        """ Initialize detector
        :param model_path: Path to model
        :param weights_file: Path to weights file
        :param subprocess_file: Path to subprocess file
        :param subprocess_python_file: Path to python3 executable with PyTorch
        :param min_score_thresh: How high score is considered acceptable
        :param interval: How much time (in milliseconds) to wait between inferences
        """
        
        # Create subprocess
        self.process = subprocess.Popen(
            [subprocess_python_file, subprocess_file],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        # Send paths to the subprocess
        self.send_data(bytes(model_path), self.process.stdin)
        self.send_data(bytes(weights_file), self.process.stdin)

        # Image feed
        self.br = CvBridge()
        self.new_frame = False
        self.image = Image()
        self.image_lock = threading.Lock()
        self.image_downsize_factor = 1

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
        
    def publish_detection(self, result_df, image):
        """ Function to publish detections
        :param image: sensor_msgs.msg.Image, image message used to do detections on
        """
        # Forming message
        self.detection_msg.header.stamp = image.header.stamp
        self.detection_msg.detections = []
        result_df = result_df.loc[result_df['confidence'] > self.min_score_thresh]
        for index, row in result_df.iterrows():
            # Detection id & scores
            result = ObjectHypothesisWithPose()
            result.id = row['class']
            result.score = row['confidence']

            # Bounding box
            ymin = row['ymin'] * self.image_downsize_factor
            ymax = row['ymax'] * self.image_downsize_factor
            xmin = row['xmin'] * self.image_downsize_factor
            xmax = row['xmax'] * self.image_downsize_factor

            # Creating message
            det = Detection2D()
            det.results.append(result)
            det.bbox = BoundingBox2D(
                center = Pose2D(x = (xmin + xmax) / 2, y = (ymin + ymax) / 2, theta = 0),
                size_x = xmax - xmin,
                size_y = ymax - ymin
            )
            self.detection_msg.detections.append(det)
        
        self.detection_pub.publish(self.detection_msg)

    # Detecting objects courtesy of:
    # https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/Easy_Object_Detection_With_Custom_Data_Demo_Training.ipynb
    def running_detection(self):
        """ Function running continuously to detect objects
        """
        self.process_running = True
        while not self.terminate:
            try:
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
                
                # Convert image to PIL and send to subprocess
                img_bytes = io.BytesIO()
                pil_im = PIL.Image.fromarray(cv2.resize(
                    self.br.imgmsg_to_cv2(image), None,
                    fx=1.0/self.image_downsize_factor,
                    fy=1.0/self.image_downsize_factor
                ))
                pil_im.save(img_bytes, format='BMP')
                self.send_data(img_bytes.getvalue(), self.process.stdin)
                result = self.receive_data(self.process.stdout)

                # Check if problem with detection
                if result is None:
                    print("Received blank detection, subprocess probably died")
                    break

                # Pass dataframe to publisher
                result_df = pandas.read_json(result)
                self.publish_detection(result_df, image)
            except IOError as e:
                print("faux_subprocess pipe error: " + str(e))
                self.process_running = False
                break
            except Exception as e:
                print("Exception: " + str(e))
                print(self.process.stderr.read())
                # Inform the subprocess to close
                self.send_data(None, self.process.stdin)
                self.process_running = False
                break
        
        # Wait until ROS shutdown
        while not self.terminate:
            continue
                
    def destructor(self):
        self.terminate = True
        if self.process_running:
            # Inform the subprocess to close
            self.send_data(None, self.process.stdin)

        self.process.stdin.close()
        self.process.stdout.close()
        self.process.stderr.close()
        # self.process.terminate()
        self.process.wait()