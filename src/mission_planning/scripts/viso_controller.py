from curses.panel import bottom_panel
import math
from turtle import pos

import numpy
import rospy
import threading
from vision_msgs.msg import Detection2DArray, BoundingBox2D
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointCloud, Image
from geometry_msgs.msg import Point, PoseArray, Pose, Transform, Vector3, Vector3Stamped, TransformStamped, Quaternion
import tf2_ros
import numpy as np
from numpy.linalg import norm
import tf2_geometry_msgs.tf2_geometry_msgs as tfgmtfgm
import tf
from geometry_msgs.msg import Point32
from object_detection.utils import label_map_util

from transforms3d import euler
from geometry_msgs.msg import PoseArray, Pose, Transform, PointStamped, PoseStamped, Quaternion
import numpy as np
import message_filters
from copy import deepcopy
import std_msgs.msg
import std_msgs
import cv2
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, cos, sin, sqrt, pi
from cv_bridge import CvBridge, CvBridgeError
from calc_utils import getDegsDiff

import color_normalization
from enum import Enum
class camera_type(Enum):
    FRONT = "front"
    BOTTOM = "bottom"
class camera():
    def set_params_general(self,cam_enum):
        self.world_frame = rospy.get_param('~world_frame')
        self.detection_label_path = rospy.get_param('~front_label_file')
        self.width=rospy.get_param('~'+cam_enum.value+'_camera_width')
        self.height=rospy.get_param('~'+cam_enum.value+'_camera_height')
        self.aspect_ratio = self.width/self.height
        self.Hfov=math.radians(rospy.get_param('~'+cam_enum.value+'_camera_HFOV'))
        self.detection_topic=rospy.get_param('~'+cam_enum.value+'_detection_topic')
        self.sub_topic=rospy.get_param('~'+cam_enum.value+'_camera_topic')
        self.is_sim=bool(rospy.get_param('~is_sim'))
        # if(cam_enum.value=="front"):
        #     self.sub_topic=rospy.get_param('~'+cam_enum.value+'_camera_topic')
        # else:
        #     self.sub_topic=rospy.get_param('~'+cam_enum.value+'_camera_topic')
        
    def __init__(self,cam_enum):
        self.set_params_general(cam_enum)
        self.label_to_id=label_map_util.get_label_map_dict(self.detection_label_path) #dictionary mapping label -> id
        self.Vfov= 2 * math.atan(math.tan(self.Hfov/2)*self.aspect_ratio)

        self.is_fetched=False
        self.last_detection_time = 0.0 # Saving last detection time
        self.viso_detect=Detection2DArray()
        self.detection_sub = message_filters.Subscriber(self.detection_topic, Detection2DArray)

        self.mutex = threading.Lock()

        # Getting transformation from camera frame to world
        self.trans = TransformStamped()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #Opencv manip to get bbox visualizations 
        self.cv_image=None
        self.bridge = CvBridge()

        self.image_mutex=threading.Lock()
        self.image_sub=message_filters.Subscriber(self.sub_topic, Image)

    #TESTED
    def get_angles(self,pixel_x,pixel_y):
        #Input: pixel coordinates
        #Returns: (angle relative to center along x axis, angle relative to center along y axis)
        L = self.width / (2 * math.tan(self.Hfov / 2))
        angle_x = math.atan((pixel_x - self.width / 2) / L)
        angle_y = math.atan((pixel_y - self.height / 2) / L)
        return angle_x, angle_y

    def set_HFOV(self,Hfov):
        self.Hfov=Hfov
        self.Vfov= 2 * math.atan(math.tan(Hfov/2)*self.aspect_ratio)

    def get_center_coord(self):
        return [self.width/2,self.height/2]

    def get_detection_t(self, label, timeout = 0.1):
        #try and get detections over a specified period of time instead of just the last fetched instant
        detection = []
        current_time=rospy.get_time()
        goal_time = current_time + timeout

        while(current_time <= goal_time):
            current_time=rospy.get_time()
            if(self.is_fetched):
                detection = self.get_detection([label])
                if(len(detection)!=0):
                    return detection
                    
        return detection
    def get_detection(self, array_of_labels = [], confidence_threshold = 0.1): #np array of labels passed, returns dictionary of detections for those id's
        # if(self.is_fetched): # Might not need this if want to get again

        #add distance label
        detections = []
        self.mutex.acquire()
        if len(array_of_labels) > 0: 
            req_ids = [self.get_id_from_label(x) for x in array_of_labels]
            for det in self.viso_detect.detections:
                if det.results[0].id in req_ids:
                    if det.results[0].score > confidence_threshold:
                        detections.append({
                            'id': det.results[0].id,                            # ID
                            'score': det.results[0].score,                      # Confidence / score
                            'center': (det.bbox.center.x, det.bbox.center.y),   # Center, tuple (x, y)
                            'size': (det.bbox.size_x, det.bbox.size_y),         # Size, tuple (x, y)
                            'label': self.get_label_from_id(det.results[0].id), # Label string
                            'time': self.viso_detect.header.stamp               # Time, rospy.Time
                        })
            self.is_fetched=False
        else:
            if(len(self.viso_detect.detections)>0):
                for det in self.viso_detect.detections:
                    if det.results[0].score > confidence_threshold:
                        detections.append({
                            'id': det.results[0].id,                            # ID
                            'score': det.results[0].score,                      # Confidence / score
                            'center': (det.bbox.center.x, det.bbox.center.y),   # Center, tuple (x, y)
                            'size': (det.bbox.size_x, det.bbox.size_y),         # Size, tuple (x, y)
                            'label': self.get_label_from_id(det.results[0].id), # Label string
                            'time': self.viso_detect.header.stamp               # Time, rospy.Time
                        })
            self.is_fetched=False
        self.mutex.release()
        
        return detections
    def update_tf(self, timeout=5):
        """Function to update stored tf of camera
        :param timeout: How much time in seconds lookup should wait for"""
        delay = 0.1
        counter = 0
        # Find current location
        while counter < timeout / delay:
            try:
                self.trans = self.tfBuffer.lookup_transform(self.world_frame, self.camera_frame, rospy.Time(), rospy.Duration(4))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Cannot find transformation of ROV! Retrying...")
                rospy.sleep(delay)
                counter += 1
            break
        # Check if there were any issues
        return (counter < timeout / delay)

    def get_id_from_label(self,label): #lookup label_to_id dictionary and return label for a specific id
        return self.label_to_id[label]

    def get_label_from_id(self, id):
        return self.label_to_id.keys()[self.label_to_id.values().index(id)]

    def get_cv_img(self):
        self.image_mutex.acquire()
        print("GETTING CV IMAGE")
        #imgn=color_normalization.get_normalized_image(self.cv_image)FOR REAL TESTING
        imgn=self.cv_image.copy()
        self.image_mutex.release()
       
        return imgn
    def get_ros_img(self):
        self.image_mutex.acquire()
        try:
            img = self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image_mutex.release()
        return img
    
    # Get location of objects from intersection between ray and plane
    def pixel_ray_plane_intersect(self,
                                  pixel_xy,                     # Pixel on image                        | Tuple (x, y)
                                  plane_xyz,                    # Plane origin point                    | Tuple (x, y, z)
                                  plane_normal):                # Plane normal vector                   | Tuple (x, y, z)

        # Get camera pose
        self.update_tf()
        translation = self.trans.transform.translation
        angle_x, angle_y = self.get_angles(pixel_xy[0], pixel_xy[1])
        center_vec_cam = Vector3Stamped(vector=Vector3(math.tan(angle_x), math.tan(angle_y), 1))
        center_vec_world = tfgmtfgm.do_transform_vector3(center_vec_cam, self.trans)

        # Get translations from camera pose
        dot_product = plane_normal[0] * center_vec_world.vector.x + \
                      plane_normal[1] * center_vec_world.vector.y + \
                      plane_normal[2] * center_vec_world.vector.z
        if dot_product == 0:
            return None
        t = (
            plane_normal[0] * (plane_xyz[0] - translation.x) +
            plane_normal[1] * (plane_xyz[1] - translation.y) +
            plane_normal[2] * (plane_xyz[2] - translation.z)
        ) / dot_product
        return Point(center_vec_world.vector.x * t + translation.x,
                     center_vec_world.vector.y * t + translation.y,
                     center_vec_world.vector.z * t + translation.z)

    def get_min_approach_dist(self,width):
        clearance_d=(width/2)/math.tan(self.Hfov/2)
        return clearance_d
class stereo(camera, object):
    def set_params(self):

        self.camera_frame = rospy.get_param('~front_camera_frame')

        self.pcl_topic = rospy.get_param('~pcl_topic')

    def __init__(self):
        super(stereo,self).__init__(camera_type.FRONT)
        self.set_params()
        self.pcl_sub= message_filters.Subscriber(self.pcl_topic, PointCloud2)

        ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub, self.pcl_sub, self.image_sub], 3,slop=0.2)
        ts.registerCallback(self.detection_callback)

        self.labeled_img_pub = rospy.Publisher("/LABELED_IMAGE_STREAM", Image, latch=True, queue_size=10)

        self.pointcloud=PointCloud2()
        self.pointcloud_mutex=threading.Lock()

        self.pcl_pub =rospy.Publisher("POINTS_SVD", PointCloud, latch=True)
    
    def detection_callback(self, viso_detect,pointcloud,image):
        self.mutex.acquire()
        self.viso_detect = viso_detect

        # Only set true when something is detected, to save energy
        if len(list(self.viso_detect.detections)) > 0:
            self.is_fetched=True
            self.last_detection_time = rospy.get_time()
        self.mutex.release() 

        self.pointcloud_mutex.acquire()
        self.pointcloud=pointcloud
        self.pointcloud_mutex.release()

        if(self.is_sim):
            self.image_callback(image)
    
    def image_callback(self,data):
        self.image_mutex.acquire()
        try:
            #get detections 
            self.cv_image=cv2.resize(self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8'),(640,400))
            labeled_cv_image=self.cv_image.copy()
            for det in self.viso_detect.detections:
                ll_vertex=(int(det.bbox.center.x-det.bbox.size_x/2),int(det.bbox.center.y-det.bbox.size_y/2))
                rr_vertex=(int(det.bbox.center.x+det.bbox.size_x/2),int(det.bbox.center.y+det.bbox.size_y/2))
                cv2.rectangle(labeled_cv_image, ll_vertex, rr_vertex,(0,255,0), 3)
                cv2.putText(labeled_cv_image, str(det.results[0].score) , rr_vertex,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
                cv2.putText(labeled_cv_image, str(self.get_label_from_id(det.results[0].id)) , ll_vertex,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
            self.labeled_img_pub.publish(self.bridge.cv2_to_imgmsg(labeled_cv_image))
        except CvBridgeError as e:
            print(e)
        self.image_mutex.release()
    def __np_to_ps(self,np_arr):
        ps = PointStamped()
        ps.point.x=np_arr[0]
        ps.point.y=np_arr[1]
        ps.point.z=np_arr[2]
        return ps
    def __np_to_vector(self,np_arr):
        vc= Vector3Stamped()
        vc.vector.x=np_arr[0]
        vc.vector.y=np_arr[1]
        vc.vector.z=np_arr[2]
        return vc
    def get_ps(self, detection, timeout = 0.5, delay = 0.1, counter = 0):
        #Input: detection object from get detection
        #Returns: Pose of best fit plane of planar object

        if(counter >= timeout / delay):
            return None
        else:
            rospy.sleep(delay)
            counter += 1
            detection_arr = self.get_detection([detection['label']])
            if(len(detection_arr)!=0):
                ps=self.estimate_pose_svd(detection['center'],detection['size'])
                if(ps!=None):
                    print("RETURNING PS")
                    return ps
                else:
                    return self.get_ps(detection_arr[0], timeout, delay, counter)
            else:
               return self.get_ps(detection, timeout, delay, counter) 
            
    def get_distance_det(self, detection):
        #given detection return their distances from the camera by averaging their respective points

        pass

    def estimate_gate_pose(self, points_region, max_points = 300):
        self.pointcloud_mutex.acquire()
        pose_time = self.pointcloud.header.stamp
        #points_region: [[x,y] [x,y] ...]
        M,_=points_region.shape
        temp = int(math.floor(M/max_points))
        if(temp!=0):
            points_region=points_region[::temp]

        points = np.array(list(pc2.read_points(
            self.pointcloud,
            field_names={'x', 'y', 'z'},
            skip_nans=True, uvs=points_region.tolist() # [[u,v], [u,v], [u,v]] a set of xy coords
        ))).T
        self.pointcloud_mutex.release()
        print(points)

        if(not np.any(points)):
            return None
        # [[1,2,3],[,,], ..., dt]

        # Approx. plane normal courtesy of:
        # https://math.stackexchange.com/a/99317
        centroid_cam = np.mean(points, axis=1, keepdims=True)
        svd = np.linalg.svd(points - centroid_cam) # [:,-1]
        left = svd[0]
        normal_cam = left[:, -1] #normal direction vector relative to object plane

        centroid_cam = np.concatenate(centroid_cam,axis=0)
        centroid_cam_ps=self.__np_to_ps(centroid_cam)
        normal_cam_vc=self.__np_to_vector(normal_cam)

        # Transform to world frame
        import movement_controller as mc
        self.update_tf()
        mc.mov_control.update_tf()
        rotation = mc.mov_control.trans.transform.rotation
        roll_rov, pitch_rov, yaw_rov = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')

        centroid_world = tfgmtfgm.do_transform_point(centroid_cam_ps, self.trans)
        normal_world = tfgmtfgm.do_transform_vector3(normal_cam_vc, self.trans)

        # if(self.is_sim):
            #Publish points in space for testing
        pcld = PointCloud()
        pcld.header = std_msgs.msg.Header()
        pcld.header.stamp = rospy.Time.now()
        pcld.header.frame_id = self.camera_frame
        pts = points.T
        for i,j in np.ndindex(pts.shape):
            if(j==0):
                pcld.points.append(Point32(pts[i][0],pts[i][1],pts[i][2]))
        self.pcl_pub.publish(pcld)

        # Assume uprightness
        pose_yaw =  math.atan2(normal_world.vector.y, normal_world.vector.x)
        if(abs(getDegsDiff(pose_yaw,yaw_rov)) > math.pi / 2):
            #make sure x axis is into the page of the detection
            pose_yaw += math.pi
            pose_yaw, aj, ak = euler.axangle2euler([1, 0, 0], pose_yaw)
            print(pose_yaw)

        q = euler.euler2quat(0, 0, pose_yaw, 'sxyz')
        pose_msg = PoseStamped()
        pose_msg.header.stamp = pose_time
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position = centroid_world.point
        pose_msg.pose.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        return pose_msg

    def estimate_pose_svd(self, center, size, max_points = 250, ratio = 0.4): # Estimate pose of flat thing with SVD
        """ 
            Input:
                max_points: Max number of points allowed for sample
                ratio: Ratio of bounding box width and height to select
                center: Numpy array of bounding box center in [x,y]
                size: Numpy array of bounding box size in [width,height]
        """

        # Get points from pcl courtesy of:
        # http://docs.ros.org/en/jade/api/sensor_msgs/html/point__cloud2_8py_source.html
        # https://answers.ros.org/question/240491/point_cloud2read_points-and-then/
        if(size[0] * ratio > max_points/2):
            r_x = max_points/4
        else:
            r_x = size[0]/2 * ratio
        
        if(size[1] * ratio > max_points/2):
            r_y = max_points/4
        else:
            r_y = size[1]/2 * ratio

        self.update_tf()
        points_region = np.mgrid[
            math.ceil(center[0] - r_x):math.floor(center[0] + r_x),
            math.ceil(center[1] - r_y):math.floor(center[1] + r_y)].reshape(2,-1).T.astype(int)
        self.pointcloud_mutex.acquire()
        pose_time = self.pointcloud.header.stamp
        points = np.array(list(pc2.read_points(
            self.pointcloud,
            field_names={'x', 'y', 'z'},
            skip_nans=True, uvs=points_region.tolist() # [[u,v], [u,v], [u,v]] a set of xy coords
        ))).T
        self.pointcloud_mutex.release()


        if(not np.any(points)):
            return None
        # [[1,2,3],[,,], ..., dt]

        # Approx. plane normal courtesy of:
        # https://math.stackexchange.com/a/99317
        centroid_cam = np.mean(points, axis=1, keepdims=True)

        svd = np.linalg.svd(points - centroid_cam) # [:,-1]
        left = svd[0]
        normal_cam = left[:, -1] #normal direction vector relative to object plane

        centroid_cam = np.concatenate(centroid_cam,axis=0)
        centroid_cam_ps=self.__np_to_ps(centroid_cam)
        normal_cam_vc=self.__np_to_vector(normal_cam)

        # Transform to world frame
        import movement_controller as mc
        mc.mov_control.update_tf()
        rotation = mc.mov_control.trans.transform.rotation
        roll_rov, pitch_rov, yaw_rov = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')

        centroid_world = tfgmtfgm.do_transform_point(centroid_cam_ps, self.trans)
        normal_world = tfgmtfgm.do_transform_vector3(normal_cam_vc, self.trans)

        if(self.is_sim):
            #Publish points in space for testing
            pcld = PointCloud()
            pcld.header = std_msgs.msg.Header()
            pcld.header.stamp = rospy.Time.now()
            pcld.header.frame_id = self.camera_frame
            pts = points.T
            for i,j in np.ndindex(pts.shape):
                if(j==0):
                    pcld.points.append(Point32(pts[i][0],pts[i][1],pts[i][2]))
            self.pcl_pub.publish(pcld)

        # Assume uprightness
        pose_yaw =  math.atan2(normal_world.vector.y, normal_world.vector.x)
        if(abs(getDegsDiff(pose_yaw,yaw_rov))>math.pi/2):
            #make sure x axis is into the page of the detection
            pose_yaw += math.pi
            if(pose_yaw>math.pi):
                pose_yaw=pose_yaw-2*math.pi

        q = euler.euler2quat(0, 0, pose_yaw, 'sxyz')
        pose_msg = PoseStamped()
        pose_msg.header.stamp = pose_time
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position = centroid_world.point
        pose_msg.pose.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        return pose_msg

class mono(camera, object):
    def set_params(self):

        self.camera_frame = rospy.get_param('~bottom_camera_frame')

        self.pcl_topic = rospy.get_param('~pcl_topic')

    def __init__(self):
        super(mono,self).__init__(camera_type.BOTTOM)
        self.set_params()
        
        ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub,self.image_sub], 3,slop=0.2)
        ts.registerCallback(self.detection_callback)
    
    def detection_callback(self, viso_detect,image):
        self.mutex.acquire()
        self.viso_detect = viso_detect
        # Only set true when something is detected, to save energy
        if len(list(self.viso_detect.detections)) > 0:
            self.is_fetched=True
            self.last_detection_time = rospy.get_time()
        self.mutex.release()

        self.image_callback(image)

    def image_callback(self,data):
        self.image_mutex.acquire()
        try:
            self.cv_image = cv2.resize(self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8'),(150,150))
        except CvBridgeError as e:
            print(e)
        self.image_mutex.release()
        
def init():
    global front_camera,bottom_camera
    front_camera=stereo()
    bottom_camera=mono()