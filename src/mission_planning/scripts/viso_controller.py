from curses.panel import bottom_panel
import math

import numpy
import rospy
import threading
from vision_msgs.msg import Detection2DArray, BoundingBox2D
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseArray, Pose, Transform, Vector3Stamped, TransformStamped, Quaternion
import tf2_ros
import numpy as np
from numpy.linalg import norm
import tf2_geometry_msgs.tf2_geometry_msgs as tfgmtfgm
import tf
from object_detection.utils import label_map_util

from transforms3d import euler
from geometry_msgs.msg import PoseArray, Pose, Transform, PointStamped, PoseStamped, Quaternion
import numpy as np
import message_filters
from copy import deepcopy

import cv2
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, cos, sin, sqrt, pi
from cv_bridge import CvBridge, CvBridgeError

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

        if(cam_enum.value=="front"):
            self.sub_topic=None
        else:
            self.sub_topic=rospy.get_param('~'+cam_enum.value+'_camera_topic')
        
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

    def get_detection(self, array_of_labels): #np array of labels passed, returns dictionary of detections for those id's
        # if(self.is_fetched): # Might not need this if want to get again
            detections = []
            if len(array_of_labels) > 0:
                self.mutex.acquire()
                req_ids = [self.get_id_from_label(x) for x in array_of_labels]
                for det in self.viso_detect.detections:
                    if det.results[0].id in req_ids:
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

class stereo(camera, object):
    def set_params(self):

        self.camera_frame = rospy.get_param('~front_camera_frame')

        self.pcl_topic = rospy.get_param('~pcl_topic')

    def __init__(self):
        super(stereo,self).__init__(camera_type.FRONT)
        self.set_params()
        self.pcl_sub= message_filters.Subscriber(self.pcl_topic, PointCloud2)

        ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub, self.pcl_sub], 3,slop=0.2)
        ts.registerCallback(self.detection_callback)

        self.pointcloud=PointCloud2()
        self.pointcloud_mutex=threading.Lock()
    
    def detection_callback(self, viso_detect,pointcloud):

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

    def estimate_pose_svd(self, center, size): # Estimate pose of flat thing with SVD
        # Get points from pcl courtesy of:
        # http://docs.ros.org/en/jade/api/sensor_msgs/html/point__cloud2_8py_source.html
        # https://answers.ros.org/question/240491/point_cloud2read_points-and-then/
        pose_time = self.pointcloud.header.stamp
        points_region = np.mgrid[
            math.ceil(center[0] - size[0] * 0.5):math.floor(center[0] + size[0] * 0.5),
            math.ceil(center[1] - size[1] * 0.5):math.floor(center[1] + size[1] * 0.5)].reshape(2,-1).T.astype(int)
        self.pointcloud_mutex.acquire()
        points = np.array(list(pc2.read_points(
            self.pointcloud,
            field_names={'x', 'y', 'z'},
            skip_nans=True, uvs=points_region.tolist() # [u,v u,v u,v] a set of xy coords
        )))
        self.pointcloud_mutex.release()
        # [[1,2,3],[,,], ..., dt]

        # Approx. plane normal courtesy of:
        # https://math.stackexchange.com/a/99317
        centroid_cam = np.mean(points, axis=1, keepdims=True)
        svd = np.linalg.svd(points - centroid_cam) # [:,-1]
        left = svd[0]
        normal_cam = left[:, -1] #normal direction vector relative to object plane

        # Transform to world frame
        self.update_tf()
        centroid_world = tfgmtfgm.do_transform_point(centroid_cam, self.trans)
        normal_world = tfgmtfgm.do_transform_vector3(normal_cam, self.trans)

        # Assume uprightness
        pose_yaw =  math.atan2(normal_world.vector.y, normal_world.vector.x)
        q = euler.euler2quat(0, 0, pose_yaw, 'sxyz')
        pose_msg = PoseStamped()
        pose_msg.header.stamp = pose_time
        pose_msg.header.frame_id = self.world_frame
        pose_msg.position = centroid_world
        pose_msg.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        return pose_msg

class mono(camera, object):
    def set_params(self):

        self.camera_frame = rospy.get_param('~bottom_camera_frame')

        self.pcl_topic = rospy.get_param('~pcl_topic')

    def __init__(self):
        super(mono,self).__init__(camera_type.BOTTOM)
        self.set_params()

        self.image_mutex=threading.Lock()
        self.image_sub=message_filters.Subscriber(self.sub_topic, Image)
        self.cv_image=None
        self.bridge = CvBridge()
        
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

# class viso_controller():
    
#     def __init__(self,world_frame,camera_frame,detection_topic,pcl_topic,bottom_camera_topic,detection_label_path): #should be passed in radians

#         self.world_frame=world_frame
#         self.camera_frame=camera_frame

#         self.pcl_sub= message_filters.Subscriber(pcl_topic, PointCloud2)
#         self.detection_sub = message_filters.Subscriber(detection_topic, Detection2DArray)
#         #OAK_D=camera(50.41,640,400)
#         #OAK_1=camera(69,1920,1080)

#         ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub, self.pcl_sub], 3,slop=0.2)
#         ts.registerCallback(self.detection_callback)
#         #self.detection_sub = rospy.Subscriber(detection_topic, Detection2DArray, self.detection_callback)

#         self.label_to_id=label_map_util.get_label_map_dict(detection_label_path) #dictionary mapping label -> id
#         self.mutex = threading.Lock()

#         self.is_fetched=False

#         self.pointcloud=PointCloud2()
#         self.pointcloud_mutex=threading.Lock()

#         self.viso_detect=Detection2DArray()

#         # Getting transformation from camera frame to world
#         self.trans = TransformStamped()
#         self.tfBuffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.tfBuffer)
        

#     def get_id_from_label(self,label): #lookup label_to_id dictionary and return label for a specific id
#         return self.label_to_id[label]

#     def get_label_from_id(self, id):
#         return self.label_to_id.keys()[self.label_to_id.values().index(id)]

#     def detection_callback(self, viso_detect,pointcloud):

#         self.mutex.acquire()
#         self.viso_detect = viso_detect
#         # Only set true when something is detected, to save energy
#         if len(list(self.viso_detect.detections)) > 0:
#             self.is_fetched=True
#         self.mutex.release()

#         self.pointcloud_mutex.acquire()
#         self.pointcloud=pointcloud
#         self.pointcloud_mutex.release()

#     def get_detection(self, array_of_labels): #np array of labels passed, returns dictionary of detections for those id's
#         # if(self.is_fetched): # Might not need this if want to get again
#             detections = []
#             if len(array_of_labels) > 0:
#                 self.mutex.acquire()
#                 req_ids = [self.get_id_from_label(x) for x in array_of_labels]
#                 for det in self.viso_detect.detections:
#                     if det.results[0].id in req_ids:
#                         detections.append({
#                             'id': det.results[0].id,                            # ID / score
#                             'score': det.results[0].score,                      # Confi
#                             'center': (det.bbox.center.x, det.bbox.center.y),   # Center, tuple (x, y)
#                             'size': (det.bbox.size_x, det.bbox.size_y),   
#                             'label':  self.get_label_from_id(det.results[0].id)# Size, tuple (x, y)
#                         })
#                 # detections=self.viso_detect.detections
#                 # det_ids={} #{id1 : [Detection2D,], id2}

#                 # for det in list(detections):
#                 #     if det.results.id in req_ids:
#                 #         if det.results.id in det_ids:
#                 #             det_ids[self.get_label_from_id(det.results.id)].append(det)
#                 #         else:
#                 #             det_ids[self.get_label_from_id(det.results.id)]=[det]
#                 self.is_fetched=False
#                 self.mutex.release()
#             return detections
#                 # return det_ids
#             # self.is_fetched=False # keep is_fetched true as nothing was fetched

#             # list_new=list.copy(detections)
#             # for det in list(list_new): #this will remove ids we are not interested in
#             #     id=det.results.id
#             #     if(id in ids):
#             #         pass
#             #     else:
#             #         list_new.remove(det)   
            
#             # #unordered list 

#             # self.mutex.release()
#             # return list_new 

#             #{id:Detection2DArray,id:}
#         # else:
#         #     self.mutex.release()
#         #     return None

#     def pcl_detection_callback(self,viso_detect,pointcloud):
#         self.detection_callback(viso_detect)
#         self.pointcloud=pointcloud


#     def get_position_of_pixel(self,x_coord,y_coord): #get xyz of pixel from x,y coord in ros img frame
#         pass
#     #Courtesy of https://github.com/paul-shuvo/planar_pose/blob/main/src/planar_pose_estimation.py
#     def estimate_pose(
#                 self,
#                 object_name,
#                 bbox,#: BoundingBox2D, #change this to bbox object and decreaese its size
#             ):
#         """
#         Estimates planar pose of detected objects and
#         updates the stored pose.
#         Parameters
#         ----------
#         object_name: str
#             Name of the object.
#         bbox : bbox object
#             Contains the coordinates of the bounding box
#             of the detected object.
#         pc_sub : PointCloud2
#             A pointcloud object containing the 3D locations
#             in terms of the frame `self.frame_id`
#         """
#         x_center=bbox.center.x
#         y_center=bbox.center.y
        
#         #decrease the bbox size by half because we might have some areas not in the detection if at an angle or slop
#         size_x=bbox.size_x//2
#         size_y=bbox.size_y//2

#         bbox = np.array()

#         # Compute the center, the mid point of the right
#         # and top segment of the bounding box

#         #get c,x,y
#         c=[x_center,y_center]
#         x=[math.floor(x_center+size_x*0.5),y_center]
#         y=[x_center,math.floor(y_center-size_y*0.5)]

#         points_region = np.mgrid[math.floor(x_center-size_x*0.5):x[0], y[1]:math.floor(y_center-size_y*0.5):1].reshape(2,-1).T
        

#         points = np.array([c, x, y]).tolist()
#         vectors_3D = np.zeros((3, 3))

#         # pcl_region = pc2.read_points(
#         #                 pc_sub,
#         #                 field_names={'x', 'y', 'z'},
#         #                 skip_nans=False, uvs=points_region.tolist() # [u,v u,v u,v] a set of xy coords
#         #             )
#         # # [[1,2,3],[,,], ..., dt]

#         # norm = np.linalg.svd(pcl_region.T - np.mean(pcl_region.T, keepdims=True))[:,-1] #normal direction vector relative to object plane
#         # center = np.mean(pcl_region,axis=0)
#         # norm_v=Vector3(norm[0],norm[1],norm[2])
        
#         # t = mc.mov_control.tfBuffer.lookup_transform(self.world_frame, self.camera_frame, rospy.Time()) #make this oakd_frame


#         # norm_world = tf2_geometry_msgs.do_transform_vector3(norm_v, t.inverse()) #normal in the world_frame VECTOR3
#         # norm_world.z=0

#         try:
#             # Get the corresponding 3D location of c, x, y
#             self.pointcloud_mutex.acquire()
#             for pt_count, dt in enumerate(
#                 pc2.read_points(
#                         self.pointcloud,
#                         field_names={'x', 'y', 'z'},
#                         skip_nans=False, uvs=points # [u,v u,v u,v] a set of xy coords
#                     )
#                 ):
#                 # # If any point returns nan, return
#                 if np.any(np.isnan(dt)):
#                     if object_name in self.object_pose_info.keys():
#                         del self.object_pose_info[object_name]
#                     rospy.loginfo('No corresponding 3D point found')
#                     self.pointcloud_mutex.acquire()
#                     return
#                 else:
#                     vectors_3D[pt_count] = dt
#                     if pt_count == 2:
#                         self.vectors_3D = vectors_3D
#                 self.pointcloud_mutex.acquire()
#         except:
#             #rospy.loginfo(err)
#             print("HORRIBLE ERROR REACHED IN PCL")
#             self.pointcloud_mutex.acquire()
#             return

#         try:
#             # 3D position of the object
#             c_3D = self.vectors_3D[0]

#             # Center the vectors to the origin
#             x_vec = self.vectors_3D[1] - c_3D
#             x_vec /= norm(x_vec)

#             y_vec = self.vectors_3D[2] - c_3D
#             y_vec /= norm(y_vec)
#             # Take the cross product of x and y vector
#             # to generate z vector.
#             z_vec = np.cross(x_vec, y_vec)
#             z_vec /= norm(z_vec)

#             # Recompute x vector to make it truly orthognal
#             x_vec_orth = np.cross(y_vec, z_vec)
#             x_vec_orth /= norm(x_vec_orth)
#         except RuntimeWarning as w:
#             rospy.loginfo(w)
#             return

#         if self.viz_pose:
#             self.draw_pose(object_name, np.vstack((self.vectors_3D, z_vec)))

#         # Compute Euler angles i.e. roll, pitch, yaw
#         roll = np.arctan2(y_vec[2], z_vec[2])
#         pitch = np.arctan2(-x_vec_orth[2], np.sqrt(1 - x_vec_orth[2]**2))
#         # pitch = np.arcsin(-x_vec_orth[2])
#         yaw = np.arctan2(x_vec_orth[1], x_vec_orth[0])

#         [qx, qy, qz, qw] = self.euler_to_quaternion(roll, pitch, yaw)

#         # Generate Pose message.
#         pose_msg = Pose()

#         pose_msg.position.x = c_3D[0]
#         pose_msg.position.y = c_3D[1]
#         pose_msg.position.z = c_3D[2]

#         # Make sure the quaternion is valid and normalized
#         pose_msg.orientation.x = qx
#         pose_msg.orientation.y = qy
#         pose_msg.orientation.z = qz
#         pose_msg.orientation.w = qw

#         return pose_msg

#     def estimate_pose_svd(self, center, size): # Estimate pose of flat thing with SVD
#         # Get points from pcl courtesy of:
#         # http://docs.ros.org/en/jade/api/sensor_msgs/html/point__cloud2_8py_source.html
#         # https://answers.ros.org/question/240491/point_cloud2read_points-and-then/
#         pose_time = self.pointcloud.header.stamp
#         points_region = np.mgrid[
#             math.ceil(center[0] - size[0] * 0.5):math.floor(center[0] + size[0] * 0.5),
#             math.ceil(center[1] - size[1] * 0.5):math.floor(center[1] + size[1] * 0.5)].reshape(2,-1).T
#         self.pointcloud_mutex.acquire()
#         points = np.array(list(pc2.read_points(
#             self.pointcloud,
#             field_names={'x', 'y', 'z'},
#             skip_nans=True, uvs=points_region.tolist() # [u,v u,v u,v] a set of xy coords
#         )))
#         self.pointcloud_mutex.release()
#         # [[1,2,3],[,,], ..., dt]

#         # Approx. plane normal courtesy of:
#         # https://math.stackexchange.com/a/99317
#         centroid_cam = np.mean(points, axis=1, keepdims=True)
#         svd = np.linalg.svd(points - centroid_cam) # [:,-1]
#         left = svd[0]
#         normal_cam = left[:, -1] #normal direction vector relative to object plane

#         # Transform to world frame
#         self.update_tf()
#         centroid_world = tfgmtfgm.do_transform_point(centroid_cam, self.trans)
#         normal_world = tfgmtfgm.do_transform_vector3(normal_cam, self.trans)

#         # Assume uprightness
#         pose_yaw =  math.atan2(normal_world.vector.y, normal_world.vector.x)
#         q = euler.euler2quat(0, 0, pose_yaw, 'sxyz')
#         pose_msg = PoseStamped()
#         pose_msg.header.stamp = pose_time
#         pose_msg.header.frame_id = self.world_frame
#         pose_msg.position = centroid_world
#         pose_msg.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
#         return pose_msg

#     def recursive_pcl(pc_sub, point, x_axis): #fix along x_axis
#         # Get the corresponding 3D location of c, x, y
#         for dt in enumerate(
#             pc2.read_points(
#                     pc_sub,
#                     field_names={'x', 'y', 'z'},
#                     skip_nans=False, uvs=point # [u,v u,v u,v] a set of xy coords
#                 )
#             ):
#             # If any point returns nan, return
#             if np.any(np.isnan(dt)):
#                 if(x_axis):
#                     point[0]=point[0]-1
#                 else:
#                     point[1]=point[1]-1

#                 return (viso_controller.recursive_pcl(pc_sub,point,x_axis))
#             else:
#                 return dt

#     def update_tf(self, timeout=5):
#         """Function to update stored tf of ROV
#         :param timeout: How much time in seconds lookup should wait for"""
#         delay = 0.1
#         counter = 0
#         # Find current location
#         while counter < timeout / delay:
#             try:
#                 self.trans = self.tfBuffer.lookup_transform(self.world_frame, self.camera_frame, rospy.Time(), rospy.Duration(4))
#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 print("Cannot find transformation of ROV! Retrying...")
#                 rospy.sleep(delay)
#                 counter += 1
#             break
#         # Check if there were any issues
#         return (counter < timeout / delay)

# Global variable courtesy of:
# https://stackoverflow.com/a/13034908
def init():
    global front_camera,bottom_camera
    front_camera=stereo()
    bottom_camera=mono()
    #viso_control=viso_controller(world_frame,camera_frame,detection_topic,pcl_topic,OAK1_topic,detection_label_path)


    # def np_to_point(numpy_array):
    #     ros_point = PointStamped()
    #     ros_point.point.x = numpy_array[0]
    #     ros_point.point.y = numpy_array[1]
    #     ros_point.point.z = numpy_array[2]
    #     return ros_point
    # def estimate_gate_pose(self,bbox1,bbox2,pc_sub):
    #     vectors_3D = np.zeros((3, 2))
    #     self.pointcloud_mutex.acquire()
    #     for pt_count, dt in enumerate(pc2.read_points(
    #                         pc_sub,
    #                         field_names={'x', 'y', 'z'},
    #                         skip_nans=False, uvs=[[bbox1.center.x,bbox1.center.x],[bbox2.center.y,bbox2.center.y]] # [u,v u,v u,v] a set of xy coords
    #                     )):
    #                 vectors_3D[pt_count]=dt
    #     self.pointcloud_mutex.release()

    #     import movement_controller as mc
    #     transform=mc.mov_control.tfBuffer.lookup_transform(self.world_frame, self.camera_frame, rospy.Time(), rospy.Duration(1))
        
    #     #get 3D midpoint between both points in PCL frame
    #     midpoint=[(vectors_3D[0]+vectors_3D[1])/2]
        
    #     vector_w=vectors_3D[1]-vectors_3D[0]
        
    #     midpoint=self.np_to_point(midpoint)
    #     vector_w=self.np_to_point(vector_w)

    #     #get points in world frame
    #     midpoint=tfgmtfgm.do_transform_point(midpoint,transform)
    #     vector_w=tfgmtfgm.do_transform_point(vector_w,transform)

    #     vector_w=numpy.array([vector_w.point.x,vector_w.point.y,vector_w.point.z])
    #     yaw=math.acos(vector_w[0]/np.linalg.norm(vector_w)) #gets yaw of frame of gate relative to x of world frame

    #     #transform = self.tf_buffer.lookup_transform(frame, required_position, rospy.Time(0), rospy.Duration(1)) #transform stamped
    #     gate_pose = PoseStamped()
    #     gate_pose.pose.position.x=midpoint[0]
    #     gate_pose.pose.position.y=midpoint[1]
    #     gate_pose.pose.position.z=midpoint[2]

    #     #get quaternion of pose
    #     q=euler.euler2quat(0,0,yaw)
    #     gate_pose.pose.orientation.w=q[0]
    #     gate_pose.pose.orientation.x=q[1]
    #     gate_pose.pose.orientation.y=q[2]
    #     gate_pose.pose.orientation.z=q[3]

    #     return gate_pose


    # def detection_callback(self,viso_detect): 
    #     for det in viso_detect.detections:
    #         #check if the region is in array
    #         if(det.results[0].id in self.detection_of_interest):
    #             #if its an id of interest, store it in the dictionary
    #             self.mutex.acquire()
    #             self.detection_of_interest[det.results[0].id]['is_fetched']=True
    #             self.detection_of_interest[det.results[0].id]['detection']=det
    #             self.mutex.release()

    # def get_detection(self,label): #returns Detection2D if label is found, None if otherwise

    #     self.mutex.acquire()
    #     id=self.label_to_id[label]
    #     if(id in self.detection_of_interest):
    #         if(len(self.detection_of_interest[id])!=0 and self.detection_of_interest[id]['is_fetched']):
    #             self.detection_of_interest[id]['is_fetched']=False
    #             self.mutex.release()
    #             return self.detection_of_interest[id]['detection']
    #     self.mutex.release()
    #     return None

    # #set fields of interest to watch
    # def add_detection_of_interest(self,label_name):
        
    #     temp={}
    #     temp['is_fetched']=False
    #     temp['detection']=None #Detection2D object
    #     self.mutex.acquire()
    #     self.detection_of_interest[self.label_to_id[label_name]]=temp
    #     self.mutex.release()

    # def reset_detection_of_interest(self):
    #     self.mutex.acquire()
    #     self.detection_of_interest={}
    #     self.mutex.release()