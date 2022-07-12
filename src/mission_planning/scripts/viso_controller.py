import math

import numpy
import rospy
import threading
from vision_msgs.msg import Detection2DArray, BoundingBox2D
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseArray, Pose, Transform, Vector3
import tf2_ros
import numpy as np
from numpy.linalg import norm
import movement_controller as mc
import tf2_geometry_msgs
import tf
from object_detection.utils import label_map_util

from transforms3d import euler
from geometry_msgs.msg import PoseArray, Pose, Transform, PointStamped, PoseStamped
import numpy as np
import message_filters
from copy import deepcopy

# class det_obj():
#     def __init__(self,):
        

class viso_controller():
    sleep_delay=0.05
    def __init__(self,world_frame,camera_frame,detection_topic,pcl_topic,detection_label_path,oakd_Hfov,width,height): #should be passed in radians
        aspect_ratio = width/height
        self.world_frame=world_frame
        self.camera_frame=camera_frame
        self.oakd_Hfov=oakd_Hfov
        oakd_Vfov= 2 * math.atan(math.tan(oakd_Hfov/2)*aspect_ratio)
        self.width=width
        self.height=height

        self.pcl_sub= message_filters.Subscriber(pcl_topic, PointCloud2)
        self.detection_sub = message_filters.Subscriber(detection_topic, Detection2DArray)

        ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub, self.pcl_sub], 3,slop=0.2)
        ts.registerCallback(self.pcl_detection_callback)

        self.label_to_id=label_map_util.get_label_map_dict(detection_label_path) #dictionary mapping label -> id
        self.mutex = threading.Lock()

        self.is_fetched=False

        self.pointcloud=PointCloud2()

        self.viso_detect=Detection2DArray()
        

    def get_id_from_label(self,label): #lookup label_to_id dictionary and return label for a specific id
        return self.label_to_id[label]

    def get_label_from_id(self, id):
        return self.label_to_id.keys()[self.label_to_id.values().index(id)]

    def detection_callback(self,viso_detect):
        self.mutex.acquire()
        self.viso_detect=viso_detect
        self.is_fetched=True
        self.mutex.release()

    def get_detection(self, array_of_labels): #np array of labels passed, returns dictionary of detections for those id's
        if(self.is_fetched):
            if(array_of_labels.size >=0):
                self.mutex.acquire()
                detections=self.viso_detect.detections
                det_ids={} #{id1 : [Detection2D,], id2}
                req_ids=[self.get_id_from_label(x) for x in array_of_labels]

                for det in list(detections):
                    if det.results.id in req_ids:
                        if det.results.id in det_ids:
                            det_ids[self.get_label_from_id(det.results.id)].append(det)
                        else:
                            det_ids[self.get_label_from_id(det.results.id)]=[det]
                self.is_fetched=False
                self.mutex.release()
                return det_ids
            self.is_fetched=False

            # list_new=list.copy(detections)
            # for det in list(list_new): #this will remove ids we are not interested in
            #     id=det.results.id
            #     if(id in ids):
            #         pass
            #     else:
            #         list_new.remove(det)   
            
            # #unordered list 

            # self.mutex.release()
            # return list_new 

            #{id:Detection2DArray,id:}
        else:
            self.mutex.release()
            return None
    #UNTESTED
    def get_angles(self,pixel_x,pixel_y):
        L=self.width/(2*math.tan(self.oakd_Hfov/2))
        angle_x=math.atan(abs((self.width-pixel_x))/L)
        angle_y=math.atan(abs((self.height-pixel_y))/L)
        return angle_x, angle_y

    def pcl_detection_callback(self,viso_detect,pointcloud):
        self.detection_callback(viso_detect)
        self.pointcloud=pointcloud

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

    def get_position_of_pixel(self,x_coord,y_coord): #get xyz of pixel from x,y coord in ros img frame
        pass
    #Courtesy of https://github.com/paul-shuvo/planar_pose/blob/main/src/planar_pose_estimation.py
    def estimate_pose(
                self,
                object_name,
                bbox,#: BoundingBox2D, #change this to bbox object and decreaese its size
            ):
        """
        Estimates planar pose of detected objects and
        updates the stored pose.
        Parameters
        ----------
        object_name: str
            Name of the object.
        bbox : bbox object
            Contains the coordinates of the bounding box
            of the detected object.
        pc_sub : PointCloud2
            A pointcloud object containing the 3D locations
            in terms of the frame `self.frame_id`
        """
        x_center=bbox.center.x
        y_center=bbox.center.y
        
        #decrease the bbox size by half because we might have some areas not in the detection if at an angle or slop
        size_x=bbox.size_x//2
        size_y=bbox.size_y//2

        bbox = np.array()

        # Compute the center, the mid point of the right
        # and top segment of the bounding box

        #get c,x,y
        c=[x_center,y_center]
        x=[math.floor(x_center+size_x*0.5),y_center]
        y=[x_center,math.floor(y_center-size_y*0.5)]

        points_region = np.mgrid[math.floor(x_center-size_x*0.5):x[0], y[1]:math.floor(y_center-size_y*0.5):1].reshape(2,-1).T
        

        points = np.array([c, x, y]).tolist()
        vectors_3D = np.zeros((3, 3))

        # pcl_region = pc2.read_points(
        #                 pc_sub,
        #                 field_names={'x', 'y', 'z'},
        #                 skip_nans=False, uvs=points_region.tolist() # [u,v u,v u,v] a set of xy coords
        #             )
        # # [[1,2,3],[,,], ..., dt]

        # norm = np.linalg.svd(pcl_region.T - np.mean(pcl_region.T, keepdims=True))[:,-1] #normal direction vector relative to object plane
        # center = np.mean(pcl_region,axis=0)
        # norm_v=Vector3(norm[0],norm[1],norm[2])
        
        # t = mc.mov_control.tfBuffer.lookup_transform(self.world_frame, self.camera_frame, rospy.Time()) #make this oakd_frame


        # norm_world = tf2_geometry_msgs.do_transform_vector3(norm_v, t.inverse()) #normal in the world_frame VECTOR3
        # norm_world.z=0

        try:
            # Get the corresponding 3D location of c, x, y
            for pt_count, dt in enumerate(
                pc2.read_points(
                        self.pointcloud,
                        field_names={'x', 'y', 'z'},
                        skip_nans=False, uvs=points # [u,v u,v u,v] a set of xy coords
                    )
                ):
                # # If any point returns nan, return
                if np.any(np.isnan(dt)):
                    if object_name in self.object_pose_info.keys():
                        del self.object_pose_info[object_name]
                    rospy.loginfo('No corresponding 3D point found')
                    return
                else:
                    vectors_3D[pt_count] = dt
                    if pt_count == 2:
                        self.vectors_3D = vectors_3D
        except:
            #rospy.loginfo(err)
            print("HORRIBLE ERROR REACHED IN PCL")
            return

        try:
            # 3D position of the object
            c_3D = self.vectors_3D[0]

            # Center the vectors to the origin
            x_vec = self.vectors_3D[1] - c_3D
            x_vec /= norm(x_vec)

            y_vec = self.vectors_3D[2] - c_3D
            y_vec /= norm(y_vec)
            # Take the cross product of x and y vector
            # to generate z vector.
            z_vec = np.cross(x_vec, y_vec)
            z_vec /= norm(z_vec)

            # Recompute x vector to make it truly orthognal
            x_vec_orth = np.cross(y_vec, z_vec)
            x_vec_orth /= norm(x_vec_orth)
        except RuntimeWarning as w:
            rospy.loginfo(w)
            return

        if self.viz_pose:
            self.draw_pose(object_name, np.vstack((self.vectors_3D, z_vec)))

        # Compute Euler angles i.e. roll, pitch, yaw
        roll = np.arctan2(y_vec[2], z_vec[2])
        pitch = np.arctan2(-x_vec_orth[2], np.sqrt(1 - x_vec_orth[2]**2))
        # pitch = np.arcsin(-x_vec_orth[2])
        yaw = np.arctan2(x_vec_orth[1], x_vec_orth[0])

        [qx, qy, qz, qw] = self.euler_to_quaternion(roll, pitch, yaw)

        # Generate Pose message.
        pose_msg = Pose()

        pose_msg.position.x = c_3D[0]
        pose_msg.position.y = c_3D[1]
        pose_msg.position.z = c_3D[2]
        # Make sure the quaternion is valid and normalized
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw

        # self.object_pose_info[object_name] = {
        #         'position': c_3D.tolist(),
        #         'orientation': [qx, qy, qz, qw]
        #     }

        return pose_msg

    def recursive_pcl(pc_sub, point, x_axis): #fix along x_axis
        # Get the corresponding 3D location of c, x, y
        for dt in enumerate(
            pc2.read_points(
                    pc_sub,
                    field_names={'x', 'y', 'z'},
                    skip_nans=False, uvs=point # [u,v u,v u,v] a set of xy coords
                )
            ):
            # If any point returns nan, return
            if np.any(np.isnan(dt)):
                if(x_axis):
                    point[0]=point[0]-1
                else:
                    point[1]=point[1]-1

                return (viso_controller.recursive_pcl(pc_sub,point,x_axis))
            else:
                return dt
    
    def np_to_point(numpy_array):
        ros_point = PointStamped()
        ros_point.point.x = numpy_array[0]
        ros_point.point.y = numpy_array[1]
        ros_point.point.z = numpy_array[2]
        return ros_point
    # def estimate_gate_pose(self,bbox1,bbox2,pc_sub):
    #     vectors_3D = np.zeros((3, 2))
    #     for pt_count, dt in enumerate(pc2.read_points(
    #                         pc_sub,
    #                         field_names={'x', 'y', 'z'},
    #                         skip_nans=False, uvs=[[bbox1.center.x,bbox1.center.x],[bbox2.center.y,bbox2.center.y]] # [u,v u,v u,v] a set of xy coords
    #                     )):
    #                 vectors_3D[pt_count]=dt
        
    #     transform=mc.mov_control.tfBuffer.lookup_transform(mc.mov_control.world_frame, self.camera_frame, rospy.Time(), rospy.Duration(1))
        
    #     #get 3D midpoint between both points in PCL frame
    #     midpoint=[(vectors_3D[0]+vectors_3D[1])/2]
        
    #     vector_w=vectors_3D[1]-vectors_3D[0]
        
    #     midpoint=self.np_to_point(midpoint)
    #     vector_w=self.np_to_point(vector_w)

    #     #get points in world frame
    #     midpoint=tf2_geometry_msgs.do_transform_point(midpoint,transform)
    #     vector_w=tf2_geometry_msgs.do_transform_point(vector_w,transform)

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
    

# Global variable courtesy of:
# https://stackoverflow.com/a/13034908
def init(world_frame,camera_frame,detection_topic,pcl_topic,detection_label_path,oakd_Hfov,width,height):
    global viso_control
    viso_control=viso_controller(world_frame,camera_frame,detection_topic,pcl_topic,detection_label_path,oakd_Hfov,width,height)
