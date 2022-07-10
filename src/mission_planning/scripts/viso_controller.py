import math
import rospy
import threading
from vision_msgs.msg import Detection2DArray

from object_detection.utils import label_map_util

class viso_controller():
    #oakd_fov=
    #oak1_mxid=""
    sleep_delay=0.05
    def __init__(self,detection_topic,detection_label_path,oakd_Hfov,width,height): #should be passed in radians
        aspect_ratio = width/height
        self.oakd_Hfov=oakd_Hfov
        oakd_Vfov= 2 * math.atan(math.tan(oakd_Hfov/2)*aspect_ratio)
        self.width=width
        self.height=height

        self.detection_subscriber = rospy.Subscriber(detection_topic, Detection2DArray , self.detection_callback)
        self.label_to_id=label_map_util.get_label_map_dict(detection_label_path) #dictionary mapping label -> id
        self.mutex = threading.Lock()

        self.detection_of_interest={}
        # {
        #   1 : {is_fetched:False,"detection": Detection2D}
        # }

        self.viso_detect=Detection2DArray()
        #50.41 VFOV 
        #calibData = dai_device.readCalibration()
         #print(f"RGB FOV {calibData.getFov(dai.CameraBoardSocket.RGB)}, Mono FOV {calibData.getFov(dai.CameraBoardSocket.LEFT)}")

    #UNTESTED
    def get_angles(self,pixel_x,pixel_y):
        L=self.width/(2*math.tan(self.oakd_Hfov/2))
        angle_x=math.atan(abs((self.width-pixel_x))/L)
        angle_y=math.atan(abs((self.height-pixel_y))/L)
        return angle_x, angle_y

    def detection_callback(self,viso_detect):
        #label = viso_detect.detections.results.id
        #any(det in self.detection_of_interest for det in viso_detect.detections)
        for det in viso_detect.detections: #make 
            #print("FOUND INDEX")
            #check if the region is in array
            if(det.results[0].id in self.detection_of_interest): #concerned about data types here LIST OBJECT HAS NO ATTRIBUTE ID
                #print("in the dict ID: ") #THIS WORKS
                self.mutex.acquire()
                self.detection_of_interest[det.results[0].id]['is_fetched']=True
                self.detection_of_interest[det.results[0].id]['detection']=det
                self.mutex.release()

                #update the values
                #self.viso_detect=viso_detect

        #update values of detections

    # def get_detection(self): #returns Detection2D if label is found, false if otherwise
    #     #mutex lock here
    #     self.mutex.acquire()
    #     if(self.is_detected):
    #         self.is_detected=False
    #         return self.viso_detect.detections[self.detect_index], True #only return one of interest
    #     self.mutex.release()

    #     return None, False
        
    # def get_detection(self,label): #returns Detection2D if label is found, false if otherwise
    #     #mutex lock here
    #     print("GET DETECTION CALLED")
    #     self.mutex.acquire()
    #     id=self.label_to_id[label]
    #     if(id in self.detection_of_interest):
    #         if(len(self.detection_of_interest[id])!=0):
    #             print("inRETURNING DICT dict")
    #             self.detection_of_interest[id]['is_fetched']=False
    #             self.mutex.release()
    #             return self.detection_of_interest[id]['detection'],True
    #     else:
    #         self.mutex.release()
    #         return {"filler":"filler"}, False

    def get_detection(self,label): #returns Detection2D if label is found, false if otherwise
        #mutex lock here
        #print("GET DETECTION CALLED")
        self.mutex.acquire()
        id=self.label_to_id[label]
        if(id in self.detection_of_interest):
            if(len(self.detection_of_interest[id])!=0):
                #print("inRETURNING DICT dict")
                self.detection_of_interest[id]['is_fetched']=False
                self.mutex.release()
                return self.detection_of_interest[id]['detection']
        else:
            self.mutex.release()
            return None
        

    def get_id_from_label(self,label): #lookup label_to_id dictionary and return label for a specific id
        self.label_to_id[label]
        pass

    #set fields of interest to watch
    def add_detection_of_interest(self,label_name):
        temp={}
        temp['is_fetched']=False
        temp['detection']=None #Detection2D object
        self.mutex.acquire()
        self.detection_of_interest[self.label_to_id[label_name]]=temp
        self.mutex.release()

    def reset_detection_of_interest(self):
        self.mutex.acquire()
        self.detection_of_interest={}
        self.mutex.release()

    def get_position_of_pixel(self,x_coord,y_coord): #get xyz of pixel from x,y coord in ros img frame
        pass

    
        
    

# Global variable courtesy of:
# https://stackoverflow.com/a/13034908
def init(detection_topic,detection_label_path,oakd_Hfov,width,height):
    global viso_control
    viso_control=viso_controller(detection_topic,detection_label_path,oakd_Hfov,width,height)
