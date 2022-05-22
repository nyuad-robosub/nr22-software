#!/usr/bin/env python
# Lucas Walter
# transform a pointcloud into a new frame

import copy
from threading import Lock

import rospy
import tf2_ros
import tf2_py as tf2

from dynamic_reconfigure.server import Server
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# from transform_point_cloud.cfg import LookupTransformConfig
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("enable", bool_t, 0, "enable the transform", True)
gen.add("update_rate", double_t, 0, "publish at regular rate instead of in callback (0.0)", 0.0, 0.0, 100.0)
gen.add("target_frame", str_t, 0,
        "First parameter in lookupTransform(), the new frame_id of the output point cloud, set blank to use the input point cloud frame_id",
        "map")
gen.add("source_frame", str_t, 0,
        "Second parameter in lookupTransform(), leave blank to use the input point cloud frame_id, which will leave the output cloud coordinates the same in world coordinates/as viewed in rviz",
        "")
gen.add("timeout", double_t,    0, "lookup timeout",  1.0, 0.0, 10.0)
gen.add("offset_lookup_time", double_t,    0, "Offset the lookup from point cloud time",  0, -10.0, 10.0)
# gen.add("offset_output_time", double_t,    0, "Offset the time from the lookup transform results",  0, -10.0, 10.0)
# TODO(lwalter) output frame_id - override the frame id on the output point cloud
gen.add("scale_x", double_t, 2, "Scale the x of each point", 1.0, -10000.0, 10000.0)
gen.add("scale_y", double_t, 2, "Scale the y of each point", 1.0, -10000.0, 10000.0)
gen.add("scale_z", double_t, 2, "Scale the z of each point", 1.0, -10000.0, 10000.0)
# gen.add("offset_output_time", double_t,    0, "Offset the time from the lookup transform results",  0, -10.0, 10.0)

gen.generate("transform_point_cloud", "transform_point_cloud", "LookupTransform")



class TransformPointCloud:
    def __init__(self):
        self.lock = Lock()
        self.config = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("point_cloud_transformed", PointCloud2, queue_size=2)
        self.dr_server = Server(gen, self.dr_callback)
        self.cloud_in = None
        self.sub = rospy.Subscriber("point_cloud", PointCloud2,
                                    self.point_cloud_callback, queue_size=2)

    def dr_callback(self, config, level):
        with self.lock:
            self.config = copy.deepcopy(config)
        return config

    def point_cloud_callback(self, msg):
        with self.lock:
            config = copy.deepcopy(self.config)

        self.publish_point_cloud(config, msg, msg.header.stamp)

        with self.lock:
            self.cloud_in = msg

    def publish_point_cloud(self, config, cloud_in, stamp):
        lookup_time = stamp + rospy.Duration(config.offset_lookup_time)
        target_frame = cloud_in.header.frame_id if config.target_frame == "" else config.target_frame
        source_frame = cloud_in.header.frame_id if config.source_frame == "" else config.source_frame
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time,
                                                    rospy.Duration(config.timeout))
        except tf2.LookupException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(cloud_in, trans)
        self.pub.publish(cloud_out)


if __name__ == '__main__':
    rospy.init_node('transform_point_cloud')
    transform_point_cloud = TransformPointCloud()
    rospy.spin()