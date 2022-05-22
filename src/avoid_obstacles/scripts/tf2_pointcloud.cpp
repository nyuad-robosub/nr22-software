// Point cloud transform with tf2 courtesy of:
// https://github.com/stereolabs/zed-ros-wrapper/issues/393#issuecomment-478958782
// https://answers.ros.org/question/262179/transform-pointcloud-with-tf2/

#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

// TF2 includes
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

// Global transformStamped
geometry_msgs::TransformStamped transformStamped;

// Global publisher
ros::Publisher pub;

// Callback for cloud listener
void cloud_callback (const sensor_msgs::PointCloud2& cloud_in) {
    std::cout << transformStamped << std::endl;

    // Do transform
    sensor_msgs::PointCloud2 cloud_out;
    tf2::doTransform(cloud_in, cloud_out, transformStamped);

    // Publish cloud
    cloud_out.header = cloud_in.header;
    cloud_out.header.frame_id = "world";
    pub.publish(cloud_out);
}


int main(int argc, char** argv) {
    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "point_cloud_transform");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud/cloud_transformed", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // sensor_msgs::PointCloud2 cloud_in, cloud_out;

    ros::Rate rate(30.0);
    while (nh.ok()){
        try{
            transformStamped = tfBuffer.lookupTransform("world", "camera_depth_optical_frame",
                ros::Time(0));
        }
        catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
            // ros::Duration(1.0).sleep();
            continue;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}