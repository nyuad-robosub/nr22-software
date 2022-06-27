// Point cloud transform courtesy of:
// https://github.com/stereolabs/zed-ros-wrapper/issues/393#issuecomment-478958782

#include <ros/ros.h>
#include <ros/console.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>

// Eigen includes
#include <eigen3/Eigen/Dense>

// Global publisher
ros::Publisher pub;
tf::StampedTransform transform;

// Add frame names
static std::string REF_FRAME = "camera_link"; // "world";
static std::string CHILD_FRAME = "camera_depth_optical_frame";

// Callback for cloud listener
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // if (listener_ptr == nullptr) return;
    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp1_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                        temp2_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg (*input, *temp1_cloud);

    // Downsample filter for voxels: 2cm voxels
    pcl::VoxelGrid<pcl::PointXYZ> vg1;
    vg1.setInputCloud (temp1_cloud);
    vg1.setLeafSize (0.05f, 0.05f, 0.05f);
    vg1.filter (*temp2_cloud);
    // ROS_INFO("okay 1");

    // Filter for stray points
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (temp2_cloud);
    sor.setMeanK (25);
    sor.setStddevMulThresh (0.1);
    sor.filter (*temp1_cloud);

    // Transform the pointcloud
    pcl_ros::transformPointCloud(*temp1_cloud, *temp2_cloud, transform);

    // Downsample filter for voxels: 10cm voxels
    pcl::VoxelGrid<pcl::PointXYZ> vg2;
    vg2.setInputCloud (temp2_cloud);
    vg2.setLeafSize (0.1f, 0.1f, 0.1f);
    vg2.filter (*temp1_cloud);
    // ROS_INFO("okay 2");

    // Iterate through the pointcloud courtesy of:
    // https://www.cnblogs.com/linweilin/p/11330677.html
    temp2_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>{}); // Can be created through new but unnecessary
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = temp1_cloud->begin(); it != temp1_cloud->end(); it++)
    {
        // Return an Eigen::Vector3f of points coordinate.
        // Eigen::Vector3f tmp = it->getVector3fMap(); // if double wanted, just say it->getVector3fMap().cast<double>
        // std::cout << "(" << tmp(0) << ", " << tmp(1) << ", " << tmp(2) << "), ";
        temp2_cloud->push_back(pcl::PointXYZ(round(it->x * 10) / 10, round(it->y * 10) / 10, round(it->z * 10) / 10));
    }
    // std::cout << "\n\n\n\n\n\n\n";

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg (*temp2_cloud, cloud_publish);
    cloud_publish.header = input->header;
    cloud_publish.header.frame_id = REF_FRAME;

    pub.publish(cloud_publish);
}

int main (int argc, char** argv){

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "point_cloud_transform");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud/cloud_transformed", 1);

    tf::TransformListener listener;
    // Wait for transform 
    // https://answers.ros.org/question/203274/frame-passed-to-lookuptransform-does-not-exist/?answer=203281#post-id-203281
    listener.waitForTransform(REF_FRAME, CHILD_FRAME, ros::Time(), ros::Duration(4.0));
    ros::Rate rate(30.0);
    while (nh.ok()){

        try{
            listener.lookupTransform(REF_FRAME, CHILD_FRAME, ros::Time(), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;

}