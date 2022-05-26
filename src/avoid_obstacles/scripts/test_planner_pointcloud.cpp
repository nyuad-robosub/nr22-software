// Point cloud transform courtesy of:
// https://github.com/stereolabs/zed-ros-wrapper/issues/393#issuecomment-478958782

#include <ros/ros.h>
#include <ros/console.h>

// Marker display courtesy of:
// http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
#include <visualization_msgs/Marker.h>

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

// User includes
#define USE_PCL
#define VISUALIZE_TRANSFORMED_POINTCLOUD
#include "lazytheta.cpp"
// #include <vector>

// Global publisher
ros::Publisher pub, pub2;
tf::StampedTransform st_transform;

// Add frame names
static std::string REF_FRAME = "world"; // "world";
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
    pcl_ros::transformPointCloud(*temp1_cloud, *temp2_cloud, st_transform);

    // Downsample filter for voxels: 10cm voxels
    pcl::VoxelGrid<pcl::PointXYZ> vg2;
    vg2.setInputCloud (temp2_cloud);
    vg2.setLeafSize (0.1f, 0.1f, 0.1f);
    vg2.filter (*temp1_cloud);
    // ROS_INFO("okay 2");

    // TRAJECTORY CALCULATION
    // Compute path & populate the trajectory
    visualization_msgs::Marker path_strip;
    path_strip.header.frame_id = REF_FRAME;
    path_strip.header.stamp = ros::Time::now();
    path_strip.ns = "path_strip";
    path_strip.action = visualization_msgs::Marker::ADD;
    path_strip.pose.orientation.w = 1.0;
    path_strip.id = 0;
    path_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // Point markers use x and y scale for width/height respectively
    path_strip.scale.x = 0.1;
    path_strip.scale.y = 0.1;
    path_strip.scale.z = 0.1;

    // Path is red
    path_strip.color.r = 1.0f;
    path_strip.color.a = 0.5;
    
    Eigen::Vector3i old_xyz, start(-100, 0, -520), end(100, 0, -550);
    static LazyTheta lt;
    lt.UpdateOccupancyPCL(*temp1_cloud, true, false);
    if (lt.ComputePath(start, end)) {
        Node& s = lt.s_end;
        // cout << "path " << s.xyz(0) << " " << s.xyz(1) << " " << s.xyz(2) << " " << endl;
        // cout << "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << endl;
        geometry_msgs::Point p;
        p.x = s.xyz(0) * 0.1; p.y = s.xyz(1) * 0.1; p.z = s.xyz(2) * 0.1;
        path_strip.points.push_back(p);
        std::string msg = "(" + std::to_string(s.xyz(0)) + ", " + std::to_string(s.xyz(1)) + ", " + std::to_string(s.xyz(2)) + "),";
        ROS_INFO(msg.c_str());
        while (s.xyz != start) {
            old_xyz = s.xyz;
            s = *s.parent;
            // if (!lt.LineOfSight(old_xyz, s.xyz)) {
            //     std::cout << "ERR ::: "
            //     << "(" << old_xyz(0) << ", " << old_xyz(1) << ", " << old_xyz(2) << "), "
            //     << "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << std::endl;
            // }
            // cout << "(" << s.xyz(0) << ", " << s.xyz(1) << ", " << s.xyz(2) << "), " << endl;
            p.x = s.xyz(0) * 0.1; p.y = s.xyz(1) * 0.1; p.z = s.xyz(2) * 0.1;
            path_strip.points.push_back(p);
            msg = "(" + std::to_string(s.xyz(0)) + ", " + std::to_string(s.xyz(1)) + ", " + std::to_string(s.xyz(2)) + "),";
            ROS_INFO(msg.c_str());
        }
    }
    pub2.publish(path_strip);
    // std::cout << "\n\n\n\n\n\n\n";

#ifdef VISUALIZE_TRANSFORMED_POINTCLOUD
    // Iterate through the pointcloud courtesy of:
    // https://www.cnblogs.com/linweilin/p/11330677.html
    temp2_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>{}); // Can be created through new but unnecessary
    for (auto obs: lt.obstaclesXYZ) {
        temp2_cloud->push_back(pcl::PointXYZ(obs(0) * 0.1, obs(1) * 0.1, obs(2) * 0.1));
    }
    // for (pcl::PointCloud<pcl::PointXYZ>::iterator it = temp1_cloud->begin(); it != temp1_cloud->end(); it++)
    // {
    //     // Return an Eigen::Vector3f of points coordinate.
    //     // Eigen::Vector3f tmp = it->getVector3fMap(); // if double wanted, just say it->getVector3fMap().cast<double>
    //     // std::cout << "(" << tmp(0) << ", " << tmp(1) << ", " << tmp(2) << "), ";
    //     temp2_cloud->push_back(pcl::PointXYZ(round(it->x * 10) / 10, round(it->y * 10) / 10, round(it->z * 10) / 10));
    // }

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg (*temp2_cloud, cloud_publish);
    cloud_publish.header = input->header;
    cloud_publish.header.frame_id = REF_FRAME;
    cloud_publish.header.stamp = ros::Time::now();

    pub.publish(cloud_publish);
#endif
}

int main (int argc, char** argv){

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init (argc, argv, "point_cloud_transform");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_callback);

#ifdef VISUALIZE_TRANSFORMED_POINTCLOUD
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud/cloud_transformed", 1);
#endif

    // Create a ROS publisher for the output trajectory
    pub2 = nh.advertise<visualization_msgs::Marker>("output_trajectory", 2);

    tf::TransformListener listener;
    // Wait for transform 
    // https://answers.ros.org/question/203274/frame-passed-to-lookuptransform-does-not-exist/?answer=203281#post-id-203281
    listener.waitForTransform(REF_FRAME, CHILD_FRAME, ros::Time(), ros::Duration(4.0));
    ros::Rate rate(30.0);
    while (nh.ok()){

        try{
            listener.lookupTransform(REF_FRAME, CHILD_FRAME, ros::Time(), st_transform);
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