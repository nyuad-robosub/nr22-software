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

// Global publisher
ros::Publisher pub;
tf::StampedTransform transform;

// Callback for cloud listener
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>),
                                        temp1_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                        temp2_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                        temp3_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl_conversions::toPCL(*input, pcl_pc2);
    // pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    pcl::fromROSMsg (*input, *temp1_cloud);

    // Filter for stray points
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (temp1_cloud);
    sor.setMeanK (25);
    sor.setStddevMulThresh (1.0);
    sor.filter (*temp2_cloud);

    // Downsample filter for voxels: 5cm voxels
    pcl::VoxelGrid<pcl::PointXYZ> vg1;
    vg1.setInputCloud (temp2_cloud);
    vg1.setLeafSize (0.02f, 0.02f, 0.02f);
    vg1.filter (*temp3_cloud);

    // Transform the pointcloud
    pcl_ros::transformPointCloud(*temp3_cloud, *cloud_transformed, transform);

    // Downsample filter for voxels: 10cm voxels
    pcl::VoxelGrid<pcl::PointXYZ> vg2;
    vg2.setInputCloud (cloud_transformed);
    vg2.setLeafSize (0.1f, 0.1f, 0.1f);
    vg2.filter (*cloud_filtered);

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg (*cloud_filtered, cloud_publish);
    cloud_publish.header = input->header;

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
    listener.waitForTransform("/world", "/camera_depth_frame", ros::Time(), ros::Duration(4.0));
    ros::Rate rate(30.0);
    while (nh.ok()){

        try{
            listener.lookupTransform("/world", "/camera_depth_frame", ros::Time(), transform);
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