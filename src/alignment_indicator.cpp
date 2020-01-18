#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>

#include <stdio.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr refCloudPTR(new pcl::PointCloud<pcl::PointXYZ>);

void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
  ROS_INFO("Cloud: width = %d, height = %d\n", cloud_in->width, cloud_in->height);

  pcl::PCLPointCloud2 in_pcl_pc2;
  pcl_conversions::toPCL(*cloud_in, in_pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ> in_point_cloud;
  pcl::fromPCLPointCloud2(in_pcl_pc2, in_point_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
  *cloudPTR = in_point_cloud;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  icp.setInputSource(cloudPTR);
  icp.setInputTarget(refCloudPTR);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  // ROS_INFO(""has converged:" %d, height = %d\n", cloud_in->width, cloud_in->height);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  sensor_msgs::PointCloud2ConstPtr referencePcPtr;
  referencePcPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/reference_pc", nh);
  if (referencePcPtr == NULL)
  {
    ROS_ERROR("No reference point clound message received");
  }
  else
  {
    pcl::PCLPointCloud2 reference_pcl_pc2;
    pcl_conversions::toPCL(*referencePcPtr, reference_pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> reference_point_cloud;
    pcl::fromPCLPointCloud2(reference_pcl_pc2, reference_point_cloud);
    *refCloudPTR = reference_point_cloud;

    ros::Subscriber sub = nh.subscribe("points2", 1, callback);
    ros::spin();
  }
}
