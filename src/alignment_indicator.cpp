#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// typedef pcl::PointCloud2<pcl::PointXYZ> PointCloud2;

// void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

// ROS_INFO("%s %i %d", action_name_.c_str(), msg->height);
  ROS_INFO("Cloud: width = %d, height = %d\n", cloud_in->width, cloud_in->height);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_in, pcl_pc2);

  ////
  // pcl::PCLPointCloud2 point_cloud2;
  // pcl::PointCloud<pcl::PointXYZ> point_cloud;

  // pcl::fromPCLPointCloud2( *cloud_in, point_cloud);

  // pcl::toPCLPointCloud2(point_cloud, point_cloud2);
  ///
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("points2", 1, callback);
  ros::spin();
}


/////////////////////////////


// #include <pcl/io/pcd_io.h>


//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

//   // Fill in the CloudIn data
//   cloud_in->width    = 5;
//   cloud_in->height   = 1;
//   cloud_in->is_dense = false;
//   cloud_in->points.resize (cloud_in->width * cloud_in->height);
//   for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
//   {
//     cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//     cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//     cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//   }
//   std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
//       << std::endl;
//   for (std::size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
//       cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
//       cloud_in->points[i].z << std::endl;
//   *cloud_out = *cloud_in;

//   std::cout << "size:" << cloud_out->points.size() << std::endl;
//   for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
//     cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
//   std::cout << "Transformed " << cloud_in->points.size () << " data points:"
//       << std::endl;
//   for (std::size_t i = 0; i < cloud_out->points.size (); ++i)
//     std::cout << "    " << cloud_out->points[i].x << " " <<
//       cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource(cloud_in);
//   icp.setInputTarget(cloud_out);
//   pcl::PointCloud<pcl::PointXYZ> Final;
//   icp.align(Final);
//   std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//   icp.getFitnessScore() << std::endl;
//   std::cout << icp.getFinalTransformation() << std::endl;
