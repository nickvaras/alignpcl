#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

class AlignmentIndicator
{
protected:
  ros::NodeHandle nh_;

  ros::Publisher alignment_publisher_;
  ros::Subscriber cloud_subscriber_;

  sensor_msgs::PointCloud2ConstPtr referencePcPtr_;
  float translation_tolerance_, rotation_tolerance_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_ptr_;

public:
  AlignmentIndicator() : reference_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    ros::NodeHandle pnh_("~");
    pnh_.param<float>("translation_tolerance", translation_tolerance_, 0.2);
    pnh_.param<float>("rotation_tolerance", rotation_tolerance_, 0.5);

    referencePcPtr_ = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/reference_pc", nh_);

    if (referencePcPtr_ == NULL)
    {
      ROS_ERROR("No reference point clound message received");
    }
    else
    {
      pcl::PCLPointCloud2 reference_pcl_pc2;
      pcl_conversions::toPCL(*referencePcPtr_, reference_pcl_pc2);
      pcl::PointCloud<pcl::PointXYZ> reference_point_cloud;
      pcl::fromPCLPointCloud2(reference_pcl_pc2, reference_point_cloud);
      *reference_cloud_ptr_ = reference_point_cloud;
      alignment_publisher_  = nh_.advertise<geometry_msgs::Transform>("reflectors_transform", 1);
      cloud_subscriber_     = nh_.subscribe("points2", 1, &AlignmentIndicator::cloud_callback, this);
    }
  }

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
  {
    pcl::PCLPointCloud2 in_pcl_pc2;
    pcl_conversions::toPCL(*cloud_in, in_pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> in_point_cloud;
    pcl::fromPCLPointCloud2(in_pcl_pc2, in_point_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPTR = in_point_cloud;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudPTR);
    icp.setInputTarget(reference_cloud_ptr_);
    // icp.setEuclideanFitnessEpsilon(0.5);
    // icp.setMaxCorrespondenceDistance()


    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    // icp.setMaxCorrespondenceDistance (0.15);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (30);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.01);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    if (icp.hasConverged())
    {
      Eigen::Matrix4f final_transform_matrix = icp.getFinalTransformation();
      if (fabs(final_transform_matrix(0, 3)) < translation_tolerance_ && fabs(final_transform_matrix(1, 3)) < translation_tolerance_ && fabs(asin(final_transform_matrix(1, 0))) < rotation_tolerance_)
      {
        // ROS_INFO("yaw: %f \n", asin(final_transform_matrix(1, 0)));
        tf2::Quaternion rot_quat;
        rot_quat.setRPY(0, 0, asin(final_transform_matrix(1, 0)));
        geometry_msgs::Quaternion quat_msg;
        quat_msg = tf2::toMsg(rot_quat);

        geometry_msgs::Transform transform_msg;
        transform_msg.translation.x = final_transform_matrix(0, 3);
        transform_msg.translation.y = final_transform_matrix(1, 3);
        transform_msg.rotation = quat_msg;
        alignment_publisher_.publish(transform_msg);
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "alignment_indicator");
  ROS_INFO("alignment_indicator node started");
  AlignmentIndicator alignment_indicator;
  ros::spin();
  return 0;
}