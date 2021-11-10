#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

#include <pcl/filters/crop_box.h>


class BoxFilter{
  public:
    BoxFilter(ros::NodeHandle nh) : nh_(nh), min_(Eigen::Vector4f{}), max_(Eigen::Vector4f{}){
      // Create a ROS subscriber for the input point cloud
      sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/octomap_point_cloud_centers", 1, &BoxFilter::cloud_cb, this);
      sub_min_ = nh_.subscribe<geometry_msgs::Point> ("/box_filter/min", 1, &BoxFilter::min_cb, this);
      sub_max_ = nh_.subscribe<geometry_msgs::Point> ("/box_filter/max", 1, &BoxFilter::max_cb, this);

      // Create a ROS publisher for the output point cloud
      pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/output_filtered", 1);
    };

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Subscriber sub_max_;
    ros::Subscriber sub_min_;

    Eigen::Vector4f min_;
    Eigen::Vector4f max_;

    void min_cb (const geometry_msgs::PointConstPtr& min) {
      ROS_INFO("callback successful");
      min_ = Eigen::Vector4f(min->x, min->y, min->z, 1.0);
    };

    void max_cb (const geometry_msgs::PointConstPtr& max) {
      ROS_INFO("callback successful");
      max_ = Eigen::Vector4f(max->x, max->y, max->z, 1.0);
    };

    void
    cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      // Container for original & filtered data
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      pcl::PCLPointCloud2 cloud_filtered;

      // Convert to PCL data type
      pcl_conversions::toPCL(*cloud_msg, *cloud);

      // Perform the actual filtering
      pcl::CropBox<pcl::PCLPointCloud2> cbox;
      cbox.setMin (min_);
      cbox.setMax (max_);
      cbox.setInputCloud(cloudPtr);
      cbox.filter (cloud_filtered);

      // Convert to ROS data type
      sensor_msgs::PointCloud2 output;
      pcl_conversions::moveFromPCL(cloud_filtered, output);

      // Publish the data
      pub_.publish (output);
    };

};

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh("~");

  auto filter_node = BoxFilter(nh);
  ROS_INFO("Ready to convert incomming pcl to open3d form.");

  // Spin
  ros::spin ();
  ROS_INFO("Node Died");
}