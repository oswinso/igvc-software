#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Subscriber nnSub;
ros::Subscriber lidarSub;
ros::Publisher octopus;
ros::Subscriber cephalophile; 

void lineCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input)
{

	octopus.publish(input);
}

void lidarCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input)
{
	octopus.publish(input);
}

void mapCallback(nav_msgs::OccupancyGrid::Ptr input)
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "o_grid_mapper");
  ros::NodeHandle nh;
  nnSub = nh.subscribe("/semantic_segmentation", 1, lineCallback);
  lidarSub = nh.subscribe("/scan/pointcloud", 1, lidarCallback);
  octopus = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/cloud_in", 1);
  cephalophile = nh.subscribe("/projected_map", 1, mapCallback);

  ros::spin();
}