#include <cmath>
#include <mutex>
#include <queue>
#include <stdlib.h>
#include <unordered_set>
#include <vector>

#include <boost/functional/hash.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

const char* const lidar_frame = "/lidar";
const char* const lidar_topic = "/scan/pointcloud";
const char* const camera_frame = "/optical_cam_center";
const char* const camera_topic = "/usb_cam_center/image_raw";
const char* const image_topic = "/scan/image";
const int min_gradient = 25;

std::mutex cloud_mutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
std::unique_ptr<tf::TransformListener> tf_listener;
image_transport::Publisher image_pub;

bool tf_seen = false;

void scanCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex);
  cloud->clear();

  tf::StampedTransform transform;
  if (!tf_seen)
  {
    tf_listener->waitForTransform(camera_frame, lidar_frame, ros::Time(0), ros::Duration(5));
    tf_seen = true;
  }
  tf_listener->lookupTransform(camera_frame, lidar_frame, ros::Time(0), transform);

  pcl_ros::transformPointCloud(*msg, *cloud, transform);
}

void barrelBlocker(cv::Mat& image, const cv::Point3d& cv_point, const cv::Point2d& cam_point)
{
  int xmin = 0;
  int xmax = image.cols;
  for (int i = cam_point.x; i > 0; --i)
  {
    auto cur_pixel = image.at<cv::Vec3b>(cam_point.y, i);
    auto next_pixel = image.at<cv::Vec3b>(cam_point.y, i - 1);
    int gradient = std::sqrt(std::pow(cur_pixel.val[0] - next_pixel.val[0], 2)
                           + std::pow(cur_pixel.val[1] - next_pixel.val[1], 2)
                           + std::pow(cur_pixel.val[2] - next_pixel.val[2], 2));
    if (gradient > min_gradient)
    {
      xmin = i;
      break;
    }
  }
  for (int i = cam_point.x; i < image.cols - 1; ++i)
  {
    auto cur_pixel = image.at<cv::Vec3b>(cam_point.y, i);
    auto next_pixel = image.at<cv::Vec3b>(cam_point.y, i + 1);
    int gradient = std::sqrt(std::pow(cur_pixel.val[0] - next_pixel.val[0], 2)
                           + std::pow(cur_pixel.val[1] - next_pixel.val[1], 2)
                           + std::pow(cur_pixel.val[2] - next_pixel.val[2], 2));
    if (gradient > min_gradient)
    {
      xmax = i;
      break;
    }
  }
  int ymin = cam_point.y - 0.5 * (xmax - xmin);
  int ymax = cam_point.y + (xmax - xmin);
  xmin -= 15;
  xmax += 15;
  if (xmin < 0) {
    xmin = 0;
    ymin = 0;
    ymax = image.rows - 1;
  }
  if (xmax >= image.cols) {
    xmax = image.cols - 1;
    ymin = 0;
    ymax = image.rows - 1;
  }
  if (ymin < 0) {
    ymin = 0;
  }
  if (ymax >= image.rows) {
    ymax = image.rows - 1;
  }
  cv::rectangle(image, cv::Point2d(xmin, ymin), cv::Point2d(xmax, ymax), cv::Scalar(0, 0, 0), CV_FILLED);
}

void cameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex);
  if (cloud->empty())
  {
    return;
  }

  cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(image_msg, "");

  cv::Mat image = input_bridge->image;
  int top_crop = image.rows / 2 + 50;
  cv::Rect ground_mask(0, top_crop, image.cols, image.rows - top_crop);
  cv::Mat cropped_image = image(ground_mask);

  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(info_msg);

  for (auto point : *cloud)
  {
    cv::Point3d cv_point(point.x, point.y, point.z);
    cv::Point2d cam_point = cam_model.project3dToPixel(cv_point);
    cam_point.y -= top_crop;
    if (cam_point.x >= 0 && cam_point.y >= 0 && cam_point.x < image.cols && cam_point.y < image.rows)
    {
      barrelBlocker(cropped_image, cv_point, cam_point);
    }
  }
  input_bridge->image = cropped_image;
  image_pub.publish(input_bridge->toImageMsg());
}

int main(int argc, char** argv)
{
  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  ros::init(argc, argv, "scan_to_image");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  tf_listener.reset(new tf::TransformListener());

  ros::Subscriber scan_sub = nh.subscribe(lidar_topic, 1, scanCallback);
  image_transport::CameraSubscriber cam_sub = it.subscribeCamera(camera_topic, 1, cameraCallback);

  image_pub = it.advertise(image_topic, 1);

  ros::spin();
}
