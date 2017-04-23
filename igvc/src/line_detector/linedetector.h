#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "igvc/CVInclude.h"

#define KERNAL_COUNT 8

class LineDetector
{
public:
  LineDetector(ros::NodeHandle& handle, const std::string& topic);

private:
  cv::Mat src_img, working, fin_img;

  void info_img_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info);
  void img_callback(const cv::Mat msg, const sensor_msgs::ImageConstPtr& origMsg);

  void initLineDetection();

  void DetectLines(int lineThickness);
  void WhitenessFilter(cv::Mat& hsv_image, cv::Mat& result);
  void MultiplyByComplements(cv::Mat* images, cv::Mat* complements, cv::Mat* results);
  void EnforceLength(cv::Mat& img, int length, std::vector<std::vector<cv::Point>>& contoursThreshold);

  // ROS COMMUNICATION
  image_transport::ImageTransport _it;
  std::string topic;
  image_transport::Publisher _filt_img;
  image_transport::Subscriber _src_img;
  image_transport::CameraSubscriber _src_img_info;
  ros::Publisher _line_cloud;
  tf::TransformListener tf_listener;
  image_geometry::PinholeCameraModel cam;

  // Tuning parameters loaded from YAML file (file specified in launch file)
  int cannyThresh;
  int ratio;
  int houghThreshold;
  int houghMinLineLength;
  int houghMaxLineGap;
  int maxDistance;

  // line thickness in pixels
  int lineThickness;
  // line threshold to continue
  int lineLengthThreshold;

  const int linewidthpixels = 13;

  cv::Mat dst_img;
  cv::Mat kernels[KERNAL_COUNT];
  cv::Mat kernelComplements[KERNAL_COUNT];
  cv::Mat kernelResults[KERNAL_COUNT];
  cv::Mat complementResults[KERNAL_COUNT];
};

#endif  // LINEDETECTOR_H
