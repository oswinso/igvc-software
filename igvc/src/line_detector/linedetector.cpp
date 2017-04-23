#include "linedetector.h"
#include <igvc/CVUtils.hpp>

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

using namespace cv;
using namespace std;
using namespace pcl;

void LineDetector::info_img_callback(const sensor_msgs::ImageConstPtr& msg,
                                     const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  cv_bridge::CvImagePtr cv_copy;
  cv_copy = cv_bridge::toCvCopy(msg, "");
  cv::Mat img = cv_copy->image;

  img = ResizeCameraImage(img, 640, 360);
  sensor_msgs::CameraInfo cam_info_rsz = ResizeCameraInfo(cam_info, 640, 360);

  cam.fromCameraInfo(cam_info_rsz);
  img_callback(img, msg);
}

void LineDetector::img_callback(const cv::Mat msg_img, const sensor_msgs::ImageConstPtr& origMsg)
{
  src_img = msg_img;

  // Convert image to grayscale
  // cv::cvtColor(src_img, src_img, CV_BGR2GRAY);

  // // Crops the image (removes sky)
  // int topCrop = 2 * src_img.rows / 3;
  // cv::Rect roiNoSky(0, topCrop, src_img.cols, src_img.rows - topCrop);
  // src_img = src_img(roiNoSky);

  // Create blank canvas
  fin_img = cv::Mat::zeros(src_img.size(), src_img.type());

  DetectLines(lineThickness);
  // std::vector<std::vector<cv::Point>> contoursThreshold;
  // EnforceLength(working, lineLengthThreshold, contoursThreshold);

  // Gaussian Blur
  // cv::GaussianBlur(src_img, working, cv::Size(3, 3), 2.0);

  // // Detect edges
  // cv::Canny(working, working, cannyThresh, cannyThresh*ratio, 3);

  // // Erosion and dilation
  // int kernel_size = 3;
  // cv::Mat erosion_kernal = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(kernel_size, kernel_size));
  // cv::dilate(working, working, erosion_kernal);
  // cv::dilate(working, working, erosion_kernal);
  // cv::dilate(working, working, erosion_kernal);
  // cv::erode(working, working, erosion_kernal);
  // cv::erode(working, working, erosion_kernal);
  // cv::erode(working, working, erosion_kernal);

  // // Find lines
  // std::vector<cv::Vec4i> lines;
  // cv::HoughLinesP(working, lines, 1.0, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
  // for (size_t i = 0; i < lines.size(); ++i)
  // {
  //   cv::line(fin_img, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]),
  //            cv::Scalar(255, 255, 255), 1, 4);
  //   //std::cout<<(lines[i][0]-lines[i][2])<< " " <<(lines[i][1]-lines[i][3]) << std::endl;
  // }

  // Re-fill sky area of image with black
  //cv::Mat black = cv::Mat::zeros(cv::Size(src_img.cols, topCrop), src_img.type());
  //cv::vconcat(black, fin_img, fin_img);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  cloud = toPointCloud(tf_listener, MatToContours(working), cam, topic);

  // Limit how far the points are plotted in the cloud
  capPointCloud(cloud, maxDistance);

  _line_cloud.publish(cloud);

  cv::cvtColor(working, working, CV_GRAY2BGR);
  cv_bridge::CvImagePtr newPtr = cv_bridge::toCvCopy(origMsg, "");
  newPtr->image = working;
  _filt_img.publish(newPtr->toImageMsg());
}

LineDetector::LineDetector(ros::NodeHandle& handle, const std::string& topic)
  : _it(handle), topic(topic), tf_listener(handle)
{
  _src_img_info = _it.subscribeCamera(topic + "/image_raw", 1, &LineDetector::info_img_callback, this);
  _filt_img = _it.advertise(topic + "/filt_img", 1);
  _line_cloud = handle.advertise<PCLCloud>(topic + "/line_cloud", 100);

  handle.getParam(ros::this_node::getName() + "/config/line/cannyThresh", cannyThresh);
  handle.getParam(ros::this_node::getName() + "/config/line/ratio", ratio);
  handle.getParam(ros::this_node::getName() + "/config/line/houghThreshold", houghThreshold);
  handle.getParam(ros::this_node::getName() + "/config/line/houghMinLineLength", houghMinLineLength);
  handle.getParam(ros::this_node::getName() + "/config/line/houghMaxLineGap", houghMaxLineGap);
  handle.getParam(ros::this_node::getName() + "/config/line/maxDistance", maxDistance);
  initLineDetection();
}

void LineDetector::initLineDetection() {
    cerr << "DetectLines::Initing" << endl;
    lineThickness = 13;
    lineLengthThreshold = 25;

    float karray[3][9][9] = {
            {
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},
                    {1,     1,     1,     1,     1,     1,     1,     1,     1},
                    {1,     1,     1,     1,     1,     1,     1,     1,     1},    
                    {1,     1,     1,     1,     1,     1,     1,     1,     1},    
                    {0,     0,     0,     0,     0,     0,     0,     0,     0},    
                    {0,     0,     0,     0,     0,     0,     0,     0,     0},    
                    {0,     0,     0,     0,     0,     0,     0,     0,     0}
            },     
            {
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},    
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,     0,     1},    
                    {-1,   -1,    -1,    -1,    -1,     0,     1,     1,     1},    
                    {-1,   -1,    -1,     0,     1,     1,     1,     1,     1},    
                    {-1,    0,     1,     1,     1,     1,     1,    .5,     0},    
                    {1,     1,     1,     1,     1,    .5,     0,     0,     0},    
                    {1,     1,     1,    .5,     0,     0,     0,     0,     0},    
                    {1,    .5,     0,     0,     0,     0,     0,     0,     0},    
                    {0,     0,     0,     0,     0,     0,     0,     0,     0}
            },
            {
                    {-.89,-.89, -.89,  -.89,  -.89,  -.89,  -.89,     1,     1},    
                    {-.89,-.89, -.89,  -.89,  -.89,  -.89,     1,     1,     1},    
                    {-.89,-.89, -.89,  -.89,  -.89,     1,     1,     1,     0},    
                    {-.89,-.89, -.89,  -.89,     1,     1,     1,     0,     0},    
                    {-.89,-.89, -.89,     1,     1,     1,     0,     0,     0},    
                    {-.89,-.89,    1,     1,     1,     0,     0,     0,     0},    
                    {-.89,  1,     1,     1,     0,     0,     0,     0,     0},    
                    {1,     1,     1,     0,     0,     0,     0,     0,     0},    
                    {1,     1,     0,     0,     0,     0,     0,     0,     0}
            }
    };

    Mat kernel1(9, 9, CV_32FC1, karray[0]);
    kernel1 /= 27;
    Mat kernel2(9, 9, CV_32FC1, karray[1]);
    kernel2 /= 25;
    Mat kernel3(9, 9, CV_32FC1, karray[2]);
    kernel3 /= 25;

    Mat kernel4 = kernel2.t();
    Mat kernel5 = kernel1.t();

    Mat kernel6;
    Mat kernel7;
    Mat kernel8;

    flip(kernel4, kernel6, 0);
    flip(kernel3, kernel7, 0);
    flip(kernel2, kernel8, 0);

    kernels[0] = kernel1.clone();
    kernels[1] = kernel2.clone();
    kernels[2] = kernel3.clone();
    kernels[3] = kernel4.clone();
    kernels[4] = kernel5.clone();
    kernels[5] = kernel6.clone();
    kernels[6] = kernel7.clone();
    kernels[7] = kernel8.clone();

    for (int i = 0; i < KERNAL_COUNT; i++) {
        Mat kernelComplement;
        flip(kernels[i], kernelComplement, -1);
        kernelComplements[i] = kernelComplement.clone();
    }
}


void LineDetector::DetectLines(int lineThickness) {

    // Resize the image such that the lines are approximately 3 pixels wide
    //cerr << "DetectLines::Reducing Image" << endl;
    lineThickness = max(1, lineThickness); // 0 thickness doesn't make sense
    resize(src_img, working, Size(3*src_img.cols/lineThickness, 3*src_img.rows/lineThickness), 0, 0, CV_INTER_AREA);

    // Convert the image into HSV space to make processing white lines easier
    //cerr << "DetectLines::Converting to HSV" << endl;
    cvtColor(working, working, CV_BGR2HSV);

    // Calculate each pixel's "whiteness" defined as value*(255-saturation);
    //cerr << "DetectLines::Filtering Whiteness" << endl;
    WhitenessFilter(working, working);

    // Pass directional kernels over image
    //cerr << "DetectLines::Filtering kernels" << endl;
    for(size_t i = 0; i < KERNAL_COUNT; i++)
        filter2D(working, kernelResults[i], -1, kernels[i]);

    // Pass directional kernel complements over image (same edge, rotated 180 degrees, 3 pixels between edge and complement)
    //cerr << "DetectLines::Filtering complements" << endl;
    for(size_t i = 0; i < KERNAL_COUNT; i++)
        filter2D(working, complementResults[i], -1, kernelComplements[i]);

    // Multiply the results of kernel filter by its complement
    //cerr << "DetectLines::Multiplying Results" << endl;
    MultiplyByComplements(kernelResults, complementResults, kernelResults);

    working = Mat::zeros(kernelResults[0].size(), kernelResults[0].type());
    //cerr << "DetectLines::Thresholding Results" << endl;
    for(int i = 0; i < KERNAL_COUNT; i++) {
        // threshold(kernelResults[i], kernelResults[i], ((float)lineContinue*lineContinue)/255, 255, CV_THRESH_BINARY);
        adaptiveThreshold(kernelResults[i], kernelResults[i], 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, -1.85);
        bitwise_or(working, kernelResults[i], working);
    }
}

void LineDetector::WhitenessFilter(Mat& hsv_image, Mat& fin_img) {
    Mat result = 255 * Mat::ones(hsv_image.size(), CV_16UC1);
    Mat tmp;
    hsv_image.convertTo(tmp, CV_16UC3, 1.0);
    Mat channel[3];
    split(tmp, channel);
    result = result - channel[1];
    result = result.mul(channel[2]);
    result.convertTo(fin_img, CV_8UC1, 1.0/255);
}

void LineDetector::MultiplyByComplements(Mat* images, Mat* complements, Mat* results) {
    for(size_t i = 0; i < KERNAL_COUNT; i++) {
        Mat image;
        Mat complement;
        Mat result = Mat::zeros(images[0].size(), CV_16UC1);
        images[i].convertTo(image, CV_16UC1, 1.0);
        complements[i].convertTo(complement, CV_16UC1, 1.0);
        result = image.mul(complement);
        result.convertTo(results[i], CV_8UC1, 1.0/255); // TODO figure this const out
    }
}


void LineDetector::EnforceLength(Mat& img, int length, std::vector<std::vector<cv::Point>>& contoursThreshold) {
    // Thresholding the line length using the area of the contours
    vector<vector<Point>> contours;
    vector<vector<Point>> smallContours;
    contoursThreshold.clear();
    vector<Vec4i> hierarchy;

    rectangle(img, Point(0, 0), Point(img.cols - 1, img.rows - 1), Scalar(0, 0, 0));
    Mat copy = img.clone();
    findContours(copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    for (unsigned int i = 0; i < contours.size(); ++i) {
        if ((int) contours[i].size() > length) {
            contoursThreshold.push_back(contours[i]);
        } else {
            smallContours.push_back(contours[i]);
        }
    }

    Scalar color(0, 0, 0);
    drawContours(img, smallContours, -1, color, -1);
}