#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "geometry_msgs/PointStamped.h"
#include <visualization_msgs/Marker.h>
#include <utility>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/QR>


#include <vector>
#include <cmath>
#include <iostream>

#define AZURE_KINECT_INTRINSICS = "calib/azure_kinect.intr"
#define AZURE_KINECT_EXTRINSICS = "calib/azure_kinect_overhead_to_world.tf"

image_transport::Publisher marked_pub;
image_transport::Publisher ball_pub;
image_transport::Publisher trans_pub;
image_transport::Publisher water_pub;
ros::Publisher pose_pub;
ros::Publisher catch_point_pub;
ros::Publisher vis;

float prev_depth;

const float NUMBER_OF_SAMPLES = 5;
const float START_DEPTH = 2.6;
const float HEIGHT_OFFSET = 0.5;

std::vector<double> samples_x(NUMBER_OF_SAMPLES);
std::vector<double> samples_y(NUMBER_OF_SAMPLES);
std::vector<double> samples_z(NUMBER_OF_SAMPLES);
std::vector<double> samples_t(NUMBER_OF_SAMPLES);
ros::Time prev_time;
int sample_count = 0;

cv::Mat K;
cv::Mat extrinsics;
int frame_idx = 0;
std::pair<float, float> invalid_estimate = std::make_pair(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());

void polyfit(const std::vector<double> &t,
		const std::vector<double> &v,
		std::vector<double> &coeff,
		int order)
{
	// Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
	Eigen::MatrixXd T(t.size(), order + 1);
	Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
	Eigen::VectorXd result;

	// check to make sure inputs are correct
	assert(t.size() == v.size());
	assert(t.size() >= order + 1);
	// Populate the matrix
	for(size_t i = 0 ; i < t.size(); ++i)
	{
		for(size_t j = 0; j < order + 1; ++j)
		{
			T(i, j) = pow(t.at(i), j);
		}
	}
	// std::cout<<T<<std::endl;
	
	// Solve for linear least square fit
	result  = T.householderQr().solve(V);
	coeff.resize(order+1);
	for (int k = 0; k < order+1; k++)
	{
		coeff[k] = result[k];
	}
}


std::pair<float, float> ball_estimation(geometry_msgs::PointStamped point_msg)
{
  ros::Duration diff = ros::Time::now() - prev_time;
  float dt = float(diff.nsec) / 1e9;

  float dist = pow(pow(point_msg.point.x,2) + pow(point_msg.point.y,2) + pow(point_msg.point.z,2), 0.5);

  ROS_ERROR("dist = %f",dist);


  if (dist < 0.2 || dist > START_DEPTH) {
    ROS_WARN("Buffers reset");
    samples_x.clear();
    samples_y.clear();
    samples_z.clear();
    samples_t.clear();
    sample_count = 0;
    return invalid_estimate;
  }

  if (sample_count < NUMBER_OF_SAMPLES) {
    samples_x.push_back(point_msg.point.x);
    samples_y.push_back(point_msg.point.y);
    samples_z.push_back(point_msg.point.z);

    visualization_msgs::Marker points;
    points.header.frame_id = "panda_link0";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = sample_count;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = point_msg.point.x;
    p.y = point_msg.point.y;
    p.z = point_msg.point.z;
    points.points.push_back(p);
    vis.publish(points);

    double t = float(ros::Time::now().nsec) / 1e9;
    samples_t.push_back(t);

    sample_count++;

    if (sample_count < NUMBER_OF_SAMPLES) {
      return invalid_estimate;
    }
  }

  ros::Time s = ros::Time::now();

  std::vector<double> x_coeff;
  std::vector<double> y_coeff;
  std::vector<double> z_coeff;
  polyfit(samples_t, samples_x, x_coeff, 1);
  polyfit(samples_t, samples_y, y_coeff, 1);
  polyfit(samples_t, samples_z, z_coeff, 2);

  float a = z_coeff[2];
  float b = z_coeff[1];
  float c = z_coeff[0] - HEIGHT_OFFSET;

  float discriminant = b*b - 4*a*c;
  float root, root1, root2;

  if (discriminant > 0)
  {
    root1 = (-b + sqrt(discriminant)) / (2*a);
    root2 = (-b - sqrt(discriminant)) / (2*a);
    root = std::min(root1, root2);
  }
  else if (discriminant==0)
  {
    root = -b/(2*a);
  }
  else
  {
    ROS_ERROR("Roots are imaginary");
    return invalid_estimate;
  }

  ROS_WARN("Root = %f, %f, %f, %f", root, root1, root2, discriminant);
  float x_cand_min = x_coeff[0] + std::min(root1, root2) * x_coeff[1];
  float x_cand_max = x_coeff[0] + std::max(root1, root2) * x_coeff[1];
  float y_cand_min = y_coeff[0] + std::min(root1, root2) * y_coeff[1];
  float y_cand_max = y_coeff[0] + std::max(root1, root2) * y_coeff[1];

  ROS_WARN("Candidates x = %f, %f | y = %f, %f", x_cand_min, x_cand_max, y_cand_min, y_cand_max);

  float x_final, y_final;

  float x_centre = 0.45;
  float y_centre = -0.1;

  if (abs(x_cand_max - x_centre) < abs(x_cand_min - x_centre)) {
    x_final = x_cand_max;
  }
  else {
    x_final = x_cand_min;
  }

  if (abs(y_cand_max - y_centre) < abs(y_cand_min - y_centre)) {
    y_final = y_cand_max;
  }
  else {
    y_final = y_cand_min;
  }

  ros::Duration end = ros::Time::now() - s;

  ROS_WARN("Selected = %f, %f", x_final, y_final);
  ROS_ERROR("%f", end.nsec / 1e9);
  prev_time = ros::Time::now();

  return std::make_pair(x_final, y_final);
}

// RGB Image
void imgCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
  /*** Converting Depth Img to OpenCV Image ***/
  // OpenCV Pointer
  cv_bridge::CvImagePtr cv_ptr_depth;

  // Convert the ROS message  
  cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, "32FC1");

  // Store Image into Mat
  cv::Mat depth_frame = cv_ptr_depth->image;

  /*** Converting RGB Image to OpenCV Image ***/
  // Create Image pointer to be published
  sensor_msgs::ImagePtr msg;

  // OpenCV Pointer
  cv_bridge::CvImagePtr cv_ptr;

  // Convert the ROS message  
  cv_ptr = cv_bridge::toCvCopy(img_msg, "rgb8");
    
  // Store Image into Mat
  cv::Mat frame = cv_ptr->image;

  /*** OpenCV Operations ***/
  // RGB to HSV
  cv::Mat hsvFrame;
  cv::cvtColor(frame, hsvFrame, cv::COLOR_RGB2HSV);

  // Gaussian Blur
  cv::Mat blurFrame;
  cv::blur(hsvFrame, blurFrame, cv::Size(1, 1));

  // Threshold
  cv::Scalar lowerBound = cv::Scalar(28, 85, 107);
  cv::Scalar upperBound = cv::Scalar(38, 255, 255);

  cv::Mat threshFrame;
  cv::inRange(blurFrame, lowerBound, upperBound, threshFrame);
  
  // Erosion & Dilation for Noise
  int kernel_size = 1;
  cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                          cv::Size(2*kernel_size + 1, 2*kernel_size + 1),
                          cv::Point(kernel_size, kernel_size));
  cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                          cv::Size(2*kernel_size + 3, 2*kernel_size + 3),
                          cv::Point(kernel_size, kernel_size));
  cv::erode(threshFrame, threshFrame, element_erode);
  cv::dilate(threshFrame, threshFrame, element_dilate);

  /*** Watershed Algorithm ***/
  // Perform the distance transform algorithm
  cv::Mat dist_w;
  cv::distanceTransform(threshFrame, dist_w, cv::DIST_L2, 3);

  cv:: Mat imgResult;
  cv::cvtColor(threshFrame, imgResult, CV_GRAY2BGR, 3);

  // Normalize the distance image for range = {0.0, 1.0}
  // so we can visualize and threshold it
  cv::normalize(dist_w, dist_w, 0, 1.0, cv::NORM_MINMAX);

  // This will be the markers for the foreground objects
  cv::threshold(dist_w, dist_w, 0.4, 1.0, cv::THRESH_BINARY);

  // Create the CV_8U version of the distance image
  // It is needed for findContours()
  cv::Mat dist_8u;
  dist_w.convertTo(dist_8u, CV_8U);
  // Find total markers
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(dist_8u, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    ROS_INFO_THROTTLE(1, "No contours, prev_depth = %f", prev_depth);
    return;
  }

  // Create the marker image for the watershed algorithm
  cv::Mat markers = cv::Mat::zeros(dist_w.size(), CV_32S);

  int largest_idx = -1;
  int largest_contour_size = 0;
  for (size_t i = 0; i < contours.size(); i++)
  {
    if (contours[i].size() > largest_contour_size && contours[i].size() > 5) {
      largest_idx = i;
      largest_contour_size = contours[i].size();
    }
  }
  // ROS_INFO("Contour %d has %d size", largest_idx, largest_contour_size);

  std::vector<std::vector<cv::Point>> contours_largest;
  // Find total markers
  if (largest_contour_size > 0) {
    contours_largest.push_back(contours[largest_idx]);
  }
  else {
    ROS_INFO_THROTTLE(1, "No contours were of sufficient size");
    return;
  }

  // Draw the foreground markers
  // cv::drawContours(markers, contours_largest, static_cast<int>(0), cv::Scalar(static_cast<int>(0)+1), -1);
  
  // Draw the background marker
  // cv::circle(markers, cv::Point(5,5), 3, cv::Scalar(255), -1);
  // cv::Mat markers8u;
  // markers.convertTo(markers8u, CV_8U, 10);

  // Perform the watershed algorithm
  cv::watershed(imgResult, markers);
  cv::Mat mark;
  markers.convertTo(mark, CV_8U);
  cv::bitwise_not(mark, mark);

  // Draw center point
  cv::Scalar color = cv::Scalar(0, 0, 255);  
  
  ///////////////// Add radius if needed
  float depth_sum = 0;
  int depth_count = 0;
  float x_sum = 0;
  float y_sum = 0;

  float depth;
  // Get depth from 2D Depth Image
  for (int i = 0; i < contours[largest_idx].size(); i++) {
    auto point = contours[largest_idx][i];
    x_sum += point.x;
    y_sum += point.y;
  }

  x_sum /= contours[largest_idx].size();
  y_sum /= contours[largest_idx].size();

  float min_depth = std::numeric_limits<float>::max();
  int half_dim = 25;

  // Get depth from 2D Depth Image
  for (int i = -half_dim; i <= half_dim; i++) {
    for (int j = -half_dim; j <= half_dim; j++) {
      depth = depth_frame.at<float>(int(y_sum)+i, int(x_sum)+j);
      if (depth < min_depth && depth > 0.2) {
        min_depth = depth;
      }
      // if (depth > 0.05 && depth < 2) {
      //   std::cout << depth << std::endl;
      //   depth_sum += depth;
      //   depth_count++;
      // }
    }
  }

  if (min_depth < 0.2 || min_depth > 3) {
    depth = prev_depth;
  }
  else {
    depth = min_depth;
    prev_depth = depth;
  }

  if (depth == 0) {
    ROS_INFO("Invalid depth");
    return;
  }

  //depth_sum /= depth_count;
  // ROS_INFO("Float depth: %f", depth);

  // cv::circle(mark, cv::Point(x_sum,y_sum), 15, cv::Scalar(0), -1);

  // Create homogenous pixel position
  cv::Mat pixel_pos = (cv::Mat_<double>(3,1) << x_sum, y_sum, 1.0);

  // Get ball position relative to camera
  cv::Mat ball_to_cam = depth*K.inv()*pixel_pos;

  // Append into homogenous vector
  ball_to_cam.push_back(1.0);

  // Get ball to world frame
  cv::Mat ball_to_world = extrinsics * ball_to_cam;

  // Divide by last element to get x,y,z
  ball_to_world /= ball_to_world.at<double>(3);

  geometry_msgs::PointStamped ball;
  ball.point.x = ball_to_cam.at<double>(0);

  // Publish Image Message
  // msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", blurFrame).toImageMsg();
  // trans_pub.publish(msg);

  // Publish Thresholded Image Message
  // msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshFrame).toImageMsg();
  // ball_pub.publish(msg);

  // Publish Thresholded Image Message
  // msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mark).toImageMsg();
  // water_pub.publish(msg);

  /*** Publish Ball Center Position Relative to World Frame ***/
  // Create Image pointer to be published
  geometry_msgs::PointStamped point_msg;

  point_msg.header = std_msgs::Header();
  point_msg.point.x = ball_to_world.at<double>(0);
  point_msg.point.y = ball_to_world.at<double>(1);
  point_msg.point.z = ball_to_world.at<double>(2);

  std::pair<float, float> estimated_catch_point = ball_estimation(point_msg);

  if (estimated_catch_point != invalid_estimate) {
    geometry_msgs::PointStamped catch_point;
    catch_point.point.x = estimated_catch_point.first;
    catch_point.point.y = estimated_catch_point.second;
    catch_point.point.z = 0.3;
    catch_point.header.frame_id = "panda_link0";
    catch_point_pub.publish(catch_point);

    ROS_WARN("POINT CATCHED AT = %f, %f, %f", catch_point.point.x, catch_point.point.y, catch_point.point.z);
  }

  point_msg.header.frame_id = "panda_link0";
  pose_pub.publish(point_msg);

  // pose_pub.publish(ball_in_panda);
}

int main(int argc, char **argv)
{
  // Initialize node and handler
  ros::init(argc, argv, "detect");
  ros::NodeHandle nh_;

  // Image_transport is responsible for publishing and subscribing to Images
  image_transport::ImageTransport it(nh_);

  // Set up subscibers and publishers
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh_, "/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh_, "/depth_to_rgb/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), image_sub, depth_sub);
  sync.registerCallback(boost::bind(&imgCallback, _1, _2));

  // Advertisers
  // marked_pub = it.advertise("ball_marked", 1);
  // trans_pub = it.advertise("ball_trans", 1);
  // ball_pub = it.advertise("ball_img", 1);
  // water_pub = it.advertise("water_img", 1);
  pose_pub = nh_.advertise<geometry_msgs::PointStamped>("ball_pose", 5);
  catch_point_pub = nh_.advertise<geometry_msgs::PointStamped>("catch_point", 5);
  vis = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Intrinsics
  K = (cv::Mat_<double>(3,3) << 909.8428955078125, 0.0, 961.3718872070312, 0.0, 909.5616455078125, 549.1278686523438, 0.0, 0.0, 1.0);

  // Extrisnics for Camera to World Frame
  extrinsics = (cv::Mat_<double>(4,4) <<  -0.425703, 0.050087, 0.903465, -0.064335, 
                                          -0.904793, -0.035017, -0.424387, 0.394543, 
                                          0.010381, -0.998131 ,0.060225,  0.701992, 
                                          0.0, 0.0, 0.0, 1.0);

  prev_time = ros::Time::now();
  ros::spin();

  return 0;
}  