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
#include <utility>

#include <cmath>
#include <iostream>

#define AZURE_KINECT_INTRINSICS = "calib/azure_kinect.intr"
#define AZURE_KINECT_EXTRINSICS = "calib/azure_kinect_overhead_to_world.tf"

image_transport::Publisher marked_pub;
image_transport::Publisher ball_pub;
image_transport::Publisher trans_pub;
ros::Publisher pose_pub;
ros::Publisher catch_point_pub;

cv::Mat K;
cv::Mat extrinsics;
int frame_idx = 0;
std::pair<float, float> invalid_estimate = std::make_pair(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());

std::pair<float, float> ball_estimation(geometry_msgs::PointStamped point_msg)
{
  float dt = 1/30;
  float traj_time, root, root1, root2;
  float g = 9.81;
  float z_desired = 0.2;
  float curr_x_pos = point_msg.point.x;
  float curr_y_pos = point_msg.point.y;
  float curr_z_pos = point_msg.point.z;
  float prev_x_pos, prev_y_pos, prev_z_pos;
  float x_vel, y_vel, z_vel, x_vel_mean, y_vel_mean, z_vel_mean;

  if (point_msg.point.x > 2) {
    return std::make_pair(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
  }

  if (frame_idx == 0)
  {
    prev_x_pos = curr_x_pos;
    prev_y_pos = curr_y_pos;
    prev_z_pos = curr_z_pos;
    frame_idx++;
    return std::make_pair(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
  }

  if (frame_idx>=1 && frame_idx <= 10)
  {
    x_vel = (curr_x_pos - prev_x_pos)/dt;
    y_vel = (curr_y_pos - prev_y_pos)/dt;
    z_vel = (curr_z_pos - prev_z_pos)/dt;
    x_vel_mean += x_vel/10;
    y_vel_mean += y_vel/10; 
    z_vel_mean += z_vel/10; 
    frame_idx++;
    return std::make_pair(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
  }

  float a = -g;
  float b = 2*z_vel;
  float c = -0.4;

  float discriminant = b*b - 4*a*c;

  if (discriminant > 0)
  {
    root1 = (-b + sqrt(discriminant)) / (2*a);
    root2 = (-b - sqrt(discriminant)) / (2*a);
    root = std::max(root1, root2);
  }
  else if (discriminant==0)
  {
    root = -b/(2*a);
  }
  else
  {
    ROS_ERROR("Roots are imaginary");
    return std::make_pair(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
  }

  float x_final = curr_x_pos + x_vel_mean*root;
  float y_final = curr_y_pos + y_vel_mean*root;

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
                          cv::Size(2*kernel_size + 1, 2*kernel_size + 1),
                          cv::Point(kernel_size, kernel_size));
  cv::erode(threshFrame, threshFrame, element_erode);
  cv::dilate(threshFrame, threshFrame, element_dilate);

  // Calculate pixel centroid
  cv::Moments m = cv::moments(threshFrame, false);
  cv::Point com(m.m10 / m.m00, m.m01 / m.m00);

  // Draw center point
  cv::Scalar color = cv::Scalar(0, 0, 255);
  cv::drawMarker(frame, com, color, cv::MARKER_CROSS, 20, 5);

  if (m.m00 == 0) {
    ROS_WARN("Skipping, did not detect ball.");
    return;
  }

  /*** 2D Image Space to 3D Position ***/
  double x = m.m10 / m.m00;
  double y = m.m01 / m.m00;
  
  ///////////////// Add radius if needed
  ROS_INFO("x: %f, y: %f", x, y);
  // Get depth from 2D Depth Image
  float depth = depth_frame.at<float>(int(y), int(x));
  ROS_INFO("%f", depth);

  if (depth <= 0) {
    ROS_WARN("Skipping, Depth less than or equal to 0.");
    return;
  }

  // Create homogenous pixel position
  cv::Mat pixel_pos = (cv::Mat_<double>(3,1) << x, y, 1.0);

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
  ball.point.y = ball_to_cam.at<double>(1);
  ball.point.z = ball_to_cam.at<double>(2);
  ball.header.frame_id = "cam";

  /*** Converting to ROS Image Message and Publishing ***/
  // Publish Original Image with Marger Message
  msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
  marked_pub.publish(msg);

  // Publish Image Message
  msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", blurFrame).toImageMsg();
  trans_pub.publish(msg);

  // Publish Thresholded Image Message
  msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshFrame).toImageMsg();
  ball_pub.publish(msg);

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
    catch_point.point.z = 0.2;
    catch_point.header.frame_id = "panda_link0";
    catch_point_pub.publish(catch_point);
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
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, depth_sub);
  sync.registerCallback(boost::bind(&imgCallback, _1, _2));

  // Advertisers
  marked_pub = it.advertise("ball_marked", 1);
  trans_pub = it.advertise("ball_trans", 1);
  ball_pub = it.advertise("ball_img", 1);
  pose_pub = nh_.advertise<geometry_msgs::PointStamped>("ball_pose", 5);
  catch_point_pub = nh_.advertise<geometry_msgs::PointStamped>("catch_point", 5);

  // Intrinsics
  K = (cv::Mat_<double>(3,3) << 972.31787109375, 0.0, 1022.3043212890625, 0.0, 971.8189086914062, 777.7421875, 0.0, 0.0, 1.0);

  // Extrisnics for Camera to World Frame
  extrinsics = (cv::Mat_<double>(4,4) <<  -0.383097, -0.244886, 0.890645,-0.069287, 
                                          -0.923347, 0.074961, -0.376553, 0.371678, 
                                          0.025449, -0.966650, -0.254833,  0.654526, 
                                          0.0, 0.0, 0.0, 1.0);

  ros::spin();

  return 0;
}