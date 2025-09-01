#ifndef LATLON_UTM_TRANS_NODE
#define LATLON_UTM_TRANS_NODE

#ifdef DEBUG_MODE
#define DEBUG_PRINT(A) std::cout << (#A) << " : " << (A) << std::endl;
#else
#define DEBUG_PRINT(A) {;}
#endif

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fix_msgs/msg/fix_with_orientation.hpp>
#include <proj.h>
#include "fix_xyz_transform.h"

using namespace fix_xyz_trans;

class XYZLLATransNode : public rclcpp::Node {
private:
  LLAXYZTrans l_u_transformer;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  std::string debug_msg;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub1;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub2;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posecov_sub3;

  rclcpp::Publisher<fix_msgs::msg::FixWithOrientation>::SharedPtr fix_pub1;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub2;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub3;

  std::string sub_odom_topic1;
  std::string sub_pose_topic2;
  std::string sub_posecov_topic3;

  std::string pub_fix_topic1;
  std::string pub_fix_topic2;
  std::string pub_fix_topic3;

  std::string origin_pose_str;
  std::string origin_quat_str;
  std::string map_frame;

  int epsg_code_num;

public:
  void odom_callback1(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pose_callback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void posecov_callback3(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  XYZLLATransNode();
  ~XYZLLATransNode();
};

#endif
