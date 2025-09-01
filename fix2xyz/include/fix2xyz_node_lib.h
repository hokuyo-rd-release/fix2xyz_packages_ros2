#ifndef LATLON_UTM_TRANS_NODE_HPP
#define LATLON_UTM_TRANS_NODE_HPP

#include <cmath>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fix_msgs/msg/fix_with_orientation.hpp>

#include <proj.h>
#include "fix_xyz_transform.h"

using namespace fix_xyz_trans;

#ifdef DEBUG_MODE
#define DEBUG_PRINT(A) std::cout << (#A) << " : " << (A) << std::endl;
#else
#define DEBUG_PRINT(A) {;}
#endif

class LLAXYZTransNode : public rclcpp::Node {
public:
  LLAXYZTransNode();
  ~LLAXYZTransNode();

private:
  // 変換器とTF
  LLAXYZTrans l_u_transformer;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  std::string debug_msg;

  // サブスクライバ
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub0;
  rclcpp::Subscription<fix_msgs::msg::FixWithOrientation>::SharedPtr fix_sub1;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub2;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub3;
  
  // パブリッシャ
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub1;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub2;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posecov_pub3;
  
  // トピック名
  std::string sub_fix_topic0;
  std::string sub_fix_topic1;
  std::string sub_fix_topic2;
  std::string sub_fix_topic3;

  std::string pub_odom_topic0;
  std::string pub_odom_topic1;
  std::string pub_pose_topic2;
  std::string pub_posecov_topic3;

  std::string origin_pose_str;
  std::string origin_quat_str;
  std::string map_frame;

  int epsg_code_num;
  double rot_cov;
  bool make_angle_from_movement;

  bool fix1_isfirst;
  bool fix2_isfirst;
  bool fix3_isfirst;
  geometry_msgs::msg::Pose last_fix1_pose;
  geometry_msgs::msg::Pose last_fix2_pose;
  geometry_msgs::msg::Pose last_fix3_pose;

  // コールバック関数
  void fix_callback0(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void fix_callback1(const fix_msgs::msg::FixWithOrientation::SharedPtr msg);
  void fix_callback2(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void fix_callback3(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

#endif  // LATLON_UTM_TRANS_NODE_HPP
