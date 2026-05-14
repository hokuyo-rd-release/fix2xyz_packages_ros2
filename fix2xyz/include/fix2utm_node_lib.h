#ifndef FIX_UTM_TRANS_NODE
#define FIX_UTM_TRANS_NODE

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

using namespace fix_utm_trans;

#ifdef DEBUG_MODE
#define DEBUG_PRINT(A) std::cout << (#A) << " : " << (A) << std::endl;
#else
#define DEBUG_PRINT(A) {;}
#endif

class LLAUTMTransNode : public rclcpp::Node {
public:
  LLAUTMTransNode();
  ~LLAUTMTransNode();

private:
  // 変換器とTF
  LLAUTMTrans lla_utm_transformer;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  std::string debug_msg;

  // サブスクライバ
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub;
  
  // パブリッシャ
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  
  // トピック名
  std::string sub_fix_topic;

  std::string pub_odom_topic;

  std::string frame_id;

  double rot_cov;
  bool make_angle_from_movement;

  bool isfirst;
  geometry_msgs::msg::Pose last_utm;

  // コールバック関数
  void fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

#endif  // LATLON_UTM_TRANS_NODE_HPP
