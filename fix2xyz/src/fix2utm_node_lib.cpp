// #define DEBUG_MODE
#include "fix2utm_node_lib.h"

void LLAUTMTransNode::fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  debug_msg = "get fix_msg";
  DEBUG_PRINT(debug_msg);

  nav_msgs::msg::Odometry ret_msg;
  UTM utm;
  LatLonAlt latlonalt;

  latlonalt.latitude = msg->latitude;
  latlonalt.longitude = msg->longitude;
  latlonalt.altitude = msg->altitude;

  utm = lla_utm_transformer.get_utm_from_latlonalt(latlonalt);

  ret_msg.header = msg->header;
  ret_msg.header.frame_id = frame_id;
  ret_msg.pose.pose.position.x = utm.x;
  ret_msg.pose.pose.position.y = utm.y;
  ret_msg.pose.pose.position.z = utm.z;
  ret_msg.pose.pose.orientation.w = 1.0;

  ret_msg.pose.covariance[0] = msg->position_covariance[0];
  ret_msg.pose.covariance[1] = msg->position_covariance[1];
  ret_msg.pose.covariance[2] = msg->position_covariance[2];
  ret_msg.pose.covariance[6] = msg->position_covariance[3];
  ret_msg.pose.covariance[7] = msg->position_covariance[4];
  ret_msg.pose.covariance[8] = msg->position_covariance[5];
  ret_msg.pose.covariance[12] = msg->position_covariance[6];
  ret_msg.pose.covariance[13] = msg->position_covariance[7];
  ret_msg.pose.covariance[14] = msg->position_covariance[8];
  ret_msg.pose.covariance[21] = rot_cov;
  ret_msg.pose.covariance[28] = rot_cov;
  ret_msg.pose.covariance[35] = rot_cov;

  if(make_angle_from_movement){
    if (isfirst) {
      isfirst = false;
    } else {
      double movement_x = utm.x - last_utm.position.x;
      double movement_y = utm.y - last_utm.position.y;
      double movement = movement_x * movement_x + movement_y * movement_y;
      if (movement > 0.1) {
        double theta = std::atan2(movement_y, movement_x);
        ret_msg.pose.pose.orientation.z = std::sin(0.5 * theta);
        ret_msg.pose.pose.orientation.w = std::cos(0.5 * theta);
      } else {
        ret_msg.pose.pose.orientation = last_utm.orientation;
      }
    }
  }

  last_utm = ret_msg.pose.pose;
  odom_pub->publish(ret_msg);
}


LLAUTMTransNode::LLAUTMTransNode()
: Node("llaxyz_trans_node"),
  tfBuffer(this->get_clock()),
  tfListener(tfBuffer),
  lla_utm_transformer(true)
{
  // Declare parameters with default values
  this->declare_parameter<std::string>("sub_fix_topic", "fix");

  this->declare_parameter<std::string>("pub_odom_topic", "utm");

  this->declare_parameter<std::string>("frame_id", "utm");
  this->declare_parameter<double>("rot_cov", 1e9);
  this->declare_parameter<bool>("make_angle_from_movement", true);

  // Get parameters
  this->get_parameter("sub_fix_topic", sub_fix_topic);

  this->get_parameter("pub_odom_topic", pub_odom_topic);

  this->get_parameter("frame_id", frame_id);
  this->get_parameter("rot_cov", rot_cov);
  this->get_parameter("make_angle_from_movement", make_angle_from_movement);

  // Publishers
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic, 10);

  // Subscriptions
  fix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    sub_fix_topic, 10, std::bind(&LLAUTMTransNode::fix_callback, this, std::placeholders::_1));

  isfirst = true;
}

LLAUTMTransNode::~LLAUTMTransNode() {}
