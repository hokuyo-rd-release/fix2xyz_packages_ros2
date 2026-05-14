#define DEBUG_MODE
#include "xyz2fix_node_lib.h"

void XYZLLATransNode::odom_callback1(const nav_msgs::msg::Odometry::SharedPtr msg) {
  debug_msg = "get odom_msg";
  DEBUG_PRINT(debug_msg);

  fix_msgs::msg::FixWithOrientation ret_msg;
  sensor_msgs::msg::NavSatFix fix_msg;
  fix_xyz_trans::Pose pose;
  fix_xyz_trans::LLAWithOrientation lla_with_ori;

  std::string lio_frame = msg->header.frame_id;
  RCLCPP_INFO(this->get_logger(), "map_frame : %s", map_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "tar_frame : %s", lio_frame.c_str());

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    rclcpp::Time time_zero(0);
    transformStamped = tfBuffer.lookupTransform(map_frame, lio_frame, time_zero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  geometry_msgs::msg::Pose pose_on_map;
  tf2::doTransform(msg->pose.pose, pose_on_map, transformStamped);

  pose.position << pose_on_map.position.x, pose_on_map.position.y, pose_on_map.position.z;
  pose.orientation << pose_on_map.orientation.x, pose_on_map.orientation.y,
                      pose_on_map.orientation.z, pose_on_map.orientation.w;

  lla_with_ori = lla_xyz_transformer.get_latlonalt_from_xyz(pose);

  fix_msg.header = msg->header;
  fix_msg.header.frame_id = "";
  fix_msg.latitude = lla_with_ori.lla.latitude;
  fix_msg.longitude = lla_with_ori.lla.longitude;
  fix_msg.altitude = lla_with_ori.lla.altitude;

  for (int i = 0; i < 9; ++i) {
    fix_msg.position_covariance[i] = msg->pose.covariance[i + (i / 3) * 3];
  }

  ret_msg.fix = fix_msg;
  ret_msg.orientation.x = lla_with_ori.orientation(0);
  ret_msg.orientation.y = lla_with_ori.orientation(1);
  ret_msg.orientation.z = lla_with_ori.orientation(2);
  ret_msg.orientation.w = lla_with_ori.orientation(3);

  fix_pub1->publish(ret_msg);
  debug_msg = "published fix_msg1 from odometry";
  DEBUG_PRINT(debug_msg);
}

void XYZLLATransNode::pose_callback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  debug_msg = "get pose_stamped";
  DEBUG_PRINT(debug_msg);

  sensor_msgs::msg::NavSatFix ret_msg;
  Eigen::Vector3d xyz;
  LatLonAlt latlonalt;

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    rclcpp::Time time_zero(0);
    transformStamped = tfBuffer.lookupTransform(map_frame, msg->header.frame_id, time_zero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  geometry_msgs::msg::Pose pose_on_map;
  tf2::doTransform(msg->pose, pose_on_map, transformStamped);

  xyz << pose_on_map.position.x, pose_on_map.position.y, pose_on_map.position.z;
  latlonalt = lla_xyz_transformer.get_latlonalt_from_xyz(xyz);

  ret_msg.header = msg->header;
  ret_msg.header.frame_id = "";
  ret_msg.latitude = latlonalt.latitude;
  ret_msg.longitude = latlonalt.longitude;
  ret_msg.altitude = latlonalt.altitude;

  fix_pub2->publish(ret_msg);
  debug_msg = "published fix_msg2 from pose_stamped";
  DEBUG_PRINT(debug_msg);
}

void XYZLLATransNode::posecov_callback3(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  debug_msg = "get pose_with_covariance_stamped";
  DEBUG_PRINT(debug_msg);

  sensor_msgs::msg::NavSatFix ret_msg;
  Eigen::Vector3d xyz;
  LatLonAlt latlonalt;

  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    rclcpp::Time time_zero(0);
    transformStamped = tfBuffer.lookupTransform(map_frame, msg->header.frame_id, time_zero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  geometry_msgs::msg::Pose pose_on_map;
  tf2::doTransform(msg->pose.pose, pose_on_map, transformStamped);

  xyz << pose_on_map.position.x, pose_on_map.position.y, pose_on_map.position.z;
  latlonalt = lla_xyz_transformer.get_latlonalt_from_xyz(xyz);

  ret_msg.header = msg->header;
  ret_msg.header.frame_id = "";
  ret_msg.latitude = latlonalt.latitude;
  ret_msg.longitude = latlonalt.longitude;
  ret_msg.altitude = latlonalt.altitude;

  for (int i = 0; i < 9; ++i) {
    ret_msg.position_covariance[i] = msg->pose.covariance[i + (i / 3) * 3];
  }

  fix_pub3->publish(ret_msg);
  debug_msg = "published fix_msg3 from pose_with_covariance_stamped";
  DEBUG_PRINT(debug_msg);
}


XYZLLATransNode::XYZLLATransNode()
: Node("xyzlla_trans_node"),
  tfBuffer(this->get_clock()),
  tfListener(tfBuffer)
{
  // Declare parameters with default values
  this->declare_parameter<std::string>("sub_odom_topic1", "odom1");
  this->declare_parameter<std::string>("sub_pose_topic2", "pose2");
  this->declare_parameter<std::string>("sub_posecov_topic3", "posecov3");

  this->declare_parameter<std::string>("pub_fix_topic1", "fix/from_odom1");
  this->declare_parameter<std::string>("pub_fix_topic2", "fix/from_pose2");
  this->declare_parameter<std::string>("pub_fix_topic3", "fix/from_posecov3");

  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("origin_pose", "");
  this->declare_parameter<std::string>("origin_quat", "");
  this->declare_parameter<std::string>("origin_quat_inv", "");

  // Get parameters
  this->get_parameter("sub_odom_topic1", sub_odom_topic1);
  this->get_parameter("sub_pose_topic2", sub_pose_topic2);
  this->get_parameter("sub_posecov_topic3", sub_posecov_topic3);

  this->get_parameter("pub_fix_topic1", pub_fix_topic1);
  this->get_parameter("pub_fix_topic2", pub_fix_topic2);
  this->get_parameter("pub_fix_topic3", pub_fix_topic3);

  this->get_parameter("map_frame", map_frame);
  LatLonAlt origin_latlonalt_;
  Eigen::Vector4d origin_quat_;
  double qx, qy, qz, qw;

  origin_pose_str = this->get_parameter("origin_pose").as_string();
  if (!origin_pose_str.empty()) {
    std::replace(origin_pose_str.begin(), origin_pose_str.end(), ',', ' ');
    std::istringstream iss_pose(origin_pose_str);
    iss_pose >> origin_latlonalt_.latitude >> origin_latlonalt_.longitude >> origin_latlonalt_.altitude;
  }

  origin_quat_str = this->get_parameter("origin_quat").as_string();
  if (!origin_quat_str.empty()) {
    std::replace(origin_quat_str.begin(), origin_quat_str.end(), ',', ' ');
    std::istringstream iss_quat(origin_quat_str);
    iss_quat >> qx >> qy >> qz >> qw;
    origin_quat_ << qx, qy, qz, qw;
  } else {
    origin_quat_str = this->get_parameter("origin_quat_inv").as_string();
    std::replace(origin_quat_str.begin(), origin_quat_str.end(), ',', ' ');
    std::istringstream iss_quat(origin_quat_str);
    iss_quat >> qx >> qy >> qz >> qw;
    origin_quat_ << -qx, -qy, -qz, qw;
  }

  if (origin_quat_.norm() == 0) {
    std::cerr << "invalid quaternion" << std::endl;
    origin_quat_ << 0.0, 0.0, 0.0, 1.0;
  } else {
    origin_quat_.normalize();
  }

  lla_xyz_transformer.set_origin(origin_latlonalt_, origin_quat_);

  // Subscribers
  odom_sub1 = this->create_subscription<nav_msgs::msg::Odometry>(
    sub_odom_topic1, 10, std::bind(&XYZLLATransNode::odom_callback1, this, std::placeholders::_1));
  pose_sub2 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    sub_pose_topic2, 10, std::bind(&XYZLLATransNode::pose_callback2, this, std::placeholders::_1));
  posecov_sub3 = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    sub_posecov_topic3, 10, std::bind(&XYZLLATransNode::posecov_callback3, this, std::placeholders::_1));

  // Publishers
  fix_pub1 = this->create_publisher<fix_msgs::msg::FixWithOrientation>(pub_fix_topic1, 10);
  fix_pub2 = this->create_publisher<sensor_msgs::msg::NavSatFix>(pub_fix_topic2, 10);
  fix_pub3 = this->create_publisher<sensor_msgs::msg::NavSatFix>(pub_fix_topic3, 10);
}

XYZLLATransNode::~XYZLLATransNode() {}
