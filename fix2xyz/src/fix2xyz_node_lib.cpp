#define DEBUG_MODE
#include "fix2xyz_node_lib.h"

void LLAXYZTransNode::fix_callback0(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  debug_msg = "get fix_msg0";
  DEBUG_PRINT(debug_msg);

  nav_msgs::msg::Odometry ret_msg;
  Eigen::Vector3d xyz;
  fix_xyz_trans::LatLonAlt latlonalt;

  latlonalt.latitude = msg->latitude;
  latlonalt.longitude = msg->longitude;
  latlonalt.altitude = msg->altitude;

  xyz = l_u_transformer.get_xyz_from_latlonalt(latlonalt);

  ret_msg.header = msg->header;
  ret_msg.header.frame_id = map_frame;
  ret_msg.pose.pose.position.x = xyz(0);
  ret_msg.pose.pose.position.y = xyz(1);
  ret_msg.pose.pose.position.z = xyz(2);
  ret_msg.pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 9; ++i) {
    ret_msg.pose.covariance[i] = msg->position_covariance[i];
  }
  ret_msg.pose.covariance[21] = rot_cov;
  ret_msg.pose.covariance[28] = rot_cov;
  ret_msg.pose.covariance[35] = rot_cov;

  if (fix1_isfirst) {
    fix1_isfirst = false;
  } else {
    double movement_x = xyz(0) - last_fix1_pose.position.x;
    double movement_y = xyz(1) - last_fix1_pose.position.y;
    double movement = movement_x * movement_x + movement_y * movement_y;
    if (movement > 0.1) {
      double theta = std::atan2(movement_y, movement_x);
      ret_msg.pose.pose.orientation.z = std::sin(0.5 * theta);
      ret_msg.pose.pose.orientation.w = std::cos(0.5 * theta);
    } else {
      ret_msg.pose.pose.orientation = last_fix1_pose.orientation;
    }
  }

  last_fix1_pose = ret_msg.pose.pose;
  odom_pub0->publish(ret_msg);
}

void LLAXYZTransNode::fix_callback1(const fix_msgs::msg::FixWithOrientation::SharedPtr msg) {
  debug_msg = "get fix_msg1";
  DEBUG_PRINT(debug_msg);

  const auto& fix_msg = msg->fix;
  nav_msgs::msg::Odometry ret_msg;
  fix_xyz_trans::Pose pose;
  fix_xyz_trans::LLAWithOrientation lla_with_ori;

  lla_with_ori.lla.latitude = fix_msg.latitude;
  lla_with_ori.lla.longitude = fix_msg.longitude;
  lla_with_ori.lla.altitude = fix_msg.altitude;
  lla_with_ori.orientation << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;

  pose = l_u_transformer.get_xyz_from_latlonalt(lla_with_ori);

  ret_msg.header = fix_msg.header;
  ret_msg.header.frame_id = map_frame;
  ret_msg.pose.pose.position.x = pose.position(0);
  ret_msg.pose.pose.position.y = pose.position(1);
  ret_msg.pose.pose.position.z = pose.position(2);
  ret_msg.pose.pose.orientation.x = pose.orientation(0);
  ret_msg.pose.pose.orientation.y = pose.orientation(1);
  ret_msg.pose.pose.orientation.z = pose.orientation(2);
  ret_msg.pose.pose.orientation.w = pose.orientation(3);

  for (int i = 0; i < 9; ++i) {
    ret_msg.pose.covariance[i] = fix_msg.position_covariance[i];
  }
  ret_msg.pose.covariance[21] = rot_cov;
  ret_msg.pose.covariance[28] = rot_cov;
  ret_msg.pose.covariance[35] = rot_cov;

  odom_pub1->publish(ret_msg);

  debug_msg = "published odometry_msg from fix_msg1";
  DEBUG_PRINT(debug_msg);
}

void LLAXYZTransNode::fix_callback2(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  debug_msg = "get fix_msg2";
  DEBUG_PRINT(debug_msg);

  geometry_msgs::msg::PoseStamped ret_msg;
  Eigen::Vector3d xyz;
  fix_xyz_trans::LatLonAlt latlonalt;

  latlonalt.latitude = msg->latitude;
  latlonalt.longitude = msg->longitude;
  latlonalt.altitude = msg->altitude;

  xyz = l_u_transformer.get_xyz_from_latlonalt(latlonalt);

  ret_msg.header = msg->header;
  ret_msg.header.frame_id = map_frame;
  ret_msg.pose.position.x = xyz(0);
  ret_msg.pose.position.y = xyz(1);
  ret_msg.pose.position.z = xyz(2);
  ret_msg.pose.orientation.w = 1.0;

  if (fix2_isfirst) {
    fix2_isfirst = false;
  } else {
    double movement_x = xyz(0) - last_fix2_pose.position.x;
    double movement_y = xyz(1) - last_fix2_pose.position.y;
    double movement = movement_x * movement_x + movement_y * movement_y;
    if (movement > 0.1) {
      double theta = std::atan2(movement_y, movement_x);
      ret_msg.pose.orientation.z = std::sin(0.5 * theta);
      ret_msg.pose.orientation.w = std::cos(0.5 * theta);
    } else {
      ret_msg.pose.orientation = last_fix2_pose.orientation;
    }
  }

  last_fix2_pose = ret_msg.pose;
  pose_pub2->publish(ret_msg);

  debug_msg = "published pose_stamped_msg from fix_msg2";
  DEBUG_PRINT(debug_msg);
}

void LLAXYZTransNode::fix_callback3(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  debug_msg = "get fix_msg3";
  DEBUG_PRINT(debug_msg);

  geometry_msgs::msg::PoseWithCovarianceStamped ret_msg;
  Eigen::Vector3d xyz;
  fix_xyz_trans::LatLonAlt latlonalt;

  latlonalt.latitude = msg->latitude;
  latlonalt.longitude = msg->longitude;
  latlonalt.altitude = msg->altitude;

  xyz = l_u_transformer.get_xyz_from_latlonalt(latlonalt);

  ret_msg.header = msg->header;
  ret_msg.header.frame_id = map_frame;
  ret_msg.pose.pose.position.x = xyz(0);
  ret_msg.pose.pose.position.y = xyz(1);
  ret_msg.pose.pose.position.z = xyz(2);
  ret_msg.pose.pose.orientation.w = 1.0;

  for (int i = 0; i < 9; ++i) {
    ret_msg.pose.covariance[i] = msg->position_covariance[i];
  }
  ret_msg.pose.covariance[21] = rot_cov;
  ret_msg.pose.covariance[28] = rot_cov;
  ret_msg.pose.covariance[35] = rot_cov;

  if (fix3_isfirst) {
    fix3_isfirst = false;
  } else {
    double movement_x = xyz(0) - last_fix3_pose.position.x;
    double movement_y = xyz(1) - last_fix3_pose.position.y;
    double movement = movement_x * movement_x + movement_y * movement_y;
    if (movement > 0.1) {
      double theta = std::atan2(movement_y, movement_x);
      ret_msg.pose.pose.orientation.z = std::sin(0.5 * theta);
      ret_msg.pose.pose.orientation.w = std::cos(0.5 * theta);
    } else {
      ret_msg.pose.pose.orientation = last_fix3_pose.orientation;
    }
  }

  last_fix3_pose = ret_msg.pose.pose;
  posecov_pub3->publish(ret_msg);

  debug_msg = "published pose_with_covariance_stamped_msg from fix_msg3";
  DEBUG_PRINT(debug_msg);
}


LLAXYZTransNode::LLAXYZTransNode()
: Node("llaxyz_trans_node"),
  tfBuffer(this->get_clock()),
  tfListener(tfBuffer)
{
  // Declare parameters with default values
  this->declare_parameter("sub_fix_topic0", "fix0");
  this->declare_parameter("sub_fix_topic1", "fix1");
  this->declare_parameter("sub_fix_topic2", "fix2");
  this->declare_parameter("sub_fix_topic3", "fix3");

  this->declare_parameter("pub_odom_topic0", "odometry/from_fix0");
  this->declare_parameter("pub_odom_topic1", "odometry/from_fix1");
  this->declare_parameter("pub_pose_topic2", "pose/from_fix2");
  this->declare_parameter("pub_posecov_topic3", "posecov/from_fix3");

  this->declare_parameter("map_frame", "map");
  this->declare_parameter("rot_cov", 1e9);
  this->declare_parameter("make_angle_from_movement", false);
  this->declare_parameter("epsg_code_num", 0);
  this->declare_parameter("origin_pose", "");
  this->declare_parameter("origin_quat", "");
  this->declare_parameter("origin_quat_inv", "");

  // Get parameters
  this->get_parameter("sub_fix_topic0", sub_fix_topic0);
  this->get_parameter("sub_fix_topic1", sub_fix_topic1);
  this->get_parameter("sub_fix_topic2", sub_fix_topic2);
  this->get_parameter("sub_fix_topic3", sub_fix_topic3);

  this->get_parameter("pub_odom_topic0", pub_odom_topic0);
  this->get_parameter("pub_odom_topic1", pub_odom_topic1);
  this->get_parameter("pub_pose_topic2", pub_pose_topic2);
  this->get_parameter("pub_posecov_topic3", pub_posecov_topic3);

  this->get_parameter("map_frame", map_frame);
  this->get_parameter("rot_cov", rot_cov);
  this->get_parameter("make_angle_from_movement", make_angle_from_movement);

  if (this->get_parameter("epsg_code_num", epsg_code_num)) {
    l_u_transformer.set_epsg_code(epsg_code_num);
  }

  fix_xyz_trans::LatLonAlt origin_latlonalt_;
  Eigen::Vector4d origin_quat_(0.0, 0.0, 0.0, 1.0);
  double qx, qy, qz, qw;

  if (this->get_parameter("origin_pose", origin_pose_str) && !origin_pose_str.empty()) {
    std::replace(origin_pose_str.begin(), origin_pose_str.end(), ',', ' ');
    std::istringstream iss_pose(origin_pose_str);
    iss_pose >> origin_latlonalt_.latitude >> origin_latlonalt_.longitude >> origin_latlonalt_.altitude;
  }

  if (this->get_parameter("origin_quat", origin_quat_str) && !origin_quat_str.empty()) {
    std::replace(origin_quat_str.begin(), origin_quat_str.end(), ',', ' ');
    std::istringstream iss_quat(origin_quat_str);
    iss_quat >> qx >> qy >> qz >> qw;
    origin_quat_ << qx, qy, qz, qw;
    if (origin_quat_.norm() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid quaternion");
      origin_quat_ << 0.0, 0.0, 0.0, 1.0;
    } else {
      origin_quat_.normalize();
    }
  } else if (this->get_parameter("origin_quat_inv", origin_quat_str) && !origin_quat_str.empty()) {
    std::replace(origin_quat_str.begin(), origin_quat_str.end(), ',', ' ');
    std::istringstream iss_quat(origin_quat_str);
    iss_quat >> qx >> qy >> qz >> qw;
    origin_quat_ << -qx, -qy, -qz, qw;
    if (origin_quat_.norm() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid quaternion");
      origin_quat_ << 0.0, 0.0, 0.0, 1.0;
    } else {
      origin_quat_.normalize();
    }
  }

  l_u_transformer.set_origin(origin_latlonalt_, origin_quat_);

  // Publishers
  odom_pub0 = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic0, 10);
  odom_pub1 = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic1, 10);
  pose_pub2 = this->create_publisher<geometry_msgs::msg::PoseStamped>(pub_pose_topic2, 10);
  posecov_pub3 = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pub_posecov_topic3, 10);
  
  // Subscriptions
  fix_sub0 = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    sub_fix_topic0, 10, std::bind(&LLAXYZTransNode::fix_callback0, this, std::placeholders::_1));
  fix_sub1 = this->create_subscription<fix_msgs::msg::FixWithOrientation>(
    sub_fix_topic1, 10, std::bind(&LLAXYZTransNode::fix_callback1, this, std::placeholders::_1));
  fix_sub2 = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    sub_fix_topic2, 10, std::bind(&LLAXYZTransNode::fix_callback2, this, std::placeholders::_1));
  fix_sub3 = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    sub_fix_topic3, 10, std::bind(&LLAXYZTransNode::fix_callback3, this, std::placeholders::_1));

  fix1_isfirst = true;
  fix2_isfirst = true;
  fix3_isfirst = true;
}

LLAXYZTransNode::~LLAXYZTransNode() {}
