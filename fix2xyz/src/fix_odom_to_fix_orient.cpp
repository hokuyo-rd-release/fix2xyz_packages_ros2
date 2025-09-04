#include <rclcpp/rclcpp.hpp>
#include <fix_msgs/msg/fix_with_orientation.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class FixOrientPublisher : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Publisher<fix_msgs::msg::FixWithOrientation>::SharedPtr fix_with_orient_pub_;

    nav_msgs::msg::Odometry utm_odom_;
    bool utm_odom_is_initialized_;

public:
    FixOrientPublisher();
    void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callbackFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

FixOrientPublisher::FixOrientPublisher() : Node("fix_odom_to_fix_orient"){
    utm_odom_is_initialized_ = false;

    std::string utm_odom_topic;
    std::string fix_topic;
    std::string fix_with_orient_topic;

    this->declare_parameter<std::string>("utm_odom_topic", "/odometry/utm");
    this->declare_parameter<std::string>("fix_topic", "/fix/switch");
    this->declare_parameter<std::string>("fix_with_orient_topic", "/fix_with_orient/switch");

    this->get_parameter("utm_odom_topic", utm_odom_topic);
    this->get_parameter("fix_topic", fix_topic);
    this->get_parameter("fix_with_orient_topic", fix_with_orient_topic);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        utm_odom_topic, 10,
        std::bind(&FixOrientPublisher::callbackOdometry, this, std::placeholders::_1)
    );

    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        fix_topic, 10,
        std::bind(&FixOrientPublisher::callbackFix, this, std::placeholders::_1)
    );

    fix_with_orient_pub_ = this->create_publisher<fix_msgs::msg::FixWithOrientation>(
        fix_with_orient_topic, 10
    );
}

void FixOrientPublisher::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    utm_odom_ = *msg;
    utm_odom_is_initialized_ = true;
}

void FixOrientPublisher::callbackFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (utm_odom_is_initialized_) {
        auto pub_msg = fix_msgs::msg::FixWithOrientation();
        pub_msg.fix = *msg;
        pub_msg.orientation = utm_odom_.pose.pose.orientation;
        fix_with_orient_pub_->publish(pub_msg);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixOrientPublisher>());
    rclcpp::shutdown();
    return 0;
}
