#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <fix_msgs/msg/fix_with_orientation.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "rsf_chain_monitor.h"

class FixOrientPublisher : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Publisher<fix_msgs::msg::FixWithOrientation>::SharedPtr fix_with_orient_pub_;

    nav_msgs::msg::Odometry utm_odom_;
    bool utm_odom_is_initialized_;

    // 受信状況のログ出力
    std::unique_ptr<RsfChainMonitor> fix_monitor_;
    std::unique_ptr<RsfChainMonitor> utm_monitor_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    sensor_msgs::msg::NavSatFix last_fix_;
    int8_t last_fix_status_;
    uint64_t published_;
    uint64_t dropped_no_utm_;
    uint64_t n_invalid_fix_;
    uint64_t n_invalid_utm_;

public:
    FixOrientPublisher();
    void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callbackFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void diagCallback();
};

FixOrientPublisher::FixOrientPublisher() : Node("fix_odom_to_fix_orient"){
    utm_odom_is_initialized_ = false;
    last_fix_status_ = std::numeric_limits<int8_t>::min();
    published_ = 0;
    dropped_no_utm_ = 0;
    n_invalid_fix_ = 0;
    n_invalid_utm_ = 0;

    std::string utm_odom_topic;
    std::string fix_topic;
    std::string fix_with_orient_topic;

    this->declare_parameter<std::string>("utm_odom_topic", "/odometry/utm");
    this->declare_parameter<std::string>("fix_topic", "/fix/switch");
    this->declare_parameter<std::string>("fix_with_orient_topic", "/fix_with_orient/switch");
    this->declare_parameter<double>("diag_period", 1.0);
    // 最も遅い GNSS 系のトピックが 1Hz なので、1Hz で 2 回続けて欠けたら途絶とみなす
    this->declare_parameter<double>("stall_timeout", 3.0);

    this->get_parameter("utm_odom_topic", utm_odom_topic);
    this->get_parameter("fix_topic", fix_topic);
    this->get_parameter("fix_with_orient_topic", fix_with_orient_topic);
    const double diag_period = this->get_parameter("diag_period").as_double();
    const double stall_timeout = this->get_parameter("stall_timeout").as_double();

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

    fix_monitor_ = std::make_unique<RsfChainMonitor>(this, fix_topic, stall_timeout);
    utm_monitor_ = std::make_unique<RsfChainMonitor>(this, utm_odom_topic, stall_timeout);
    diag_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(diag_period), std::bind(&FixOrientPublisher::diagCallback, this));
    RCLCPP_INFO(this->get_logger(), "起動: %s + %s -> %s (diag_period=%.2fs, stall_timeout=%.2fs)",
        fix_topic.c_str(), utm_odom_topic.c_str(), fix_with_orient_topic.c_str(), diag_period, stall_timeout);
}

void FixOrientPublisher::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    utm_monitor_->onReceive(msg->header.stamp);

    const auto & q = msg->pose.pose.orientation;
    const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(q_norm) || std::abs(q_norm - 1.0) > 0.01) {
        ++n_invalid_utm_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "utm odom の向きが不正: quat=(%.4f, %.4f, %.4f, %.4f) |q|=%.4f", q.x, q.y, q.z, q.w, q_norm);
    }

    if (!utm_odom_is_initialized_) {
        RCLCPP_INFO(this->get_logger(), "utm odom を初めて受信. fix_with_orient の出力を開始する");
    }
    utm_odom_ = *msg;
    utm_odom_is_initialized_ = true;
}

void FixOrientPublisher::callbackFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    fix_monitor_->onReceive(msg->header.stamp);

    // NavSatStatus: -1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX
    if (msg->status.status != last_fix_status_) {
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
            RCLCPP_WARN(this->get_logger(), "fix status 変化: %d -> %d (stamp=%.3f)",
                last_fix_status_, msg->status.status, rclcpp::Time(msg->header.stamp).seconds());
        } else {
            RCLCPP_INFO(this->get_logger(), "fix status 変化: %d -> %d (stamp=%.3f)",
                last_fix_status_, msg->status.status, rclcpp::Time(msg->header.stamp).seconds());
        }
        last_fix_status_ = msg->status.status;
    }
    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude)) {
        ++n_invalid_fix_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "fix に NaN/Inf: lat=%f lon=%f alt=%f", msg->latitude, msg->longitude, msg->altitude);
    }
    last_fix_ = *msg;

    if (utm_odom_is_initialized_) {
        auto pub_msg = fix_msgs::msg::FixWithOrientation();
        pub_msg.fix = *msg;
        pub_msg.orientation = utm_odom_.pose.pose.orientation;
        fix_with_orient_pub_->publish(pub_msg);
        ++published_;
    } else {
        ++dropped_no_utm_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "utm odom を未受信のため fix を破棄 (累計 %lu)", static_cast<unsigned long>(dropped_no_utm_));
    }
}

void FixOrientPublisher::diagCallback() {
    fix_monitor_->checkStall();
    utm_monitor_->checkStall();

    const std::string fix_summary = fix_monitor_->summarize();
    if (fix_monitor_->received()) {
        RCLCPP_INFO(this->get_logger(), "%s | status=%d lat=%.8f lon=%.8f alt=%.2f cov_xx=%.4f",
            fix_summary.c_str(), last_fix_.status.status, last_fix_.latitude, last_fix_.longitude,
            last_fix_.altitude, last_fix_.position_covariance[0]);
    } else {
        RCLCPP_INFO(this->get_logger(), "%s", fix_summary.c_str());
    }

    const std::string utm_summary = utm_monitor_->summarize();
    if (utm_monitor_->received()) {
        const auto & q = utm_odom_.pose.pose.orientation;
        RCLCPP_INFO(this->get_logger(), "%s | yaw=%.1fdeg",
            utm_summary.c_str(), rsfChainYaw(q.x, q.y, q.z, q.w) * 180.0 / M_PI);
    } else {
        RCLCPP_INFO(this->get_logger(), "%s", utm_summary.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "[out] published=%lu dropped_no_utm=%lu invalid_fix=%lu invalid_utm=%lu",
        static_cast<unsigned long>(published_), static_cast<unsigned long>(dropped_no_utm_),
        static_cast<unsigned long>(n_invalid_fix_), static_cast<unsigned long>(n_invalid_utm_));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixOrientPublisher>());
    rclcpp::shutdown();
    return 0;
}
