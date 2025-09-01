#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class TFTestNode : public rclcpp::Node
{
public:
    TFTestNode() : Node("tf_test")
    {
        // Create a TransformBuffer and a TransformListener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Example usage: Get a transform
        this->get_transform("base_link", "target_frame");
    }

private:
    void get_transform(const std::string &source_frame, const std::string &target_frame)
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0));
            RCLCPP_INFO(this->get_logger(), "Transform from %s to %s: [%f, %f, %f]",
                        source_frame.c_str(), target_frame.c_str(),
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFTestNode>());
    rclcpp::shutdown();
    return 0;
}