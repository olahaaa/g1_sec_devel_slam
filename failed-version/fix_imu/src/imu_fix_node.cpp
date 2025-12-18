#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <memory>

class ImuFixNode : public rclcpp::Node
{
public:
    ImuFixNode() : Node("imu_fix_node")
    {
        // imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("utlidar/fixed_imu_mid360", 10);
        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(2000)).reliability(rclcpp::ReliabilityPolicy::Reliable);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("utlidar/fixed_imu_mid360", pub_qos);
        
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
        sub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);  //设置为RELIABLE
        
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "utlidar/imu_livox_mid360", sub_qos,  // 使用配置好的QoS
            std::bind(&ImuFixNode::imuCallback, this, std::placeholders::_1));
    }
private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "fixing IMU message");
        auto fixed_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);
        fixed_msg->header.stamp = this->now();
        // rclcpp::Time now = this->now();
        // fixed_msg->header.stamp = rclcpp::Time(
        //     now.nanoseconds() - 10 * 1000000  // 提前 10ms
        // );
        fixed_msg->header.frame_id = "imu_link";
        const double G_TO_MS2 = 9.80665;
        const double RAD_PER_REV = 6.283185307179586; // 2 * π
        fixed_msg->angular_velocity.x *= RAD_PER_REV;
        fixed_msg->angular_velocity.y *= RAD_PER_REV;
        fixed_msg->angular_velocity.z *= RAD_PER_REV;
        fixed_msg->linear_acceleration.x *= G_TO_MS2;
        fixed_msg->linear_acceleration.y *= G_TO_MS2;
        fixed_msg->linear_acceleration.z *= G_TO_MS2;

        imu_publisher_->publish(*fixed_msg);
        
    }
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuFixNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}