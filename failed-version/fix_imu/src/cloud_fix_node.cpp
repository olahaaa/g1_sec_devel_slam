#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudTimeFixer : public rclcpp::Node {
public:
    PointCloudTimeFixer() : Node("cloud_fix_node") {

        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::Reliable);
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/utlidar/fixed_cloud_mid360", pub_qos);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);  //设置为RELIABLE

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud_livox_mid360", qos,
            std::bind(&PointCloudTimeFixer::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto fixed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
        
        //强制时间戳 = 当前 ROS 时间（保证单调）
        fixed_msg->header.stamp = this->now();
        
        pub_->publish(*fixed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTimeFixer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}