#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudSubscriber : public rclcpp::Node
{
public:
    PointCloudSubscriber() : Node("point_cloud_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud_topic", 10, std::bind(&PointCloudSubscriber::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Callback function to process the received PointCloud2 message
        // You can add your processing logic here
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
