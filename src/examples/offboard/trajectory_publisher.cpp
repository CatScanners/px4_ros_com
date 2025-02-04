#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher"), counter_(0)
    {
        trajectory_publisher_ = this->create_publisher<TrajectorySetpoint>("/custom_trajectory", 10);
        // Publishes trajectory every 100ms.
        timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryPublisher::publish_trajectory, this));
    }

private:
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

    void publish_trajectory()
    {
        TrajectorySetpoint msg{};
        // msg.position = {20, 20, -5.0};
        // Initially counter is set to 0.
        if (counter_ < 50) {
            msg.position = {0.0, 0.0, -5.0};
        } else if (counter_ < 100) {
            msg.position = {20, 0.0, -5.0};
        } else if (counter_ < 150) {
            msg.position = {20, 20, -5.0};
        } else if (counter_ < 200) {
            msg.position = {0.0, 20, -5.0};
        }

        msg.yaw = -3.14;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        if ((counter_ % 50) == 0) {
            RCLCPP_INFO(this->get_logger(), "Published trajectory: [%.2f, %.2f, %.2f]",
                    msg.position[0], msg.position[1], msg.position[2]);
        }

        counter_++;
        
        // Reset the counter?
        if (counter_ >= 200) {
            counter_ = 0;
        }
        
        trajectory_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
