#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher"), counter_(0), state_(TAKEOFF)
    {
        trajectory_publisher_ = this->create_publisher<TrajectorySetpoint>("/custom_trajectory", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        // rclcpp::Logger logger = this->get_logger();
        RCLCPP_INFO(logger, "Trajectory Publisher Node Started.");
        // Publishes trajectory every 100ms.
        timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryPublisher::publish_trajectory, this));
    }

private:
    enum MissionState { TAKEOFF, FLY_UP_DOWN, FLY_LEFT_RIGHT, FLY_SQUARE, LAND, DONE };
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    MissionState state_;

    void publish_trajectory()
    {
        TrajectorySetpoint msg{};

        switch (state_)
        {
            case TAKEOFF:
                if (counter_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "Takeoff.");
                }
                
                msg.position = {0.0, 0.0, -5.0}; // Go up (takeoff) to 5m height.
                msg.yaw = -3.14;
                if (counter_ >= 100) { // Wait for 10 seconds (assuming counter_ is updated every 100ms).
                    state_ = FLY_UP_DOWN; // Start a new mission.
                    counter_ = 0;
                }
                break;
            
            case FLY_UP_DOWN:
                if (counter_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "'Fly up down'-mission.");
                }
                
                if (counter_ < 100) {
                    msg.position = {0.0, 0.0, 0.0};
                } else if (counter_ < 200) {
                    msg.position = {0.0, 0.0, -5.0};
                }
                
                msg.yaw = -3.14;

                if (counter_ >= 200) { // Wait for 10 seconds (assuming counter_ is updated every 100ms).
                    state_ = FLY_LEFT_RIGHT; // Start a new mission.
                    counter_ = 0;
                }

                break;
            
            case FLY_LEFT_RIGHT:
                if (counter_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "'Fly left right'-mission.");
                }
                
                if (counter_ < 100) {
                    msg.position = {0.0, -4.0, -5.0}; // Go left 4m.
                } else if (counter_ < 200) {
                    msg.position = {0.0, 0.0, -5.0}; // Go right (go back).
                } else if (counter_ < 300) {
                    msg.position = {0.0, -4.0, -5.0};  // Go left 4m.
                } else if (counter_ < 400) {
                    msg.position = {0.0, 0.0, -5.0}; // Go right (go back).
                }

                msg.yaw = -3.14;

                if (counter_ >= 400) {
                    state_ = DONE; // state_ = LAND;
                    counter_ = 0;
                }

                break;
            
            case LAND:
                if (counter_ < 100) {
                    msg.position = {0.0, 0.0, 0.0};
                }

                msg.yaw = -3.14;
                
                if (counter_ >= 100) {
                    state_ = DONE;
                    counter_ = 0;
                }

                break;

            case DONE:
                if (counter_ == 0) {
                    RCLCPP_INFO(this->get_logger(), "Mission done. Disarming drone...");
                    send_land_command();
                }
                
                return;
        }
        
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        counter_++;
        trajectory_publisher_->publish(msg);
    }

    void send_land_command();
};

void TrajectoryPublisher::send_land_command()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND; // Land command
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    vehicle_command_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Landing command sent.");
}

int main(int argc, char *argv[])
{
    rclcpp::Logger logger = rclcpp::get_logger("trajectory_publisher");
    if (rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
        RCLCPP_WARN(logger, "Failed to set log level.");
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
