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
    TrajectoryPublisher() : Node("trajectory_publisher"), counter_(1), mission_count_(0), state_(FLY_LEFT_RIGHT)
    {
        trajectory_publisher_ = this->create_publisher<TrajectorySetpoint>("/custom_trajectory", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        rclcpp::Logger logger = this->get_logger();
        RCLCPP_INFO(logger, "Trajectory Publisher Node Started.");
        // Publishes trajectory every 100ms.
        timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryPublisher::publish_trajectory, this));
    }

private:
    enum MissionState { FLY_LEFT_RIGHT, FLY_FORWARD_BACKWARD, FLY_UP_DOWN, ROTATE, DONE };
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    int mission_count_;
    MissionState state_;

    void publish_trajectory()
    {
        TrajectorySetpoint msg{};

        switch (state_)
        {
            case FLY_LEFT_RIGHT:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "'Fly left right'-mission.");
                }
                
                if (counter_ < 20) {
                    msg.position = {0.0, -4.0, -5.0}; // Go left 4m.
                } else if (counter_ < 40) {
                    msg.position = {0.0, 0.0, -5.0}; // Go right (go back).
                }

                msg.yaw = -3.14;

                if (counter_ >= 40) {
                    state_ = FLY_FORWARD_BACKWARD;
                    counter_ = 0;
                }

                break;
            
            case FLY_FORWARD_BACKWARD:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "'Fly forward backward'-mission.");
                }

                if (counter_ < 20) {
                    msg.position = {4.0, 0.0, -5.0}; // Fly forward (north)
                } else if (counter_ < 40) {
                    msg.position = {0.0, 0.0, -5.0}; // Go back.
                }

                msg.yaw = -3.14;
                
                if (counter_ >= 40) {
                    state_ = FLY_UP_DOWN;
                    counter_ = 0;
                }

                break;
            
            case FLY_UP_DOWN:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "'Fly up down'-mission.");
                }
                
                if (counter_ < 20) {
                    msg.position = {0.0, 0.0, -3.0}; // Go down.
                } else if (counter_ < 40) {
                    msg.position = {0.0, 0.0, -5.0}; // Go up.
                }
                
                msg.yaw = -3.14;

                if (counter_ >= 40) {
                    mission_count_++;

                    if (mission_count_ < 3) {
                        state_ = FLY_LEFT_RIGHT;
                    } else {
                        state_ = DONE; // Stop sending commands (End of all missions).
                    }

                    counter_ = 0;
                }

                break;
            
            case DONE:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "Mission done. Set LAND-mode.");
                }

                set_land_mode();

                return;
        }
        
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        counter_++;
        trajectory_publisher_->publish(msg);
    }

    void set_land_mode();
};

// TODO: This function should be before deploying it.
void TrajectoryPublisher::set_land_mode()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE; // Command to change mode
    msg.param1 = 1;  // Custom mode
    msg.param2 = 6;  // 6 corresponds to LAND mode in PX4
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    vehicle_command_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "LAND mode set.");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
