#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // A simple message type for the demo
#include <chrono>

class SonarConnectionNode : public rclcpp::Node
{
public:
    SonarConnectionNode() : Node("sonar_connection_node")
    {
        // 1. Declare and get the watchdog timeout parameter
        this->declare_parameter<double>("watchdog_timeout_sec", 3.0);
        double watchdog_timeout = this->get_parameter("watchdog_timeout_sec").as_double();

        RCLCPP_INFO(
            this->get_logger(),
            "Creating a watchdog with a %.2f second timeout.", watchdog_timeout
        );

        // 2. Create the watchdog timer (and cancel it immediately)
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(watchdog_timeout),
            std::bind(&SonarConnectionNode::watchdog_callback, this)
        );
        watchdog_timer_->cancel(); // The timer is created but "paused" until we start it.

        // 3. Create a subscription to the data topic
        // The topic_callback function will be executed for each incoming message.
        std::string topic_name = "sonar_data";
        data_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            topic_name,
            10, // QoS history depth
            std::bind(&SonarConnectionNode::topic_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(
            this->get_logger(),
            "Node initialized. Listening for data on topic '%s'.", topic_name.c_str()
        );
        RCLCPP_INFO(this->get_logger(), "Call start_monitoring() to arm the watchdog.");
    }

    // Call this method after you've established your initial connection
    void start_monitoring()
    {
        RCLCPP_INFO(this->get_logger(), "Connection monitoring is now active!");
        // This starts the countdown for the first time.
        watchdog_timer_->reset();
    }

private:
    /**
     * @brief This is the callback for incoming data. It represents a healthy connection.
     * In your real code, this would be the bathyHandler, wcHandler, etc.
     */
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Data received: '%s'. Petting the watchdog.", msg->data.c_str());

        // This is the most important line: reset the watchdog's countdown.
        // As long as messages keep coming, the watchdog_callback will never be called.
        watchdog_timer_->reset();
    }

    /**
     * @brief This callback only executes if the timer is not reset within its timeout period.
     * This indicates that the connection has been lost.
     */
    void watchdog_callback()
    {
        RCLCPP_ERROR(this->get_logger(), "WATCHDOG FIRED! No data received in time. Assuming disconnection.");
        
        // Stop the timer to prevent it from firing again while we try to reconnect.
        watchdog_timer_->cancel();

        // Here, you would implement your reconnection logic:
        // 1. Close sockets/connections.
        // 2. Attempt to reopen them in a loop.
        // 3. If successful, call start_monitoring() again to re-arm the watchdog.
        RCLCPP_INFO(this->get_logger(), "Would attempt to reconnect now...");
    }

    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SonarConnectionNode>();

    // In your real application, you would call this after successfully
    // opening the TCP sockets to the sonar.
    node->start_monitoring();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}