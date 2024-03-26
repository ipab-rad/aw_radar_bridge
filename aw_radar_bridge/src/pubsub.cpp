// #include <chrono>
// #include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PubSub : public rclcpp::Node
{
  public:
    PubSub()
    : Node("aw_radar_pubsub")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        // "/sensor/radar/bumper_front_centre/far/image", 10,
        "/input", 10,
        std::bind(&PubSub::topic_CB, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/output", 10);
    }

  private:
    void topic_CB(const std_msgs::msg::String & msg) const
    {
      // WIP
      auto message = std_msgs::msg::String();
      message.data = "Radar Output";
      publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSub>());
  rclcpp::shutdown();
  return 0;
}
