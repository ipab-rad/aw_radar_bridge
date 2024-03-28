#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "ecal_to_ros/msg/radar_detection.hpp"
#include "ecal_to_ros/msg/radar_detection_image.hpp"
#include "radar_msgs/msg/radar_scan.hpp"

using namespace std::chrono_literals;
using namespace ecal_to_ros::msg;
using namespace radar_msgs::msg;
using std::placeholders::_1;

class PubSub : public rclcpp::Node
{
  public:
    PubSub()
    : Node("aw_radar_pubsub")
    {
      this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
      this->get_parameter<std::string>("frame_id", frame_id_);

      this->declare_parameter("input_topic", rclcpp::PARAMETER_STRING);
      this->get_parameter<std::string>("input_topic", input_topic_);
      subscription_ = this->create_subscription<RadarDetectionImage>(
        input_topic_, 10, std::bind(&PubSub::topic_CB, this, _1));

      this->declare_parameter("output_topic", rclcpp::PARAMETER_STRING);
      this->get_parameter<std::string>("output_topic", output_topic_);
      publisher_ = this->create_publisher<RadarScan>(output_topic_, 10);
    }

  private:
    void topic_CB(const RadarDetectionImage & conti_msg) const
    {
      auto radar_msg = RadarScan();
      radar_msg.header = conti_msg.header;
      radar_msg.header.frame_id = frame_id_;

      for (std::vector<RadarDetection>::const_iterator
        i = conti_msg.a_radardetectionlist.begin();
        i != conti_msg.a_radardetectionlist.end(); ++i)
      {
        auto radar_return = RadarReturn();
        radar_return.range = i->f_range;          // Radial distance [m]
        radar_return.azimuth = i->a_azang_hyp[0]; // Most likely hyp. [rad]
        radar_return.elevation = 0.0f;            // 2D flat radar space
        radar_return.doppler_velocity = i->f_vrelrad;   // Rel speed [m/s]
        radar_return.amplitude = 1;               // Not provided by Conti [dB]
        radar_msg.returns.push_back(radar_return);
      }
      publisher_->publish(radar_msg);
    }
    std::string input_topic_, output_topic_, frame_id_;
    rclcpp::Subscription<RadarDetectionImage>::SharedPtr subscription_;
    rclcpp::Publisher<RadarScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSub>());
  rclcpp::shutdown();
  return 0;
}
