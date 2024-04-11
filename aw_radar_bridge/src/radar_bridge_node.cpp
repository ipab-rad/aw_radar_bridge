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

/**
 * @class RadarBridge
 * @brief Converts continental's radar detection msgs into ROS perception radar scans msgs
 *
 * This class subscribes to radar detection, processes them into a standardized
 * radar scan format, and publishes the results. It's designed to be used in systems
 * where radar data needs to be transformed before further processing.
 *
 */
class RadarBridge : public rclcpp::Node
{
public:
  RadarBridge()
      : Node("aw_radar_pubsub")
  {
    /// Get frame id
    frame_id_ = this->declare_parameter<std::string>("frame_id", "frame_name");

    /// Ger radar input topic
    input_topic_ = this->declare_parameter<std::string>("input_topic", "radar_input");

    /// Define subscription for original Radar msgs
    subscription_ = this->create_subscription<RadarDetectionImage>(
        input_topic_, 10, std::bind(&RadarBridge::radar_CB, this, _1));

    /// Get radar output topic
    output_topic_ = this->declare_parameter("output_topic", "radar_output");

    // Define publisher for converted Radar msgs
    publisher_ = this->create_publisher<RadarScan>(output_topic_, 10);
  }

private:

  /**
   * @brief Processes incoming radar detection images and publishes radar scans.
   *
   * @param conti_msg The received radar detection image message.
   *
   */
  void radar_CB(const RadarDetectionImage &conti_msg) const
  {
    auto radar_scan_msg = RadarScan();
    radar_scan_msg.header = conti_msg.header;
    radar_scan_msg.header.frame_id = frame_id_;

    // Convert continental radar msg into ROS perception radar_msgs struct
    for (const auto &detection : conti_msg.a_radardetectionlist)
    {
      auto radar_return = RadarReturn();
      radar_return.range = detection.f_range;              // Radial distance [m]
      // Ensure detection azang hyp is non-empty
      if (detection.a_azang_hyp.size() > 0)
        radar_return.azimuth = detection.a_azang_hyp[0];   // Most likely hyp. [rad]
      radar_return.elevation = 0.0f;                       // 2D flat radar space
      radar_return.doppler_velocity = detection.f_vrelrad; // Rel speed [m/s]
      radar_return.amplitude = 1;                          // Not provided by Conti [dB]

      radar_scan_msg.returns.push_back(radar_return);
    }

    publisher_->publish(radar_scan_msg);
  }

  /// Radar I/O and frame id str's
  std::string input_topic_, output_topic_, frame_id_;

  /// Subscription for Contintal's Radar msg
  rclcpp::Subscription<RadarDetectionImage>::SharedPtr subscription_;

  /// Publisher for ROS perception Radar msg
  rclcpp::Publisher<RadarScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RadarBridge>());

  rclcpp::shutdown();

  return 0;
}
