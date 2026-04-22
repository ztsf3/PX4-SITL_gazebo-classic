#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#ifdef HAVE_PX4_MSGS
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#endif

#include "CommandMotorSpeed.pb.h"

namespace {

float sanitize_value(const float value)
{
  return std::isfinite(value) ? value : 0.0f;
}

}  // namespace

class RosGazeboBridge : public rclcpp::Node
{
public:
  RosGazeboBridge()
  : Node("ros_gazebo_actuator_bridge")
  {
    control_allocation_topic_ = declare_parameter<std::string>("control_allocation_topic",
                                                                "/control_allocation/actuator_setpoint");
    control_array_offset_ = declare_parameter<int>("control_array_offset", 0);
    use_px4_topics_ = declare_parameter<bool>("use_px4_topics", false);
    motors_topic_ = declare_parameter<std::string>("motors_topic", "/fmu/out/actuator_motors");
    servos_topic_ = declare_parameter<std::string>("servos_topic", "/fmu/out/actuator_servos");
    gazebo_topic_ = declare_parameter<std::string>("gazebo_topic",
                                                   "/gazebo/default/tailsitter/gazebo/command/motor_speed");
    gazebo_channel_count_ = declare_parameter<int>("gazebo_channel_count", 16);
    motors_channel_offset_ = declare_parameter<int>("motors_channel_offset", 0);
    servos_channel_offset_ = declare_parameter<int>("servos_channel_offset", 12);
    use_servo_topic_ = declare_parameter<bool>("use_servo_topic", false);
    wait_for_connection_ = declare_parameter<bool>("wait_for_connection", true);

    if (gazebo_channel_count_ <= 0) {
      throw std::runtime_error("Parameter 'gazebo_channel_count' must be > 0");
    }

    command_channels_.assign(static_cast<size_t>(gazebo_channel_count_), 0.0f);

    gz_node_.reset(new gazebo::transport::Node());
    gz_node_->Init();

    gz_pub_ = gz_node_->Advertise<mav_msgs::msgs::CommandMotorSpeed>(gazebo_topic_);

    if (wait_for_connection_) {
      RCLCPP_INFO(get_logger(), "Waiting for Gazebo subscriber on %s", gazebo_topic_.c_str());
      gz_pub_->WaitForConnection();
      RCLCPP_INFO(get_logger(), "Gazebo subscriber connected");
    }

    auto qos = rclcpp::SensorDataQoS();

#ifdef HAVE_PX4_MSGS
    if (use_px4_topics_) {
      motors_sub_ = create_subscription<px4_msgs::msg::ActuatorMotors>(
        motors_topic_,
        qos,
        std::bind(&RosGazeboBridge::on_motors, this, std::placeholders::_1));

      if (use_servo_topic_) {
        servos_sub_ = create_subscription<px4_msgs::msg::ActuatorServos>(
          servos_topic_,
          qos,
          std::bind(&RosGazeboBridge::on_servos, this, std::placeholders::_1));
      }
    } else {
      control_array_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        control_allocation_topic_,
        qos,
        std::bind(&RosGazeboBridge::on_control_array, this, std::placeholders::_1));
    }
#else
    if (use_px4_topics_) {
      RCLCPP_WARN(get_logger(),
                  "Parameter use_px4_topics=true but this binary was built without px4_msgs support."
                  " Falling back to Float32MultiArray input.");
      use_px4_topics_ = false;
    }

    control_array_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      control_allocation_topic_,
      qos,
      std::bind(&RosGazeboBridge::on_control_array, this, std::placeholders::_1));
#endif

    if (use_px4_topics_) {
      RCLCPP_INFO(
        get_logger(),
        "ROS-Gazebo bridge ready (PX4 mode): motors=%s servos=%s(use=%s) gazebo_topic=%s channels=%d offset(motors=%d servos=%d)",
        motors_topic_.c_str(),
        servos_topic_.c_str(),
        use_servo_topic_ ? "true" : "false",
        gazebo_topic_.c_str(),
        gazebo_channel_count_,
        motors_channel_offset_,
        servos_channel_offset_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "ROS-Gazebo bridge ready (Float32 mode): input=%s gazebo_topic=%s channels=%d offset=%d",
        control_allocation_topic_.c_str(),
        gazebo_topic_.c_str(),
        gazebo_channel_count_,
        control_array_offset_);
    }
  }

private:
#ifdef HAVE_PX4_MSGS
  void on_motors(const px4_msgs::msg::ActuatorMotors::SharedPtr msg)
  {
    write_channels(msg->control, motors_channel_offset_);
    publish_command();
  }

  void on_servos(const px4_msgs::msg::ActuatorServos::SharedPtr msg)
  {
    write_channels(msg->control, servos_channel_offset_);
    publish_command();
  }
#endif

  void on_control_array(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    write_channels(msg->data, control_array_offset_);
    publish_command();
  }

  template<typename ChannelContainer>
  void write_channels(const ChannelContainer &source, const int offset)
  {
    if (offset >= gazebo_channel_count_) {
      return;
    }

    const int first_dest_index = std::max(offset, 0);
    const size_t first_source_index = static_cast<size_t>(first_dest_index - offset);
    const size_t first_dest = static_cast<size_t>(first_dest_index);

    for (size_t i = 0; first_source_index + i < source.size() && first_dest + i < command_channels_.size(); ++i) {
      command_channels_[first_dest + i] = sanitize_value(static_cast<float>(source[first_source_index + i]));
    }
  }

  void publish_command()
  {
    mav_msgs::msgs::CommandMotorSpeed msg;

    for (const float value : command_channels_) {
      msg.add_motor_speed(value);
    }

    gz_pub_->Publish(msg);
  }

  std::string control_allocation_topic_;
  std::string motors_topic_;
  std::string servos_topic_;
  std::string gazebo_topic_;

  int control_array_offset_{0};
  int gazebo_channel_count_{16};
  int motors_channel_offset_{0};
  int servos_channel_offset_{12};
  bool use_px4_topics_{false};
  bool use_servo_topic_{false};
  bool wait_for_connection_{true};

  std::vector<float> command_channels_;

  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr gz_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr control_array_sub_;

#ifdef HAVE_PX4_MSGS
  rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_sub_;
  rclcpp::Subscription<px4_msgs::msg::ActuatorServos>::SharedPtr servos_sub_;
#endif
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::vector<std::string> gazebo_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> gazebo_argv;
  gazebo_argv.reserve(gazebo_args.size());

  for (std::string &arg : gazebo_args) {
    gazebo_argv.push_back(arg.data());
  }

  gazebo::client::setup(static_cast<int>(gazebo_argv.size()), gazebo_argv.data());

  auto node = std::make_shared<RosGazeboBridge>();
  rclcpp::spin(node);

  gazebo::client::shutdown();
  rclcpp::shutdown();
  return 0;
}
