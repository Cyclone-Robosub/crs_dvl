#include <cstdio>
#include <functional>
#include <chrono>
using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "interfaces/srv/connect_serial.hpp"
#include "interfaces/srv/disconnect.hpp"
#include "interfaces/srv/start.hpp"
#include "interfaces/srv/stop.hpp"
#include "interfaces/srv/command.hpp"

#include "interfaces/msg/ins.hpp"


namespace crs::dvl {
  class NucleusControllerNode : public rclcpp::Node
  {
  public:

    rclcpp::Client<interfaces::srv::ConnectSerial>::SharedPtr connect_serial_client_;
    rclcpp::Client<interfaces::srv::Start>::SharedPtr start_dvl_client_;

    rclcpp::Subscription<interfaces::msg::INS>::SharedPtr ins_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_publisher_;

  public:
    NucleusControllerNode(const std::string& node_name)
    : Node(node_name)
    {
      connect_serial_client_ = this->create_client<interfaces::srv::ConnectSerial>(
        "/nucleus_node/connect_serial"
      );

      if(!connect_serial_client_->wait_for_service(10s)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out waiting for connect_serial service");
      }

      start_dvl_client_ = this->create_client<interfaces::srv::Start>( "/nucleus_node/start" );

      if(!start_dvl_client_->wait_for_service(10s)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out waiting for /nucleus_node/start service");
      }

      ins_subscription_ = this->create_subscription<interfaces::msg::INS>(
        "/nucleus_node/ins_packets",
        10,   // TODO: check if this QoS is acceptable
        std::bind(&NucleusControllerNode::ins_packet_cb, this, std::placeholders::_1)
      );

      position_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "position_topic",
        10
      );
    }

  private:
    void ins_packet_cb(interfaces::msg::INS::UniquePtr ins_msg) {
      // Update state provided to control system here
      RCLCPP_INFO(this->get_logger(), "Received INS Packet:\n  x: %f\n  y: %f\n  z: %f", ins_msg->position_frame_x, ins_msg->position_frame_y, ins_msg->position_frame_z);

      auto position_msg = std_msgs::msg::Float32MultiArray();

      position_msg.data = {
        ins_msg->position_frame_x,
        ins_msg->position_frame_y,
        ins_msg->position_frame_z,
        ins_msg->roll,
        ins_msg->pitch,
        ins_msg->heading
      };

      this->position_publisher_->publish(position_msg);
    }
  };

  bool connect_dvl(std::shared_ptr<crs::dvl::NucleusControllerNode> nucleus_controller_node, const std::string& serial_port) {
    auto connect_request = std::make_shared<interfaces::srv::ConnectSerial::Request>();
    connect_request->serial_port = serial_port;
    auto connect_result = nucleus_controller_node->connect_serial_client_->async_send_request(connect_request);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent DVL connect request");

    if (rclcpp::spin_until_future_complete(nucleus_controller_node, connect_result, std::chrono::seconds(10s)) == rclcpp::FutureReturnCode::SUCCESS) {
      if (connect_result.get()->status) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Conected to DVL");
        return true;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not connect to DVL");
        return false;
      }

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "connect request interuppted or timed out");
      return false;
    }
  }

  bool start_dvl(std::shared_ptr<crs::dvl::NucleusControllerNode> nucleus_controller_node) {
    auto start_request = std::make_shared<interfaces::srv::Start::Request>();
    auto start_result = nucleus_controller_node->start_dvl_client_->async_send_request(start_request);

    if (rclcpp::spin_until_future_complete(nucleus_controller_node, start_result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start request received response: %s", start_result.get()->reply.c_str());
      return true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Start request interrupted or time out");
      return false;
    }
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world crs_dvl package\n");

  rclcpp::init(argc, argv);

  auto nucleus_controller_node = std::make_shared<crs::dvl::NucleusControllerNode>("nucleus_controller_node");
  bool succeeded = crs::dvl::connect_dvl(nucleus_controller_node, "/dev/ttyUSB0");
  succeeded = crs::dvl::start_dvl(nucleus_controller_node);
  rclcpp::spin(nucleus_controller_node);

  rclcpp::shutdown();
  
  return 0;
}
