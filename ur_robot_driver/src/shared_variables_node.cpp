// Copyright ...
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-11-30
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/shared_variables_interface.h"

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ur_robot_driver/urcl_log_handler.hpp"

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class SharedVariablesNode : public rclcpp::Node
{
public:
  struct SharedVariables
  {
    double var_0;
    double var_1;
    double var_2;
    double var_3;
    double var_4;
    double stop_thread;
  };

  SharedVariablesNode(const std::string& node_name) : rclcpp::Node(node_name)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("message_callback", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SharedVariablesNode::timer_callback, this));

    stop_main_loop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_main_loop", [&](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                              std_srvs::srv::Trigger::Response::SharedPtr resp) {
          variables_.stop_thread = 1.0;
          resp->success = true;
          return true;
        });

    ur_robot_driver::registerUrclLogHandler();
  }
  ~SharedVariablesNode()
  {
    ur_robot_driver::unregisterUrclLogHandler();
  };

  void timer_callback()
  {
    std::array<double, 6> vars;
    vars = { variables_.var_0, variables_.var_1, variables_.var_2,
             variables_.var_3, variables_.var_4, variables_.stop_thread };
    //        if (shared_variables_interface_)
    if (!shared_variables_interface_->writeVariables(&vars)) {
      RCLCPP_INFO(get_logger(), "Could not write variables");
    }
  }

  bool initialize(const std::string& robot_ip, int port)
  {
    robot_ip_ = robot_ip;
    port_ = port;

    shared_variables_interface_ = std::make_unique<urcl::control::SharedVariablesInterface>(port);

    shared_variables_interface_->setExternalMessageCallback([this] { message_callback(); });

    return true;
  }

  void message_callback()
  {
    //        RCLCPP_INFO(get_logger(), "External message_callback executed");

    std_msgs::msg::String msg;
    msg.data = "External message_callback executed";
    publisher_->publish(msg);
  }

private:
  std::unique_ptr<urcl::control::SharedVariablesInterface> shared_variables_interface_;
  //      std::unique_ptr<comm::URStream<primary_interface::PrimaryPackage>> tertiary_stream_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_main_loop_service_;

  SharedVariables variables_{ 0.0, 0.0, 0.0, 0.0, 0.0, 0 };

protected:
  std::string robot_ip_;
  int port_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SharedVariablesNode>("shared_variables_node");

  // The IP address under which the robot is reachable.
  std::string robot_ip = node->declare_parameter<std::string>("robot_ip", "192.168.56.101");
  int port = node->declare_parameter<int>("port", 30003);

  // get parameters
  node->get_parameter<std::string>("robot_ip", robot_ip);
  node->get_parameter<int>("port", port);

  // init all comm stuff
  if (node->initialize(robot_ip, port)) {
    RCLCPP_INFO(node->get_logger(), "Initialized");
  }

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
