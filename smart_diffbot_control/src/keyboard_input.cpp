/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Define used keys
namespace
{
constexpr int8_t KEYCODE_RIGHT = 0x43;
constexpr int8_t KEYCODE_LEFT = 0x44;
constexpr int8_t KEYCODE_UP = 0x41;
constexpr int8_t KEYCODE_DOWN = 0x42;
constexpr int8_t KEYCODE_Q = 0x71;
constexpr int8_t KEYCODE_A = 0x61;
constexpr int8_t KEYCODE_D = 0x64;
constexpr int8_t KEYCODE_S = 0x73;
constexpr int8_t KEYCODE_W = 0x77;
}  // namespace

// Some constants used in the Teleop demo
namespace
{
const std::string TWIST_TOPIC = "/diff_drive_controller/cmd_vel_unstamped";
const std::string JOINT_TOPIC = "/camera_controller/commands";
const size_t ROS_QUEUE_SIZE = 10;
}  // namespace

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : file_descriptor_(0)
  {
    // get the console in raw mode
    tcgetattr(file_descriptor_, &cooked_);
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(file_descriptor_, TCSANOW, &raw);
  }
  void readOne(char* c)
  {
    int rc = read(file_descriptor_, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(file_descriptor_, TCSANOW, &cooked_);
  }

private:
  int file_descriptor_;
  struct termios cooked_;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardInterface
{
public:
  KeyboardInterface();
  int keyLoop();

private:
  void spin();

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
  double joint_delta_;
  std::vector<double> joint_values_;
  std::string command_frame_id_;
};

KeyboardInterface::KeyboardInterface() : joint_delta_(0.5), joint_values_({0.0, 0.0, 0.0})
{
  nh_ = rclcpp::Node::make_shared("keyboard_input");

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>(JOINT_TOPIC, ROS_QUEUE_SIZE);
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  KeyboardInterface keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardInterface::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardInterface::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::thread{ [this]() { return spin(); } }.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to drive around");
  puts("");
  puts("Use A|D keys to move camera left|right");
  puts("Use W|S keys to move camera up|down");
  puts("");
  puts("'Q' to quit.");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    auto joint_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();

    joint_msg->data.resize(3);
    std::fill(joint_msg->data.begin(), joint_msg->data.end(), 0.0);
    // Use read key-press
    switch (c)
    {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg->angular.z = 0.5;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg->angular.z = -0.5;
        publish_twist = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg->linear.x = 0.5;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg->linear.x = -0.5;
        publish_twist = true;
        break;
      case KEYCODE_A:
        RCLCPP_DEBUG(nh_->get_logger(), "A");
        joint_values_[0] += joint_delta_;
        publish_joint = true;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(nh_->get_logger(), "D");
        joint_values_[0] -= joint_delta_;
        publish_joint = true;
        break;
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "W");
        joint_values_[2] -= joint_delta_;
        publish_joint = true;
        break;
      case KEYCODE_S:
        RCLCPP_DEBUG(nh_->get_logger(), "S");
        joint_values_[2] += joint_delta_;
        publish_joint = true;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        return 0;
      default:
        RCLCPP_DEBUG(nh_->get_logger(), "Unknown key: %d", c);
        break;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist)
    {
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
    else if (publish_joint)
    {
      joint_msg->data[0] = joint_values_[0];
      joint_msg->data[1] = joint_values_[1];
      joint_msg->data[2] = joint_values_[2];
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }

  return 0;
}