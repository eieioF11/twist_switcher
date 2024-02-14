#pragma once
#include <iostream>
#include <rclcpp/clock.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
//ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class TwistSwitcher : public rclcpp::Node
{
public:
  TwistSwitcher(const rclcpp::NodeOptions & options) : TwistSwitcher("", options) {}
  TwistSwitcher(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("twist_bridge_node", name_space, options)
  {
    RCLCPP_INFO(this->get_logger(), "start twist_bridge_node");
    std::vector<std::string> IN_TWIST_TOPICS = param<std::vector<std::string>>(
      "twist_bridge.topic_name.input_twist",
      std::vector<std::string>{"/input/twist1", "/input/twist2"});
    std::string OUT_TWIST_TOPIC =
      param<std::string>("twist_bridge.topic_name.output_twist", "/output/twist");
    double ROOP_RATE = param<double>("twist_bridge.roop_rate", 0.0001);
    index_ = 0;
    // publisher
    twsit_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(OUT_TWIST_TOPIC, rclcpp::QoS(10));
    // subscriber
    switch_sub_ = this->create_subscription<std_msgs::msg::String>(
      "twist_bridge/switch", rclcpp::QoS(10), [&](const std_msgs::msg::String::SharedPtr msg) {
        if (twist_list_.find(msg->data) != twist_list_.end())
          index_ = twist_list_.at(msg->data);
        else
          RCLCPP_WARN(this->get_logger(), "Invalid topic name: %s", msg->data.c_str());
      });
    int i = 0;
    for (auto in_topic_name : IN_TWIST_TOPICS) {
      twists_.push_back(stop());
      get_twists_.push_back(false);
      auto twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        in_topic_name, rclcpp::QoS(10), [&, i](const geometry_msgs::msg::Twist::SharedPtr msg) {
          twists_[i] = *msg;
          get_twists_[i] = true;
        });
      twist_subs_.push_back(twist_sub);
      twist_list_.insert(std::make_pair(in_topic_name, i));
      i++;
    }
    // service
    ems_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "twist_bridge/ems", [&](
                            const std_srvs::srv::SetBool::Request::SharedPtr req,
                            const std_srvs::srv::SetBool::Response::SharedPtr res) {
        ems_ = req->data;
        res->success = true;
        res->message = ems_ ? "Emergency stop ON" : "Emergency stop OFF";
        if (ems_) 
          twsit_pub_->publish(stop());
      });

    timer_ = this->create_wall_timer(1s * ROOP_RATE, [&]() {
      if (get_twists_[index_]) {
        geometry_msgs::msg::Twist output_twist = stop();
        if (!ems_)
          output_twist = twists_[index_];
        else
          RCLCPP_WARN(this->get_logger(), "Emergency stop ON");
        twsit_pub_->publish(output_twist);
        get_twists_[index_] = false;
      }
    });
  }

private:
  bool ems_;
  int index_;
  std::unordered_map<std::string, int> twist_list_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twsit_pub_;
  // subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr switch_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> twist_subs_;
  // service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ems_srv_;
  // twists
  std::vector<geometry_msgs::msg::Twist> twists_;
  std::vector<bool> get_twists_;

  template <class T>
  T param(const std::string & name, const T & def)
  {
    T value;
    declare_parameter(name, def);
    get_parameter(name, value);
    return value;
  }

  geometry_msgs::msg::Twist stop()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    return twist;
  }
};