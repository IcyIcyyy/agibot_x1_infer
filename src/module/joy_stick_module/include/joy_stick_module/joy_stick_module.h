// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#pragma once
#include <future>
#include "aimrt_module_cpp_interface/module_base.h"
#include "joy_stick_module/joy.h"
#include "joy_stick_module/joy_vel_limiter.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_srvs/srv/empty.hpp"//5.28调用服务头文件
#include "omnipicker_interfaces/srv/omni_picker_control.hpp"//5.28
#include <atomic>
namespace xyber_x1_infer::joy_stick_module {

struct FloatPub {
  std::string topic_name;
  std::vector<uint8_t> buttons;
  aimrt::channel::PublisherRef pub;
};

struct TwistPub {
  std::string topic_name;
  std::vector<uint8_t> buttons;
  std::map<std::string, uint8_t> axis;
  aimrt::channel::PublisherRef pub;
  aimrt::channel::PublisherRef pub_limiter;
};

struct ServiceClient {
  std::string service_name;
  std::string interface_type;
  std::vector<uint8_t> buttons;
};

class JoyStickModule : public aimrt::ModuleBase {
 public:
  JoyStickModule() = default;
  ~JoyStickModule() override;

  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "JoyStickModule"};
  }
  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;
  //void IsPathClearCallback(const std::shared_ptr<const std_msgs::msg::Bool>& msg);//5.20添加回调函数

 private:
  auto GetLogger() { return core_.GetLogger(); }
  void MainLoop();

 private:
  aimrt::CoreRef core_;

  std::shared_ptr<Joy> joy_;
  std::atomic_bool run_flag_ = true;
  std::promise<void> stop_sig_;

  aimrt::executor::ExecutorRef executor_;
  std::vector<FloatPub> float_pubs_;
  std::vector<TwistPub> twist_pubs_;
  std::vector<ServiceClient> srv_clients_;
  std::shared_ptr<JoyVelLimiter> limiter_ = nullptr;
  std::atomic_bool path_is_clear_ = false;//5.20
  bool gripper_open_ = false;  // 5.28
  bool path_was_blocked_{false};//5.21
  std::atomic_bool publish_zero_twist_{false};  //5.21添加松开LB也能发布0速
  bool control_engaged_ = false; //6.17
  bool was_moving_ = false;//记录移动标志位6.19

  //rclcpp变量添加
  // --- 你需要添加的 ROS 2 订阅相关变量 ---
  std::shared_ptr<rclcpp::Node> ros2_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ros2_sub_;
  std::thread ros2_spin_thread_;
  //end
  std::vector<bool> last_button_pressed_states_;  // 记录每个srv_client按键上一次状态6.6
  uint32_t freq_{};
};

}  // namespace xyber_x1_infer::joy_stick_module
