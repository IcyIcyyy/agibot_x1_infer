// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#include "joy_stick_module/joy_stick_module.h"

#include <yaml-cpp/yaml.h>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

// #include "Empty.aimrt_rpc.srv.h"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/bool.hpp>  // === 修改开始 ===5.19
//#include "omnipicker_interfaces/srv/omni_picker_control.hpp"//5.28



namespace xyber_x1_infer::joy_stick_module {

JoyStickModule::~JoyStickModule() {
  run_flag_ = false; // 如果你用 run_flag_ 控制线程退出，可以先关掉
  if (ros2_node_) {
    ros2_node_.reset();  // 释放 ROS 2 节点资源
  }
  if (ros2_spin_thread_.joinable()) {
    ros2_spin_thread_.join();  // 等待线程安全退出
  }
}


bool JoyStickModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;
  joy_ = std::make_shared<Joy>();

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path.data());
      freq_ = cfg_node["freq"].as<uint32_t>();

      // prepare executor
      executor_ = core_.GetExecutorManager().GetExecutor("joy_stick_pub_thread");
      AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor 'joy_stick_pub_thread'.");

      if (cfg_node["float_pubs"]) {
        for (const auto& pub : cfg_node["float_pubs"]) {
          FloatPub publisher;
          publisher.topic_name = pub["topic_name"].as<std::string>();
          publisher.buttons = pub["buttons"].as<std::vector<uint8_t>>();
          publisher.pub = core_.GetChannelHandle().GetPublisher(publisher.topic_name);
          aimrt::channel::RegisterPublishType<std_msgs::msg::Float32>(publisher.pub);
          float_pubs_.push_back(std::move(publisher));
        }
      }
      if (cfg_node["twist_pubs"]) {
        for (const auto& pub : cfg_node["twist_pubs"]) {
          TwistPub publisher;
          publisher.topic_name = pub["topic_name"].as<std::string>();
          publisher.buttons = pub["buttons"].as<std::vector<uint8_t>>();
          publisher.axis = pub["axis"].as<std::map<std::string, uint8_t>>();
          publisher.pub = core_.GetChannelHandle().GetPublisher(publisher.topic_name);
          aimrt::channel::RegisterPublishType<geometry_msgs::msg::Twist>(publisher.pub);
          if (pub["velocity_limit_lb"] && pub["velocity_limit_ub"]) {
            publisher.pub_limiter = core_.GetChannelHandle().GetPublisher(publisher.topic_name + "_limiter");
            aimrt::channel::RegisterPublishType<geometry_msgs::msg::Twist>(publisher.pub_limiter);

            auto lb = pub["velocity_limit_lb"].as<std::vector<double>>();
            auto ub = pub["velocity_limit_ub"].as<std::vector<double>>();
            array_t lb_array = Eigen::Map<array_t>(lb.data(), lb.size());
            array_t ub_array = Eigen::Map<array_t>(ub.data(), ub.size());
            limiter_ = std::make_shared<JoyVelLimiter>(pub["axis"].size(), 1.0 / freq_, lb_array, ub_array);
          }
          twist_pubs_.push_back(std::move(publisher));
        }
      }
      if (cfg_node["rpc_clients"]) {
        for (const auto& rpc : cfg_node["rpc_clients"]) {
          ServiceClient clent;
          clent.service_name = rpc["service_name"].as<std::string>();
          clent.buttons = rpc["buttons"].as<std::vector<uint8_t>>();
          clent.interface_type = rpc["interface_type"].as<std::string>();
          srv_clients_.push_back(clent);
          // TODO: prepare rpc
          // std_srvs::srv::RegisterEmptyClientFunc(core_.GetRpcHandle(), clent.service_name);
        }
      }
    }
//5.20修改基于rclcpp的订阅
    // 创建 ROS 2 节点
//rclcpp::init(0, nullptr);
ros2_node_ = std::make_shared<rclcpp::Node>("joy_stick_path_clear_node");

// 订阅 /is_path_clear
ros2_sub_ = ros2_node_->create_subscription<std_msgs::msg::Bool>(
    "/is_path_clear", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      this->path_is_clear_ = msg->data;
      std::cout << "[rclcpp] Received /is_path_clear: " << msg->data << std::endl;
    });
// 用 spin_once 线程轮询
ros2_spin_thread_ = std::thread([this]() {
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(ros2_node_);
    loop_rate.sleep();
  }
});
//修改结束

    AIMRT_INFO("Init succeeded.");
    return true;
  } 
  catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }
}
bool JoyStickModule::Start() {
  executor_.Execute([this]() { MainLoop(); });

  AIMRT_INFO("Started succeeded.");
  return true;
}

void JoyStickModule::Shutdown() {
  run_flag_ = false;
  stop_sig_.get_future().wait();
  
// 停止 ROS 2 rclcpp_5.20
  if (ros2_spin_thread_.joinable()) {
    rclcpp::shutdown();
    ros2_spin_thread_.join();
  }
//end
  AIMRT_INFO("Shutdown succeeded.");
}

void JoyStickModule::MainLoop() {
  // TODO: prepare rpc
  // std_srvs::srv::EmptySyncProxy rpc_proxy(core_.GetRpcHandle());
  std_msgs::msg::Float32 button_msgs;
  geometry_msgs::msg::Twist vel_msgs;
  
  while (run_flag_.load()) {
    JoyStruct joy_data;
    joy_->GetJoyData(joy_data);

    //6.6he上升沿调用
    // 插入你的这段代码，完整拷贝进来即可
    //6.20调试
// for (size_t i = 0; i < srv_clients_.size(); ++i) {
//   const auto &srv_client = srv_clients_[i];

//   bool all_pressed = true;
//   for (auto button : srv_client.buttons) {
//     if (button >= joy_data.buttons.size() || !joy_data.buttons[button]) {
//       all_pressed = false;
//       break;
//     }
//   }

//   if (all_pressed && (i >= last_button_pressed_states_.size() || !last_button_pressed_states_[i])) {
//     if (last_button_pressed_states_.size() < srv_clients_.size()) {
//       last_button_pressed_states_.resize(srv_clients_.size(), false);
//     }

//     gripper_open_ = !gripper_open_;

//     if (srv_client.interface_type == "omnipicker_interfaces/srv/OmniPickerControl") {
//       auto client = ros2_node_->create_client<omnipicker_interfaces::srv::OmniPickerControl>("/" + srv_client.service_name);
//       if (!client->wait_for_service(std::chrono::milliseconds(100))) {
//         RCLCPP_WARN(ros2_node_->get_logger(), "Service not available: %s", srv_client.service_name.c_str());
//         continue;
//       }

//       auto request = std::make_shared<omnipicker_interfaces::srv::OmniPickerControl::Request>();
//       request->mode = "auto";
//       request->cmd = gripper_open_ ? "open" : "clamp";

//       client->async_send_request(request);
//       RCLCPP_INFO(ros2_node_->get_logger(), "Send OmniPickerControl: action = %s", request->cmd.c_str());
//     } else {
//       std::string cmd = "ros2 service call /" + srv_client.service_name + " " + srv_client.interface_type + " > /dev/null &";
//       system(cmd.c_str());
//       RCLCPP_INFO(ros2_node_->get_logger(), "Call service /%s", srv_client.service_name.c_str());
//     }
//   }

//   if (last_button_pressed_states_.size() < srv_clients_.size()) {
//     last_button_pressed_states_.resize(srv_clients_.size(), false);
//   }
//   last_button_pressed_states_[i] = all_pressed;
// }
// //6.6end
    for (auto float_pub : float_pubs_) {
      bool ret = true;
      for (auto button : float_pub.buttons) {
        ret &= joy_data.buttons[button];
      }
      if (ret) {
        aimrt::channel::Publish<std_msgs::msg::Float32>(float_pub.pub, button_msgs);
      }
    }

    for (auto twist_pub : twist_pubs_) {
      bool ret = true;
      geometry_msgs::msg::Twist vel_msgs{};//5.20添加初始化
      for (auto button : twist_pub.buttons) {
        ret &= joy_data.buttons[button];
      }

      if (limiter_) //5.21改加入到内层条件中&& path_is_clear_.load()//ret &&
      {
      bool lb_pressed = joy_data.buttons[4]; 
      bool any_button_pressed = lb_pressed;
      //bool any_button_pressed = flase;
  //     for (auto button : twist_pub.buttons) {
  //       any_button_pressed |= joy_data.buttons[button];
  //       control_engaged_ = true;
  // }
// === 判断是否已进入控制模式 ===
  if (lb_pressed) {
    control_engaged_ = true;
  }
  // === 修改标记：按下按钮才启用目标速度，否则设为 0 ===
  array_t target_pos;
  target_pos.resize(joy_data.axis.size());
  target_pos.setZero(); // 默认全为0
  geometry_msgs::msg::Twist vel_msgs{};
  int32_t idx = 0;
  if (any_button_pressed) {
    if (twist_pub.axis.find("linear-x") != twist_pub.axis.end()) {
      target_pos[idx] = joy_data.axis[twist_pub.axis["linear-x"]];
      vel_msgs.linear.x = target_pos[idx++];
    }
    if (twist_pub.axis.find("linear-y") != twist_pub.axis.end()) {
      target_pos[idx] = joy_data.axis[twist_pub.axis["linear-y"]];
      vel_msgs.linear.y = target_pos[idx++];
    }
    if (twist_pub.axis.find("linear-z") != twist_pub.axis.end()) {
      target_pos[idx] = joy_data.axis[twist_pub.axis["linear-z"]];
      vel_msgs.linear.z = target_pos[idx++];
    }
    if (twist_pub.axis.find("angular-x") != twist_pub.axis.end()) {
      target_pos[idx] = joy_data.axis[twist_pub.axis["angular-x"]];
      vel_msgs.angular.x = target_pos[idx++];
    }
    if (twist_pub.axis.find("angular-y") != twist_pub.axis.end()) {
      target_pos[idx] = joy_data.axis[twist_pub.axis["angular-y"]];
      vel_msgs.angular.y = target_pos[idx++];
    }
    if (twist_pub.axis.find("angular-z") != twist_pub.axis.end()) {
      target_pos[idx] = joy_data.axis[twist_pub.axis["angular-z"]];
      vel_msgs.angular.z = target_pos[idx++];
    }
     // 如果当前速度有移动（非零），标记为曾经移动过6.19
    was_moving_ = std::abs(vel_msgs.linear.x) > 1e-3 || std::abs(vel_msgs.linear.y) > 1e-3 || std::abs(vel_msgs.angular.z) > 1e-3;
  } 

  else if (control_engaged_&&was_moving_)//6.19加入移动标志位判断
  {
    //松开LB且之前有过移动才会进入原地踏步的模式
    // === ★修改点：松开 LB 后改为发布 x方向0.05 的速度，其余为0 ===
    if (twist_pub.axis.find("linear-x") != twist_pub.axis.end()) {
      target_pos[0] = 0.05000000000000001;  // 只设定 X 方向为 0.05
    }
    vel_msgs.linear.x = 0.05000000000000001;
    vel_msgs.linear.y = 0.0f;
    vel_msgs.linear.z = 0.0f;
    vel_msgs.angular.x = 0.0f;
    vel_msgs.angular.y = 0.0f;
    vel_msgs.angular.z = 0.0f;
  }
  else{
    was_moving_=false;//松开LB，未进入移动状态或速度已归0则不踏步6.19
  }

  aimrt::channel::Publish<geometry_msgs::msg::Twist>(twist_pub.pub, vel_msgs);//原始速度发布
  // === 修改标记：判断路径是否阻挡，如果是则强制目标速度为 0 ===
  if (!path_is_clear_) {
    target_pos.setZero();
    was_moving_ = false;//6.19
    if (!path_was_blocked_) {
      path_was_blocked_ = true;
    }
  } else if (path_was_blocked_) {
    limiter_->reset();//清除限制器内部状态
    path_was_blocked_ = false;
  }

  array_t cmd_vel = limiter_->update(target_pos);
  // === 构建限速后的 vel_msgs 并发布 ===
  idx = 0;
  if (twist_pub.axis.find("linear-x") != twist_pub.axis.end()) {
    vel_msgs.linear.x = cmd_vel[idx++];
  }
  if (twist_pub.axis.find("linear-y") != twist_pub.axis.end()) {
    vel_msgs.linear.y = cmd_vel[idx++];
  }
  if (twist_pub.axis.find("linear-z") != twist_pub.axis.end()) {
    vel_msgs.linear.z = cmd_vel[idx++];
  }
  if (twist_pub.axis.find("angular-x") != twist_pub.axis.end()) {
    vel_msgs.angular.x = cmd_vel[idx++];
  }
  if (twist_pub.axis.find("angular-y") != twist_pub.axis.end()) {
    vel_msgs.angular.y = cmd_vel[idx++];
  }
  if (twist_pub.axis.find("angular-z") != twist_pub.axis.end()) {
    vel_msgs.angular.z = cmd_vel[idx++];
  }

  aimrt::channel::Publish<geometry_msgs::msg::Twist>(twist_pub.pub_limiter, vel_msgs);

    }
    }
 
    for (auto srv_client : srv_clients_) {
      bool ret = true;
      for (auto button : srv_client.buttons) {
        ret &= joy_data.buttons[button];
      }
      if (ret) {
//         //6.6
//         if (srv_client.interface_type == "omnipicker_interfaces/srv/OmniPickerControl") {
//   // 防抖处理：只有状态变化时才调用服务
//   static bool last_gripper_state = false;
//   static bool current_gripper_state = false;

//   current_gripper_state = !last_gripper_state;  // 每次按下切换状态

//   auto client = ros2_node_->create_client<omnipicker_interfaces::srv::OmniPickerControl>("/" + srv_client.service_name);
//   if (!client->wait_for_service(std::chrono::milliseconds(100))) {
//     RCLCPP_WARN(ros2_node_->get_logger(), "Service not available: %s", srv_client.service_name.c_str());
//     continue;
//   }

//   auto request = std::make_shared<omnipicker_interfaces::srv::OmniPickerControl::Request>();
//   request->mode = "auto";
//   request->cmd = current_gripper_state ? "open" : "clamp";

//   auto result = client->async_send_request(request);
//   AIMRT_INFO("Send OmniPickerControl: action = %s", request->cmd.c_str());

//   last_gripper_state = current_gripper_state;
// } else {
//   // 兼容旧 Empty 类型服务
//   std::string cmd = "ros2 service call /" + srv_client.service_name + " " +
//                     srv_client.interface_type + " > /dev/null &";
//   int ret = system(cmd.data());
//   AIMRT_INFO("Call /%s", srv_client.service_name.c_str());
// }
// //6.6end

        // std::string cmd = "ros2 service call /" + srv_client.service_name + " " +
        //                   srv_client.interface_type + " > /dev/null &";
        // int ret = system(cmd.data());
        // AIMRT_INFO("Call /reset_word");
        // TODO: call rpc
        // rpc_proxy.SetServiceName("reset_world");
        // auto status = rpc_proxy.Empty(req, rsp);
        // if (status.OK()) {
        //   AIMRT_INFO("GetFooData success rsp: {}", std_srvs::srv::to_yaml(rsp));
        // } else {
        //   AIMRT_WARN("Call GetFooData failed, status: {}", status.ToString());
        // }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / freq_));
  }

  stop_sig_.set_value();
}
} // namespace xyber_x1_infer::joy_stick_module
