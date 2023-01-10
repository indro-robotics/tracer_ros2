/*
 * tracer_messenger.hpp
 *
 * Created on: Jun 14, 2022 16:37
 * Description:
 *
 * Copyright (c) 2022 tx (tx)
 */

#ifndef TRACER_MESSENGER_HPP
#define TRACER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "interfaces/msg/tracer_status.hpp"
#include "interfaces/msg/tracer_light_cmd.hpp"

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
template <typename TracerType>
class TracerMessenger {
 public:
  TracerMessenger(std::shared_ptr<TracerType> tracer, rclcpp::Node *node)
      : tracer_(tracer), node_(node) {}

  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }

  void SetSimulationMode(int loop_rate) {
    simulated_robot_ = true;
    sim_control_rate_ = loop_rate;
  }

  void SetupSubscription() {
    // odometry publisher
    odom_pub_ =
        node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
    status_pub_ = node_->create_publisher<interfaces::msg::TracerStatus>(
        "/tracer_status", 10);

    // cmd subscriber
    motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "mux/cmd_vel", 5,
        std::bind(&TracerMessenger::TwistCmdCallback, this,
                  std::placeholders::_1));
    light_cmd_sub_ = node_->create_subscription<interfaces::msg::TracerLightCmd>(
        "/light_control", 5,
        std::bind(&TracerMessenger::LightCmdCallback, this,
                  std::placeholders::_1));
    battery_status_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
        "/base/battery", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }
  void PublishBatteryStatus(){
    current_time_ = node_->get_clock()->now();

    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }

    auto state = tracer_->GetRobotState();
    auto actuator = tracer_->GetActuatorState();
    sensor_msgs::msg::BatteryState battery_msg;
    battery_msg.header.stamp = current_time_;
    battery_msg.header.frame_id = "battery";
    battery_msg.current = (actuator.actuator_hs_state[0].current + actuator.actuator_hs_state[1].current)/2;
    // if battery voltage is below 35 V, the max voltage is 29.4
    // if battery voltage is above 35 V, the max voltage is 54.6V
    // Min battery voltage is 20V
    // Min battery voltage is 40V if battery voltage is above 30 V
    battery_msg.voltage = state.system_state.battery_voltage; 
    if (battery_msg.voltage < 35) {
      battery_msg.percentage = (battery_msg.voltage - 23) / 6.4; // Current voltage - min, divided by difference between max and min
      battery_msg.design_capacity = 30; // 30Ah
    } else {
      battery_msg.percentage = (battery_msg.voltage - 40) / 14.6;
      battery_msg.design_capacity = 60; // 60Ah
    }

    battery_status_pub_->publish(battery_msg);
  }

  void PublishStateToROS() {
    current_time_ = node_->get_clock()->now();

    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
    double dt = (current_time_ - last_time_).seconds();

    auto state = tracer_->GetRobotState();

    // publish tracer state message
    interfaces::msg::TracerStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.motion_state.linear_velocity;
    status_msg.angular_velocity = state.motion_state.angular_velocity;

    status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    status_msg.battery_voltage = state.system_state.battery_voltage;

    auto actuator = tracer_->GetActuatorState();

    for (int i = 0; i < 2; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
      status_msg.actuator_states[motor_id].pulse_count =
          actuator.actuator_hs_state[i].pulse_count;

      // actuator_ls_state
      motor_id = actuator.actuator_ls_state[i].motor_id;

      status_msg.actuator_states[motor_id].driver_voltage =
          actuator.actuator_ls_state[i].driver_voltage;
      status_msg.actuator_states[motor_id].driver_temperature =
          actuator.actuator_ls_state[i].driver_temp;
      status_msg.actuator_states[motor_id].motor_temperature =
          actuator.actuator_ls_state[i].motor_temp;
      status_msg.actuator_states[motor_id].driver_state =
          actuator.actuator_ls_state[i].driver_state;
    }

    status_msg.light_control_enabled = state.light_state.enable_cmd_ctrl;
    status_msg.front_light_state.mode = state.light_state.front_light.mode;
    status_msg.front_light_state.custom_value =
        state.light_state.front_light.custom_value;
    // status_msg.rear_light_state.mode = state.light_state.rear_light.mode;
    // status_msg.rear_light_state.custom_value =
    //     state.light_state.rear_light.custom_value;
    status_pub_->publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(state.motion_state, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

 private:
  std::shared_ptr<TracerType> tracer_;
  rclcpp::Node *node_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;
  bool battery_status_ = false;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<interfaces::msg::TracerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_status_pub_;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  rclcpp::Subscription<interfaces::msg::TracerLightCmd>::SharedPtr
      light_cmd_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  double covar_pose_position = 0.0;
  double covar_pose_twist    = 1.0; // Start with a bit of uncertainty in the
                                    // twist
  double covar_twist_linear  = 0.0;
  double covar_twist_angular = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!simulated_robot_) {
      SetTracerMotionCommand(msg);
    } else {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }
  void SetTracerMotionCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    tracer_->SetMotionCommand(msg->linear.x, msg->angular.z);
  }

  void LightCmdCallback(const interfaces::msg::TracerLightCmd::SharedPtr msg) {
    if (!simulated_robot_) {
      if (msg->cmd_ctrl_allowed) {
        LightCommandMessage cmd;

        switch (msg->front_mode)
        {
          case interfaces::msg::TracerLightCmd::LIGHT_CONST_OFF:
          {
            cmd.front_light.mode = CONST_OFF;
            break;
          }
          case interfaces::msg::TracerLightCmd::LIGHT_CONST_ON:
          {
            cmd.front_light.mode = CONST_ON;
            break;
          }
          case interfaces::msg::TracerLightCmd::LIGHT_BREATH:
          {
            cmd.front_light.mode = BREATH;
            break;
          }
          case interfaces::msg::TracerLightCmd::LIGHT_CUSTOM:
          {
            cmd.front_light.mode = CUSTOM;
            cmd.front_light.custom_value = msg->front_custom_value;
            break;
          }
        }

          // switch (msg->rear_mode)
          // {
          //   case interfaces::TracerLightCmd::LIGHT_CONST_OFF:
          //   {
          //       cmd.rear_light.mode = CONST_OFF;
          //       break;
          //   }
          //   case interfaces::TracerLightCmd::LIGHT_CONST_ON:
          //   {
          //       cmd.rear_light.mode = CONST_ON;
          //       break;
          //   }
          //   case interfaces::TracerLightCmd::LIGHT_BREATH:
          //   {
          //       cmd.rear_light.mode = BREATH;
          //       break;
          //   }
          //   case interfaces::TracerLightCmd::LIGHT_CUSTOM:
          //   {
          //       cmd.rear_light.mode = CUSTOM;
          //       cmd.rear_light.custom_value = msg->rear_custom_value;
          //       break;
          //   }
          // }
        tracer_->SetLightCommand(cmd.front_light.mode,cmd.front_light.custom_value);
        } 
      } else {
      std::cout << "simulated robot received light control cmd" << std::endl;
    }
  }
  

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
    // perform numerical integration to get an estimation of pose
    double linear_speed = msg.linear_velocity;
    double angular_speed = msg.angular_velocity;

    // if (std::is_base_of<TracerMiniOmniRobot, TracerType>::value) {
    //   lateral_speed = msg.lateral_velocity;
    // } else {
    //   lateral_speed = 0;
    // }

    double d_x = linear_speed * std::cos(theta_) * dt;
    double d_y = linear_speed * std::sin(theta_) * dt;
    double d_theta = angular_speed * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    // Covariance
    covar_pose_position += 0.01 * std::sqrt(d_x*d_x + d_y*d_y);
    covar_pose_twist    += 0.01 * std::fabs(d_theta);
    covar_twist_linear  = 0.1 * std::fabs(linear_speed);
    covar_twist_angular = 0.1 * std::fabs(angular_speed);

    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed;
    
    odom_msg.pose.covariance = {covar_pose_position, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, covar_pose_position, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, covar_pose_position, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, covar_pose_twist, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, covar_pose_twist, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, covar_pose_twist};
    odom_msg.twist.covariance = {covar_twist_linear, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, covar_twist_linear, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, covar_twist_linear, 0.0, 0.0, 0.0,   
                                0.0, 0.0, 0.0, covar_twist_angular, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, covar_twist_angular, 0.0,   
                                0.0, 0.0, 0.0, 0.0, 0.0, covar_twist_angular}; 
    odom_pub_->publish(odom_msg);
  }
};
}  // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */
