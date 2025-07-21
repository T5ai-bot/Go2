/*
 *  Copyright (c) 2019-2020, Juan Miguel Jimeno
 *  All rights reserved.
 *
 *  本文件仅在原始开源协议基础上 **添加中英双语注释**，
 *  不修改任何业务逻辑或版权条款，遵循 BSD-3-Clause 许可证。
 */

#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

/* ────────────────────────────────────────────────────────────────
 *  头文件区  |  Includes
 * ────────────────────────────────────────────────────────────────*/
#include "rclcpp/rclcpp.hpp"                  // ROS2 节点基类 / ROS2 node API

/* ---- CHAMP & ROS2 消息类型 ---- */
#include <champ_msgs/msg/joints.hpp>          // 关节角度数组
#include <champ_msgs/msg/pose.hpp>            // 机身期望位姿
#include <champ_msgs/msg/point_array.hpp>     // 足端点集，可用于调试
#include <champ_msgs/msg/contacts_stamped.hpp>// 足端接触布尔值

/* ---- CHAMP 控制算法组件 ---- */
#include <champ/body_controller/body_controller.h>
#include <champ/utils/urdf_loader.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/kinematics/kinematics.h>

/* ---- 几何 / TF 工具 ---- */
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

/* ---- 低层指令消息（可接硬件或 Gazebo） ---- */
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

/* ────────────────────────────────────────────────────────────────
 *  核心控制类 | Main Control Class
 * ────────────────────────────────────────────────────────────────*/
class QuadrupedController : public rclcpp::Node
{
    /* ===== ROS2 订阅者 | Subscribers ===== */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
        // /cmd_vel：来自导航或遥控器的速度指令
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscription_;
        // /body_cmd：机身姿态指令（选配）

    /* ===== ROS2 发布者 | Publishers ===== */
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_publisher_;
        // /joint_trajectory：发往 Gazebo / 硬件驱动器的关节目标
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
        // /joint_states：调试或闭环控制用
    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_publisher_;
        // /foot_contacts：发布足端是否着地

    /* ===== 循环定时器 | Main Control Timer ===== */
    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::Clock clock_;                         // 提供精准时间戳

    /* ===== 输入缓存 | Cached Set-points ===== */
    champ::Velocities req_vel_;                  // 用户期望线/角速度
    champ::Pose req_pose_;                       // 用户期望机身姿态

    /* ===== 步态与机器人模型 | Gait & Model ===== */
    champ::GaitConfig gait_config_;              // 参见 gait_config.yaml

    champ::QuadrupedBase      base_;             // 机器人整体状态管理
    champ::BodyController     body_controller_;  // 机身姿态控制（平衡、倾角补偿）
    champ::LegController      leg_controller_;   // 足端轨迹 + 支撑摆动逻辑
    champ::Kinematics         kinematics_;       // 解析几何逆运动学

    std::vector<std::string> joint_names_;       // 关节命名，保持与 URDF 对齐

    /* ===== 运行时开关 | Run-time Flags ===== */
    bool publish_foot_contacts_; // 是否广播足端接触
    bool publish_joint_states_;  // 是否广播关节状态
    bool publish_joint_control_; // 是否发送 trajectory / motor 命令
    bool in_gazebo_;             // true=仿真模式；false=实机

    /* ── 内部方法 | Private Helpers ── */
    void controlLoop_();                       // 控制循环回调（由 loop_timer_ 触发）
    
    void publishJoints_(float target_joints[12]);      // 统一关节发布接口
    void publishFootContacts_(bool foot_contacts[4]);  // 足端接触发布

    /* ── 订阅回调 | Topic Call-backs ── */
    void cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg);

public:
    /* ===== 构造函数 | Constructor ===== */
    QuadrupedController();  // 在 .cpp 中实现：加载参数、注册话题、启动定时器
};

#endif  // QUADRUPED_CONTROLLER_H
