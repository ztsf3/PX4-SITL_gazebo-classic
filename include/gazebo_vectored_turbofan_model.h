#pragma once

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>

#include "CommandMotorSpeed.pb.h"
#include "Force.pb.h"
#include "common.h"

namespace gazebo {

/// Gazebo transport 回调接收到的 CommandMotorSpeed 消息指针类型。
using CommandMotorSpeedPtr = const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed>;

/**
 * @brief 单个“矢量涡喷执行器”Gazebo 插件。
 *
 * 功能概览：
 * 1) 从 CommandMotorSpeed 消息中读取 3 路通道（推力、X 偏角、Y 偏角）；
 * 2) 将归一化输入映射为物理量（N 与 rad）；
 * 3) 用一阶惯性更新执行器状态，避免力突变；
 * 4) 把“标量推力 + 偏角”转换为三维力矢量施加到 link；
 * 5) 可选发布 Force 消息做力可视化。
 */
class GazeboVectoredTurbofanModel : public ModelPlugin {
public:
  /// 默认构造函数：成员使用类内默认值初始化。
  GazeboVectoredTurbofanModel() = default;

  /// 默认析构函数：智能指针负责资源回收。
  ~GazeboVectoredTurbofanModel() override = default;

  /**
   * @brief 插件加载入口（Gazebo 生命周期函数）。
   *
   * 作用：
   * - 解析 SDF 参数；
   * - 查找目标 link；
   * - 初始化 transport 订阅/发布；
   * - 注册世界更新回调。
   *
   * 输入：
   * - _model：当前模型指针。
   * - _sdf：插件对应的 SDF 参数节点。
   *
   * 输出：无返回值。
   */
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /**
   * @brief 每个仿真步执行一次的更新函数。
   *
   * 作用：
   * - 依据 dt 执行一阶惯性更新；
   * - 计算三维力方向和大小；
   * - 将力施加到物理引擎；
   * - 需要时发布力可视化消息。
   *
   * 输入：
   * - _info：当前仿真步信息（含 simTime）。
   *
   * 输出：无返回值。
   */
  void OnUpdate(const common::UpdateInfo &_info);

  /**
   * @brief 命令消息回调函数。
   *
   * 作用：
   * - 从 CommandMotorSpeed 指定下标读取 3 路输入；
   * - 对输入执行限幅与缩放；
   * - 更新内部命令缓存（thrust_cmd_ / vector_x_cmd_ / vector_y_cmd_）。
   *
   * 输入：
   * - msg：Gazebo transport 收到的 CommandMotorSpeed 消息。
   *
   * 输出：无返回值。
   */
  void CommandCallback(const CommandMotorSpeedPtr &msg);

  // -------------------- Gazebo 运行时句柄 --------------------
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  transport::NodePtr node_handle_;
  transport::SubscriberPtr command_sub_;
  transport::PublisherPtr force_visual_pub_;

  // -------------------- SDF 配置参数 --------------------
  std::string namespace_;
  std::string link_name_;
  std::string command_sub_topic_{"/gazebo/command/motor_speed"};
  std::string force_visual_topic_{};
  double force_visual_pub_interval_{0.05};

  int thrust_channel_{0};
  int vector_x_channel_{0};
  int vector_y_channel_{0};

  double max_thrust_{30.0};
  double max_vector_angle_rad_{0.5235987756};  // 30 deg
  double time_constant_{0.03};

  // -------------------- 命令与状态 --------------------
  // cmd_：回调收到的目标值；state_：一阶惯性后的执行值。
  double thrust_cmd_{0.0};
  double vector_x_cmd_{0.0};
  double vector_y_cmd_{0.0};

  double thrust_state_{0.0};
  double vector_x_state_{0.0};
  double vector_y_state_{0.0};

  ignition::math::Vector3d force_application_point_{0.0, 0.0, 0.0};

  // 时间戳缓存：用于 dt 计算与可视化节流。
  double prev_sim_time_{0.0};
  double last_force_visual_pub_time_{0.0};
  // 上一次真正发布给可视化插件的力向量。
  // 用它和当前 body_force 做比较，判断“变化是否足够大”。
  ignition::math::Vector3d last_visual_force_{0.0, 0.0, 0.0};

  // 标记是否已经发布过至少一次可视化消息。
  // 第一次没有历史值可比较，所以要强制发布一次。
  bool has_last_visual_force_{false};
};

} // namespace gazebo
