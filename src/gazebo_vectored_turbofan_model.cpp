#include "gazebo_vectored_turbofan_model.h"

#include <algorithm>

#include <boost/bind.hpp>

namespace gazebo {

void GazeboVectoredTurbofanModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // 函数作用：
  // - 解析 SDF 配置；
  // - 初始化 Gazebo transport；
  // - 建立命令订阅和可选可视化发布；
  // - 注册 OnUpdate 回调。
  // 输入：_model（模型指针）、_sdf（插件参数）。
  // 输出：无（通过成员变量完成初始化）。

  // 保存模型句柄，后续用于获取 link 和模型名。
  model_ = _model;

  // 读取可选 robot namespace。
  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  // 读取受力 link 名称（必填）。
  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzthrow("[gazebo_vectored_turbofan_model] Please specify a linkName.");
  }

  // 校验 link 是否存在，避免后续空指针施力。
  link_ = model_->GetLink(link_name_);
  if (!link_) {
    gzthrow("[gazebo_vectored_turbofan_model] Couldn't find specified link \"" << link_name_ << "\".");
  }

  // 读取命令 topic、通道映射和执行器参数。
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "forceVisualTopic", force_visual_topic_, force_visual_topic_);
  getSdfParam<double>(_sdf, "forceVisualPubInterval", force_visual_pub_interval_, force_visual_pub_interval_);
  getSdfParam<int>(_sdf, "thrustChannel", thrust_channel_, thrust_channel_);
  getSdfParam<int>(_sdf, "vectorXChannel", vector_x_channel_, vector_x_channel_);
  getSdfParam<int>(_sdf, "vectorYChannel", vector_y_channel_, vector_y_channel_);
  getSdfParam<double>(_sdf, "maxThrust", max_thrust_, max_thrust_);
  getSdfParam<double>(_sdf, "maxVectorAngleRad", max_vector_angle_rad_, max_vector_angle_rad_);
  getSdfParam<double>(_sdf, "timeConstant", time_constant_, time_constant_);

  if (_sdf->HasElement("forceApplicationPoint")) {
    force_application_point_ = _sdf->GetElement("forceApplicationPoint")->Get<ignition::math::Vector3d>();
  }

  // 初始化 Gazebo transport 节点并订阅指令。
  // topic 约定："~/" + model_name + sub_topic。
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
      "~/" + model_->GetName() + command_sub_topic_,
      &GazeboVectoredTurbofanModel::CommandCallback, this);

  // 可选：发布力矢量用于 GUI 力箭头可视化（配合 libForceVisual.so）。
  if (!force_visual_topic_.empty()) {
    force_visual_pub_ = node_handle_->Advertise<physics_msgs::msgs::Force>("~/" + force_visual_topic_);
  }

  // 注册每步更新回调（WorldUpdateBegin）。
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboVectoredTurbofanModel::OnUpdate, this, _1));
}

void GazeboVectoredTurbofanModel::CommandCallback(const CommandMotorSpeedPtr &msg) {
  // 函数作用：
  // - 从 CommandMotorSpeed 中读取本喷口绑定的 3 路输入；
  // - 完成归一化值到物理量的转换。
  // 输入：msg（完整执行器数组）。
  // 输出：无（更新 thrust_cmd_ / vector_x_cmd_ / vector_y_cmd_）。

  // 安全取通道值：越界时返回 0，保证鲁棒性。
  auto channel = [&](int index) {
    if (index < 0 || index >= static_cast<int>(msg->motor_speed_size())) {
      return 0.0;
    }
    return static_cast<double>(msg->motor_speed(index));
  };

  // 推力通道：归一化 [0, 1] -> 物理推力 [0, max_thrust] (N)。
  thrust_cmd_ = ignition::math::clamp(channel(thrust_channel_), 0.0, 1.0) * max_thrust_;

  // 偏转通道：归一化 [-1, 1] -> 偏角 [-max_angle, +max_angle] (rad)。
  vector_x_cmd_ = ignition::math::clamp(channel(vector_x_channel_), -1.0, 1.0) * max_vector_angle_rad_;
  vector_y_cmd_ = ignition::math::clamp(channel(vector_y_channel_), -1.0, 1.0) * max_vector_angle_rad_;
}

void GazeboVectoredTurbofanModel::OnUpdate(const common::UpdateInfo &_info) {
  // 函数作用：
  // - 根据 dt 更新一阶惯性状态；
  // - 将推力与偏角转换为三维力矢量；
  // - 向 Gazebo 物理引擎施加力；
  // - 按需发布力可视化消息。
  // 输入：_info（包含当前仿真时间）。
  // 输出：无（通过 AddRelativeForceAtRelativePosition 和 Publish 产生效果）。

  // 计算仿真步长 dt。
  const double sim_time = _info.simTime.Double();
  const double dt = sim_time - prev_sim_time_;
  prev_sim_time_ = sim_time;

  if (dt <= 0.0) {
    return;
  }

  // 一阶惯性离散化：x(k+1)=x(k)+alpha*(u-x(k))。
  // alpha 限幅到 [0,1]，避免 time_constant 太小带来的数值异常。
  const double alpha = ignition::math::clamp(dt / std::max(time_constant_, 1e-4), 0.0, 1.0);
  thrust_state_ += alpha * (thrust_cmd_ - thrust_state_);
  vector_x_state_ += alpha * (vector_x_cmd_ - vector_x_state_);
  vector_y_state_ += alpha * (vector_y_cmd_ - vector_y_state_);

  // 根据两轴偏角构造喷流方向（默认喷流沿机体系 +X）。
  ignition::math::Quaterniond q_vec(vector_y_state_, vector_x_state_, 0.0);
  const ignition::math::Vector3d thrust_dir = q_vec.RotateVector(ignition::math::Vector3d(0.0, 0.0, 1.0));

  // 标量推力 -> 三维力矢量。
  const ignition::math::Vector3d body_force = thrust_state_ * thrust_dir;

  // 在相对作用点施加相对力，进入 Gazebo 物理求解器。
  link_->AddLinkForce(body_force, force_application_point_);

  // 可选：按固定周期发布力矢量，供 ForceVisual 绘制箭头。
  if (force_visual_pub_ && (sim_time - last_force_visual_pub_time_ >= force_visual_pub_interval_)) {
    msgs::Vector3d *force_center_msg = new msgs::Vector3d;
    force_center_msg->set_x(force_application_point_.X());
    force_center_msg->set_y(force_application_point_.Y());
    force_center_msg->set_z(force_application_point_.Z());

    msgs::Vector3d *force_vector_msg = new msgs::Vector3d;
    force_vector_msg->set_x(body_force.X());
    force_vector_msg->set_y(body_force.Y());
    force_vector_msg->set_z(body_force.Z());

    physics_msgs::msgs::Force force_msg;
    force_msg.set_allocated_center(force_center_msg);
    force_msg.set_allocated_force(force_vector_msg);
    force_visual_pub_->Publish(force_msg);

    last_force_visual_pub_time_ = sim_time;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboVectoredTurbofanModel);

} // namespace gazebo
