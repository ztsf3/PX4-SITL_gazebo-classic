#include <cmath>
#include <csignal>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>

#include "CommandMotorSpeed.pb.h"

// 全局运行标志。
// 作用：让 Ctrl+C 能优雅退出主循环。
static volatile bool g_run = true;

// SIGINT 处理函数。
// 作用：收到 Ctrl+C 时把 g_run 置为 false，主循环下一次检查时退出。
void OnSigint(int)
{
  g_run = false;
}

int main(int argc, char **argv)
{
  // 默认 topic。
  // 说明：真实 topic 应以 `gz topic -l` 查到的完整路径为准。
  std::string topic = "/gazebo/default/tailsitter/gazebo/command/motor_speed";

  // 默认基础推力，取值按你的插件约定为 0~1。
  float base_thrust = 0.6f;

  // 默认发布频率。
  double rate_hz = 20.0;

  // 允许命令行覆盖默认参数。
  if (argc > 1) topic = argv[1];
  if (argc > 2) base_thrust = std::stof(argv[2]);
  if (argc > 3) rate_hz = std::stod(argv[3]);

  // 注册 Ctrl+C 处理函数。
  std::signal(SIGINT, OnSigint);

  // 启动 Gazebo standalone client。
  // 依据：Gazebo client API 提供 setup()/shutdown() 用于建立与运行中仿真的连接。
  gazebo::client::setup(argc, argv);

  // 创建 transport 节点。
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // 在指定 topic 上创建 publisher，消息类型为 CommandMotorSpeed。
  auto pub = node->Advertise<mav_msgs::msgs::CommandMotorSpeed>(topic);

  std::cout << "[INFO] Waiting for subscriber on: " << topic << std::endl;
  pub->WaitForConnection();
  std::cout << "[INFO] Connected." << std::endl;

  // 根据频率计算周期。
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);

  // 仿真时间参数，仅用于生成输入波形。
  double t = 0.0;

  while (g_run) {
    mav_msgs::msgs::CommandMotorSpeed msg;

    // 先构造 16 通道全零数组。
    // 这样做的原因是：插件读取越界时虽然可以自保，但测试输入最好显式完整。
    for (int i = 0; i < 16; ++i) {
      msg.add_motor_speed(0.0f);
    }

    // 0~3: 四个喷口基础推力。
    // 目的：先给模型一个稳定的整体受力。
    for (int i = 0; i < 4; ++i) {
      msg.set_motor_speed(i, base_thrust);
    }

    // 4/8: 让 jet_0 做二维矢量摆动。
    // 对应关系来自当前 SDF：
    // jet_0.thrustChannel  = 0
    // jet_0.vectorXChannel = 4
    // jet_0.vectorYChannel = 8
    msg.set_motor_speed(4, static_cast<float>(std::sin(M_PI * t)));
    msg.set_motor_speed(8, static_cast<float>(std::cos(M_PI * t)));
    msg.set_motor_speed(5, static_cast<float>(std::sin(M_PI * t)));
    msg.set_motor_speed(9, static_cast<float>(std::cos(M_PI * t)));
    msg.set_motor_speed(6, static_cast<float>(std::sin(M_PI * t)));
    msg.set_motor_speed(10, static_cast<float>(std::cos(M_PI * t)));
    msg.set_motor_speed(7, static_cast<float>(std::sin(M_PI * t)));
    msg.set_motor_speed(11, static_cast<float>(std::cos(M_PI * t)));

    // 发布消息。
    pub->Publish(msg);

    // 打印当前输入，便于人工确认波形是否正常。
    std::cout
      << "t=" << t
      << " thrust=" << base_thrust
      << " ch4=" << msg.motor_speed(4)
      << " ch8=" << msg.motor_speed(8)
      << std::endl;

    std::this_thread::sleep_for(period);
    t += 1.0 / rate_hz;
  }

  // 清理 Gazebo client。
  gazebo::client::shutdown();
  return 0;
}
