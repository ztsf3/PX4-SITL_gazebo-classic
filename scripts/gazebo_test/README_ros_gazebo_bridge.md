# ROS-Gazebo 通信节点（控制分配 -> CommandMotorSpeed）

该节点用于把 ROS 侧的控制分配输出转发到 Gazebo Classic 的 `CommandMotorSpeed` 话题，
用于驱动 `tailsitter.sdf.jinja` 中 `vectored_jet_*` 插件。

## 1. 输出目标话题

默认发布到：

```bash
/gazebo/default/tailsitter/gazebo/command/motor_speed
```

> 实际以 `gz topic -l` 为准，可用参数 `gazebo_topic` 覆盖。

## 2. 输入模式

### 模式 A（默认）：`std_msgs/Float32MultiArray`

- 参数 `use_px4_topics:=false`
- 订阅话题：`control_allocation_topic`（默认 `/control_allocation/actuator_setpoint`）
- `data[i]` 直接写入 `CommandMotorSpeed[motor_offset + i]`

### 模式 B：PX4 话题（需编译时找到 `px4_msgs`）

- 参数 `use_px4_topics:=true`
- 订阅：
  - `motors_topic`（默认 `/fmu/out/actuator_motors`）
  - `servos_topic`（可选，`use_servo_topic:=true` 时开启）
- 映射：
  - `ActuatorMotors.control[i] -> CommandMotorSpeed[motors_channel_offset + i]`
  - `ActuatorServos.control[i] -> CommandMotorSpeed[servos_channel_offset + i]`

## 3. 与 tailsitter 当前映射对应关系

在 `models/tailsitter/tailsitter.sdf.jinja` 中：

- `0..3`：thrust
- `4..7`：vector_x
- `8..11`：vector_y

所以常见配置是让输入数组前 12 路直接对齐写入 `CommandMotorSpeed[0..11]`。

## 4. 编译

```bash
cd Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/gazebo_test
mkdir -p build
cd build
cmake ..
make -j
```

## 5. 运行示例

### 示例 1：Float32MultiArray 输入

```bash
./ros_gazebo_bridge --ros-args \
  -p use_px4_topics:=false \
  -p control_allocation_topic:=/control_allocation/actuator_setpoint \
  -p gazebo_topic:=/gazebo/default/tailsitter/gazebo/command/motor_speed
```

### 示例 2：PX4 actuator 输出输入

```bash
./ros_gazebo_bridge --ros-args \
  -p use_px4_topics:=true \
  -p motors_topic:=/fmu/out/actuator_motors \
  -p use_servo_topic:=false \
  -p motors_channel_offset:=0 \
  -p gazebo_topic:=/gazebo/default/tailsitter/gazebo/command/motor_speed
```

## 6. 常用参数

- `wait_for_connection`：默认 `true`，启动时等待 Gazebo subscriber
- `gazebo_channel_count`：默认 `16`
- `control_array_offset`：Float32 模式写入起始通道
- `motors_channel_offset`：PX4 motors 写入起始通道
- `servos_channel_offset`：PX4 servos 写入起始通道
