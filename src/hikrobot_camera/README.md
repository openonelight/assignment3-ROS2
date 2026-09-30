# 第三次培训 海康 USB 相机 ROS 2 功能包

基于海康 MVS SDK 封装的 ROS 2 Humble 相机节点，支持按 USB 序列号连接、发布图像、设置曝光/增益/帧率/像素格式、断线重连后恢复配置。

项目已完成从厂商 SDK 到 ROS 2 标准图像话题的封装，并在真实 USB 工业相机上验证了图像显示、曝光调节、参数异常处理和断线恢复。采用节点与设备访问层分离的结构，配套 Launch、YAML 参数、RViz 配置和自动测试，便于启动、复用与排查问题。

**成果展示与验收证据见 [成果验收报告](ACCEPTANCE.md)。** 本提交采用 2026-09-30 Ubuntu 实测版本。

## 环境

- Ubuntu 22.04.5 LTS，x86_64，ROS 2 Humble。
- 海康 MV-CA016-10UC USB 工业相机，USB 协商速率 5000M。
- MVS 安装目录 `/opt/MVS`；使用的库为 `libMvCameraControl.so.4.8.2.1`。系统包版本字段为 `2025-07-11`，不等同于已核实完整客户端安装包版本。
- 本仓库不包含厂商 SDK。到[海康官方下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/)下载 Linux x86_64 MVS，按随包说明安装 SDK、USB 支持与访问规则。
- ROS 依赖声明在 `package.xml`，由 rosdep 处理；厂商 SDK 需单独安装。


本提交沿用原仓库的 `src/hikrobot_camera/` 存放位置；实际 ROS 包名为 `hik_camera`，编译和启动请使用本文命令。目录名与 ROS 包名可以不同。原模板遗留的 `main.cpp` 和 `include/hikrobot_camera/camera_node.hpp` 不参与当前 CMake 构建；当前节点入口包含在 `src/camera_node.cpp` 中。

## 编译与测试

本 Fork 根目录就是工作空间。首次下载时执行：

```bash
git clone https://github.com/openonelight/assignment3-ROS2.git
cd assignment3-ROS2
```

然后在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --packages-select hik_camera --cmake-args -DMVS_ROOT=/opt/MVS -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select hik_camera
colcon test-result --verbose
ctest --test-dir build/hik_camera --output-on-failure --no-tests=error
source install/setup.bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

## 运行

先停止并关闭 MVS，避免相机被占用。把 `YOUR_USB_SERIAL` 替换成 MVS 中显示的完整序列号，保留前导零。

```bash
ros2 launch hik_camera camera.launch.py serial_number:=YOUR_USB_SERIAL rviz:=true
```

RViz 的左侧 Camera Image 面板显示二维图像，右侧三维区域空白不代表图像发布失败。使用 Image 插件，不是需要相机标定信息的 Camera 插件。

使用自定义参数文件：

```bash
ros2 launch hik_camera camera.launch.py serial_number:=YOUR_USB_SERIAL params_file:=/absolute/path/camera.yaml
```

## 参数与接口

| 参数 | 类型/单位 | 随包 YAML 值 | 可运行时修改 |
|---|---|---|---|
| serial_number | string | 启动时提供 | 否 |
| image_topic | string | /image_raw | 否 |
| frame_id | string | camera_optical_frame | 否 |
| reliable | bool | true | 否 |
| exposure_time | double，微秒 | 20000.0 | 是 |
| gain | double，dB | 0.0 | 是，存在下述读回限制 |
| frame_rate | double，请求 fps | 30.0 | 是 |
| pixel_format | string，相机格式名 | BayerRG8 | 是，须设备支持 |

```bash
ros2 param get /hik_camera exposure_time
ros2 param set /hik_camera exposure_time 20000.0
ros2 param set /hik_camera frame_rate 60.0
ros2 param set /hik_camera pixel_format BayerRG8
ros2 topic echo /hik_camera/diagnostics --once
ros2 topic hz /image_raw
```

相机输入格式经过 SDK 转换，`/image_raw` 发布 `sensor_msgs/msg/Image`，编码始终为 `bgr8`，step 为宽度乘以 3。时间戳为主机取得并转换图像后的 ROS 时间，不是硬件曝光时间。节点不发布 CameraInfo 或 TF，不包含相机标定。

`/hik_camera/diagnostics` 报告请求帧率、节点发布帧率、累计帧数、硬件参数读回及支持的格式。请求 fps、发布 fps、订阅端接收 fps、RViz 渲染 fps 不是同一个量。

参数更新会暂停采集、关闭自动曝光/增益，查询范围并检查 SDK 返回值，设置后恢复采集。失败时尝试恢复旧配置。离线时拒绝设置。格式变化会验证一帧可转换图像。设备断开后释放句柄并每 2 秒按相同序列号重试，恢复最后成功的受管理参数。

## 成果验收概览

| 项目 | 结论 | 证据 |
|---|---|---|
| Ubuntu 编译 | colcon build 成功，无编译错误 | [构建与测试日志](evidence/build-20260930-214154.log) |
| 模拟后端测试 | 1 项测试通过，0 失败；内部覆盖参数回退、连接与缓冲释放等分支 | 同一构建日志 |
| 按序列号连接、ROS 图像发布 | 已运行；检查收到 1440×1080、bgr8 合法图像 | 随包 check JSON、运行日志 |
| RViz 显示、曝光修改 | 有实际图像及 10000→20000 微秒设置成功截图 | evidence/ros_image_exposure.png |
| 负曝光与无效格式拒绝 | 通过 | 随包 check JSON 对应字段均为 true |
| 合法曝光更新及恢复 | 通过 | valid_exposure_update、restored_exposure 为 true |
| 拔插恢复、离线修改拒绝 | 随包拔插检查通过 | 随包 check JSON、reconnect_check.jpg |
| 重连配置恢复 | 运行日志记录重新应用曝光、增益 0、帧率及 BayerRG8 | 随包 21:44 运行日志 |
| 帧率设置 | 30/60 fps 配置已有运行记录；一次诊断发布值约 59.99 fps | [60 fps 诊断截图](evidence/diagnostics_60fps.png)；持续性能见下文 |
| 参数接口覆盖 | 已实现曝光、增益、帧率、像素格式四类接口，包含校验、读回及回退 | `camera.hpp`、`camera_node.cpp`；实测范围见下文 |

## 测试范围与后续完善

本次已验证核心采集链路、曝光与异常恢复。增益 `3.0` 的设置在读回校验阶段被拒绝并回退，需完善硬件量化适配；不同合法像素格式切换尚缺实测证据。后段日志存在通信报错和帧率下降，持续吞吐性能仍需优化，约 60 fps 的记录为阶段性测量。初次编译另有一处 signed/unsigned 比较警告。上述项目不计入已通过的验收项。

维护者：openonelight。ROS 依赖与维护者信息见 `package.xml`。

## 目录

- `src/camera_node.cpp`：ROS 节点、参数回调、发布和重连调度。
- `include/hik_camera/camera.hpp`：SDK 封装、事务回退、图像转换。
- `launch/`、`config/`、`rviz/`：启动与配置。
- `test/backend_test.cpp`：模拟 SDK 后端测试。
- `evidence/`：实际返回的日志、自动检查结果和截图。

本包不含 build、install、colcon log 中间目录，也不含 SDK。代码许可见 LICENSE，厂商 SDK 不在此许可范围内。
