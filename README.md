# mono_camera_calibration

面向 ROS 2 Jazzy 的 C++ 单目相机标定包。标定后端实现为
`rclcpp_components` 组件，可以与相机驱动加载到同一个组件容器中，通过 ROS 2
进程内通信接收全分辨率图像，避免原始图像经过 DDS 序列化和跨进程传输。

可选 GUI 是独立的 C++ 进程，只接收缩放后的 JPEG 预览和状态信息，不订阅原始图像。
因此关闭 GUI 不会影响标定后端，GUI 异常也不会导致相机和标定组件退出。

## 功能

- 支持棋盘格、对称圆点阵和非对称圆点阵
- 仅处理最新图像，不累积过期帧
- 可配置检测频率
- 根据标定板位置、尺度和倾斜程度自动筛选不同姿态样本
- 使用 OpenCV pinhole 模型计算内参和畸变参数
- 输出整体 RMS 和各视角重投影误差
- 保存兼容 `camera_info_manager` 的标准 YAML
- 通过 `sensor_msgs/srv/SetCameraInfo` 提交标定结果
- 提供开始、停止、重置、求解、保存和提交服务
- 分别统计接收帧与处理帧，便于区分主动跳帧和传输丢帧

当前 0.1.0 版本暂不支持 ChArUco 和 fisheye/equidistant 模型。

## 构建与测试

```bash
cd /path/to/ros2_workspace
colcon build --packages-select mono_camera_calibration
source install/setup.bash
colcon test --packages-select mono_camera_calibration
colcon test-result --verbose
```

## 独立运行

修改 `config/calibration.yaml` 中的图像话题、标定板和输出参数，然后运行：

```bash
ros2 launch mono_camera_calibration calibration.launch.py
```

这个 launch 会启动一个仅包含标定组件的容器。如果相机驱动在另一个进程中，原始图像仍会
经过 DDS。该方式适合功能验证，不是最高性能的部署方式。

关闭 GUI：

```bash
ros2 launch mono_camera_calibration calibration.launch.py gui:=false
```

## 进程内通信集成

高吞吐使用方式是把相机驱动组件和标定组件放入同一个
`component_container_mt`，并为两个组件都启用 `use_intra_process_comms`：

```python
ComposableNode(
    package="mono_camera_calibration",
    plugin="mono_camera_calibration::MonoCalibrationNode",
    namespace="/mono_calibration",
    name="calibrator",
    parameters=[{
        "image_topic": "/left_camera/image_raw",
        "set_camera_info_service": "/left_camera/set_camera_info",
        "camera_name": "left_camera",
    }],
    extra_arguments=[{"use_intra_process_comms": True}],
)
```

当前工作区的 `camera_calibration_bringup/launch/mono_calibration.launch.py` 已按此方式
集成 `HikCameraNode` 和 `MonoCalibrationNode`。

## 配置参数

主要参数位于 `config/calibration.yaml`：

| 参数 | 说明 |
|---|---|
| `image_topic` | 原始图像话题 |
| `set_camera_info_service` | 提交内参的服务 |
| `camera_name` | 写入 YAML 和 CameraInfo 的相机名称 |
| `board.pattern` | `chessboard`、`circles` 或 `acircles` |
| `board.columns` | 标定板横向检测点数 |
| `board.rows` | 标定板纵向检测点数 |
| `board.square_size_m` | 相邻检测点间距，单位为米 |
| `sampling.processing_rate_hz` | 标定板检测频率 |
| `sampling.minimum_samples` | 允许求解的最少样本数 |
| `sampling.maximum_samples` | 最大样本数 |
| `sampling.minimum_sample_distance` | 自动采样的最小姿态差异 |
| `display.preview_scale` | 压缩预览缩放比例 |
| `display.jpeg_quality` | JPEG 预览质量 |
| `output_path` | Save 操作写入的 YAML 路径 |

`board.columns` 和 `board.rows` 表示检测点数量，而不是方格数量。

## GUI 按键

- `G`：开始采集样本
- `X`：停止采集样本
- `R`：清空样本和标定结果
- `C`：执行标定求解
- `S`：将结果保存到 `output_path`
- `U`：通过 `set_camera_info_service` 提交结果
- `Q` 或 `Esc`：关闭 GUI

## 服务接口

不启动 GUI 时可以直接调用：

```text
/mono_calibration/start
/mono_calibration/stop
/mono_calibration/reset
/mono_calibration/calibrate
/mono_calibration/save
/mono_calibration/commit
```

例如：

```bash
ros2 service call /mono_calibration/calibrate std_srvs/srv/Trigger '{}'
ros2 service call /mono_calibration/save std_srvs/srv/Trigger '{}'
ros2 service call /mono_calibration/commit std_srvs/srv/Trigger '{}'
```

## 独立仓库使用

本目录是一个自包含 ROS 2 包，可以直接作为独立 Git 仓库发布。之后可在目标工作区中作为
子模块放回 `src`：

```bash
git submodule add <repository-url> src/mono_camera_calibration
git submodule update --init --recursive
```
