# V4L2 摄像头数据采集 ROS 示例

基于 **V4L2 (Video4Linux2)** 接口的 USB 摄像头数据采集 ROS 节点。通过 Linux 内核提供的 V4L2 驱动框架，以 **mmap 零拷贝**方式高效采集摄像头原始数据，经颜色空间转换后发布为 ROS 图像消息。

## 功能特性

- 通过 V4L2 接口直接访问 `/dev/video0` 摄像头设备
- 自动查询并打印设备能力、支持的视频格式和分辨率
- 使用 **mmap 内存映射**方式采集视频帧，避免内核/用户空间数据拷贝
- 多缓冲区轮转机制（4 帧缓冲），保证采集流畅性
- 将 YUYV 原始数据通过 OpenCV 转换为 RGB 格式
- 同时发布两种 ROS 图像消息：
  - `sensor_msgs/Image` — 原始 RGB 图像
  - `sensor_msgs/CompressedImage` — JPEG 压缩图像

## 项目结构

```
v4l2_example/
├── include/v4l2_example/       # 头文件目录（当前为空）
├── launch/
│   └── v4l2_example.launch     # ROS 启动文件
├── src/
│   └── v4l2_example_node.cpp   # 主节点源代码
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS 包描述文件
└── README.md                   # 本文件
```

## 依赖项

| 依赖 | 说明 |
|------|------|
| ROS (Noetic/Melodic) | 机器人操作系统 |
| OpenCV 4 | 图像处理与颜色空间转换 |
| roscpp | ROS C++ 客户端库 |
| sensor_msgs | ROS 标准传感器消息类型 |
| cv_bridge | OpenCV 与 ROS 图像消息桥接 |
| image_transport | ROS 图像传输库 |

### 安装依赖（Ubuntu + ROS Noetic）

```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-sensor-msgs
sudo apt install libopencv-dev
```

## 编译

```bash
# 进入 catkin 工作空间
cd ~/workspace/catkin_ws

# 编译
catkin_make

# 加载环境变量
source devel/setup.bash
```

## 运行

### 方式一：使用 roslaunch

```bash
roslaunch v4l2_example v4l2_example.launch
```

启动参数：
- `use_camera`（默认 `true`）— 是否启动摄像头节点

```bash
# 示例：不启动摄像头（仅加载其他节点）
roslaunch v4l2_example v4l2_example.launch use_camera:=false
```

### 方式二：使用 rosrun

```bash
rosrun v4l2_example v4l2_example_node
```

## 发布话题

| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/camera/video0` | `sensor_msgs/Image` | RGB 原始图像（8 位，rgb8 编码） |
| `/camera/video0/compressed` | `sensor_msgs/CompressedImage` | JPEG 压缩图像 |

### 查看图像

```bash
# 使用 rqt_image_view 查看原始图像
rosrun rqt_image_view rqt_image_view

# 或使用 image_view 查看
rosrun image_view image_view image:=/camera/video0

# 查看压缩图像
rosrun image_view image_view image:=/camera/video0/compressed
```

## V4L2 采集流程

本节点的 V4L2 采集流程如下：

```
打开设备 (/dev/video0)
    │
    ▼
查询设备能力 (VIDIOC_QUERYCAP)
    │  检查是否支持视频捕获 + 流式 I/O
    ▼
枚举支持格式 (VIDIOC_ENUM_FMT / VIDIOC_ENUM_FRAMESIZES)
    │  打印所有像素格式和分辨率
    ▼
设置采集格式 (VIDIOC_S_FMT)
    │  YUYV 640x480
    ▼
申请帧缓冲区 (VIDIOC_REQBUFS)
    │  4 个 mmap 缓冲区
    ▼
映射缓冲区 (VIDIOC_QUERYBUF + mmap)
    │  将内核缓冲区映射到用户空间
    ▼
入队缓冲区 (VIDIOC_QBUF)
    │  所有缓冲区加入采集队列
    ▼
开启视频流 (VIDIOC_STREAMON)
    │
    ▼
┌─── 主循环 (50Hz) ───────────────────────┐
│  出队 (VIDIOC_DQBUF)                     │
│      │                                    │
│      ▼                                    │
│  YUYV → RGB 颜色转换 (cv::cvtColor)      │
│      │                                    │
│      ▼                                    │
│  发布 ROS 图像消息                        │
│      │                                    │
│      ▼                                    │
│  重新入队 (VIDIOC_QBUF)                   │
└──────────────────────────────────────────┘
    │
    ▼ (Ctrl+C 退出)
停止视频流 (VIDIOC_STREAMOFF)
    │
    ▼
解除映射 (munmap) + 关闭设备 (close)
```

## 代码关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 设备路径 | `/dev/video0` | 摄像头设备文件 |
| 像素格式 | `V4L2_PIX_FMT_YUYV` | YUV 4:2:2 采样，每像素 2 字节 |
| 分辨率 | 640 × 480 | 采集分辨率 |
| 缓冲区数量 | 4 | mmap 帧缓冲区数量 |
| 发布频率 | 50 Hz | ROS 消息发布频率 |
| 话题队列长度 | 2 | ROS Publisher 队列长度 |

## 注意事项

1. **设备权限**：确保当前用户对 `/dev/video0` 有读写权限
   ```bash
   sudo chmod 666 /dev/video0
   # 或将用户加入 video 组
   sudo usermod -aG video $USER
   ```

2. **设备占用**：同一时间只能有一个进程打开摄像头设备，如果其他程序（如 cheese、Skype）正在使用摄像头，需先关闭。

3. **分辨率说明**：代码中 `cv::Mat rgb` 初始化为 1280×720，但实际采集格式为 640×480。`cv::cvtColor` 会根据输入 Mat 的实际尺寸自动调整输出，不影响功能正确性。

4. **格式兼容性**：如果摄像头不支持 YUYV 格式，`VIDIOC_S_FMT` 可能会失败。可通过启动时终端打印的枚举信息确认设备支持的格式。
