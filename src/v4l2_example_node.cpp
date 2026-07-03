/**
 * @file v4l2_example_node.cpp
 * @brief 基于 V4L2 (Video4Linux2) 的 USB 摄像头数据采集 ROS 节点
 *
 * 功能概述：
 *   1. 通过 V4L2 接口打开 /dev/video0 摄像头设备
 *   2. 查询并打印设备能力、支持的视频格式和分辨率
 *   3. 设置视频采集格式为 YUYV 640x480
 *   4. 使用 mmap 内存映射方式申请帧缓冲区，实现高效零拷贝采集
 *   5. 开启视频流，循环采集帧数据
 *   6. 将 YUYV 原始数据通过 OpenCV 转换为 RGB 格式
 *   7. 通过 ROS 发布原始图像 (sensor_msgs/Image) 和 JPEG 压缩图像 (sensor_msgs/CompressedImage)
 *
 * 发布话题：
 *   - /camera/video0            (sensor_msgs/Image)            RGB 原始图像
 *   - /camera/video0/compressed (sensor_msgs/CompressedImage)  JPEG 压缩图像
 */

/* ======================== ROS 相关头文件 ======================== */
#include "ros/ros.h"                           // ROS 核心头文件，提供节点初始化、通信等功能
#include "sensor_msgs/Image.h"                 // ROS 标准消息类型：原始图像
#include "sensor_msgs/CompressedImage.h"       // ROS 标准消息类型：压缩图像（JPEG/PNG 等）
#include "image_transport/image_transport.h"   // 图像传输库，支持高效图像传输（此处仅包含，未直接使用）
#include "cv_bridge/cv_bridge.h"              // OpenCV 与 ROS 图像消息之间的桥接库

/* ======================== 标准库头文件 ======================== */
#include <iostream>      // C++ 标准输入输出流
#include <stdio.h>       // C 标准输入输出（printf 等）
#include <fcntl.h>       // 文件控制（open 函数的 O_RDWR 等标志）
#include <memory.h>      // 内存操作（memset 等）
#include <unistd.h>      // POSIX 标准函数（close 等）
#include <signal.h>      // 信号处理
#include <string>        // C++ 字符串类

/* ======================== 系统调用头文件 ======================== */
#include <sys/ioctl.h>   // ioctl 系统调用，用于 V4L2 设备控制
#include <sys/mman.h>    // mmap 内存映射，用于零拷贝帧缓冲区

/* ======================== V4L2 头文件 ======================== */
#include <linux/videodev2.h>  // V4L2 内核接口头文件，定义所有 V4L2 数据结构和 ioctl 命令

/* ======================== OpenCV 头文件 ======================== */
#include <opencv4/opencv2/core.hpp>      // OpenCV 核心功能（Mat 类等）
#include <opencv4/opencv2/imgproc.hpp>   // 图像处理（颜色空间转换 cvtColor 等）
#include <opencv4/opencv2/imgcodecs.hpp> // 图像编解码（imwrite 等，用于可选的图像保存功能）

/**
 * @brief 帧缓冲区数量
 *
 * 使用多缓冲区轮转机制采集视频帧，避免单缓冲区导致的帧丢失。
 * 4 个缓冲区可以保证在采集和处理过程中有足够的缓冲，减少等待。
 */
#define CAP_BUF_NUM 4

/**
 * @brief 视频帧缓冲区结构体
 *
 * 存储每个 mmap 映射缓冲区的起始地址和长度，
 * 用于后续通过 munmap 解除映射。
 */
struct video_buffer {
  void* start;   ///< 缓冲区起始地址（mmap 返回的用户空间虚拟地址）
  size_t length; ///< 缓冲区长度（字节数）
};

/**
 * @brief 主函数：V4L2 摄像头数据采集 ROS 节点入口
 *
 * 程序执行流程：
 *   1. 初始化 ROS 节点
 *   2. 打开 V4L2 设备并查询设备能力
 *   3. 枚举设备支持的视频格式和分辨率
 *   4. 设置采集格式（YUYV 640x480）
 *   5. 申请并映射帧缓冲区（mmap 零拷贝方式）
 *   6. 开启视频流采集
 *   7. 主循环：出队 -> 格式转换 -> 发布 ROS 消息 -> 入队
 *   8. 停止采集，释放资源
 */
int main(int argc, char** argv) {
  /* ---- 第1步：初始化 ROS 节点 ---- */
  ros::init(argc, argv, "v4l2_example");  // 注册 ROS 节点，节点名称为 "v4l2_example"

  ros::NodeHandle node("camera");          // 创建命名空间为 "camera" 的节点句柄
                                           // 后续发布的话题将带有 /camera/ 前缀
  ros::NodeHandle private_nh("~");         // 创建私有节点句柄，用于获取 ~ 命名空间下的参数

  /* ---- 创建 ROS 发布者 ---- */
  // 发布原始 RGB 图像，话题为 /camera/video0，队列长度为 2
  ros::Publisher img_pub = node.advertise<sensor_msgs::Image>("video0", 2);
  // 发布 JPEG 压缩图像，话题为 /camera/video0/compressed，队列长度为 2
  ros::Publisher compressed_pub = node.advertise<sensor_msgs::CompressedImage>("video0/compressed", 2);

  /* ---- 第2步：打开 V4L2 设备 ---- */
  // 以读写方式打开 /dev/video0 设备文件
  // O_RDWR: 可读可写模式
  int fd = open("/dev/video0", O_RDWR);
  if (-1 == fd) {
    std::cerr << "Open the device /dev/video0 failed." << std::endl;
    return -1;
  }

  /* ---- 查询设备能力（capability） ---- */
  // VIDIOC_QUERYCAP: 查询 V4L2 设备的基本信息和能力
  // v4l2_capability 结构体包含：驱动名称、设备名称、总线信息、版本号、能力标志等
  struct v4l2_capability cap = {0};
  int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
  if (ret < 0) {
    if(EINVAL == errno){   /* EINVAL 为返回的错误值，表示不是有效的 V4L2 设备 */
      std::cerr << "/dev/video0 is no V4L2 device" << std::endl;
      return -1;
    } else {
      std::cerr << "/dev/video0 is not V4L2 device,unknow error" << std::endl;
      return -1;
    }
  }

  // 打印设备详细信息，用于调试和确认设备状态
  printf("cap.driver = %s \n", cap.driver);            // 驱动名称（如 uvcvideo）
  printf("cap.card = %s \n", cap.card);                // 设备名称（如 USB Camera）
  printf("cap.bus_info = %s \n", cap.bus_info);        // 总线信息（如 usb-0000:00:14.0-1）
  printf("cap.version = %d \n", cap.version);           // 驱动版本号
  printf("cap.capabilities = %x \n", cap.capabilities); // 设备总能力标志（位掩码）
  printf("cap.device_caps = %x \n", cap.device_caps);   // 设备具体能力标志
  printf("cap.reserved = %x %x %x \n", cap.reserved[0], cap.reserved[1], cap.reserved[2]);

  // 检查设备是否支持视频捕获功能
  // V4L2_CAP_VIDEO_CAPTURE: 表示设备支持视频捕获（从设备读取图像数据）
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    std::cerr << "/dev/video0 is no video capture device" << std::endl;
    return -1;
  }
  // 检查设备是否支持流式 I/O（streaming）
  // V4L2_CAP_STREAMING: 表示设备支持流式采集（配合 mmap 或用户指针方式）
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    std::cerr << "/dev/video0 does not support streaming i/o" << std::endl;
    return -1;
  }

  /* ---- 第3步：枚举设备支持的视频格式和分辨率 ---- */
  // v4l2_fmtdesc: 描述设备支持的像素格式
  // 通过循环调用 VIDIOC_ENUM_FMT 遍历所有支持的格式
  struct v4l2_fmtdesc fmtdesc = {0};
  fmtdesc.index = 0; // 从第一个输出格式开始查询（索引从 0 开始）
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 指定查询视频捕获类型的格式

  // v4l2_frmsizeenum: 描述指定像素格式下支持的分辨率
  struct v4l2_frmsizeenum frmsize = {0};

  // 外层循环：遍历所有支持的像素格式
  while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
    // 打印像素格式的四字符编码（FourCC）和格式描述
    // FourCC 格式如 YUYV = 'Y','U','Y','V'，每个字符占一个字节
    printf("pixelformat = %c%c%c%c, description = %s \n",
           fmtdesc.pixelformat & 0xFF, (fmtdesc.pixelformat >> 8) & 0xFF, (fmtdesc.pixelformat >> 16) & 0xFF,
           (fmtdesc.pixelformat >> 24) & 0xFF, fmtdesc.description);

    frmsize.index = 0;
    frmsize.pixel_format = fmtdesc.pixelformat;
    // 内层循环：遍历当前像素格式下支持的所有分辨率
    while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) != -1) {
      if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        // 离散分辨率：设备支持固定的分辨率列表（如 640x480, 1280x720）
        printf("line:%d %dx%d\n", __LINE__, frmsize.discrete.width, frmsize.discrete.height);
      } else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
        // 步进式分辨率：设备支持一个范围内的任意分辨率（最小值、最大值、步进值）
        printf("line:%d %dx%d\n", __LINE__, frmsize.discrete.width, frmsize.discrete.height);
      }
      frmsize.index ++;
    }

    fmtdesc.index ++;
  }

  /* ---- 第4步：设置视频采集格式 ---- */
  // v4l2_format: 设置设备的视频捕获格式
  // 这里设置为 YUYV 格式，分辨率 640x480
  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;       // 视频捕获类型
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;  // YUYV 像素格式（每像素 2 字节，YUV 4:2:2 采样）
  fmt.fmt.pix.width = 640;  // 图像宽度（像素）
  fmt.fmt.pix.height = 480; // 图像高度（像素）

  // VIDIOC_S_FMT: 设置设备格式，驱动会根据硬件能力调整实际参数
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
    std::cerr << "set format failed." << std::endl;
    return -1;
  }

  /* ---- 第5步：申请帧缓冲区（mmap 方式） ---- */
  // v4l2_requestbuffers: 向驱动申请帧缓冲区
  // 使用 mmap 内存映射方式可以避免数据从内核空间到用户空间的拷贝，提高效率
  struct v4l2_requestbuffers req = {0};
  req.count = CAP_BUF_NUM;                        // 申请 4 个缓冲区
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;         // 视频捕获类型
  req.memory = V4L2_MEMORY_MMAP;                  // 使用 mmap 内存映射方式

  // VIDIOC_REQBUFS: 请求分配缓冲区
  if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
    if (EINVAL == errno) {
      std::cerr << "/dev/video0 does not support memory mapping" << std::endl;
      return -1;
    } else {
      std::cerr << "/dev/video0 does not support memory mapping, unknow error" << std::endl;
      return -1;
    }
  }
  // 至少需要 2 个缓冲区才能实现轮转采集
  if (req.count < 2) {
    std::cerr << "Insufficient buffer memory on /dev/video0" << std::endl;
    return -1;
  }

  /* ---- 将缓冲区映射到用户空间并加入采集队列 ---- */
  // 分配 video_buffer 数组，用于记录每个缓冲区的用户空间地址和长度
  struct video_buffer* buffers = (struct video_buffer*)calloc(req.count, sizeof(*buffers));
  struct v4l2_buffer buf;
  for (size_t i = 0; i < req.count; i++) {
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    // VIDIOC_QUERYBUF: 查询缓冲区的偏移地址和长度
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
      std::cerr << "query memory mapping failed" << std::endl;
      return -1;
    }

    // 保存缓冲区长度
    buffers[i].length = buf.length;
    // mmap: 将内核缓冲区映射到用户空间，实现零拷贝访问
    // MAP_SHARED: 共享映射，对映射区域的写入会反映到内核缓冲区
    // buf.m.offset: 内核缓冲区的偏移量，作为 mmap 的 offset 参数
    buffers[i].start = 
        mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (buffers[i].start == MAP_FAILED) {
      std::cerr << "map memory failed" << std::endl;
      return -1;
    }

    // VIDIOC_QBUF: 将缓冲区加入采集队列（入队），驱动开始使用该缓冲区采集数据
    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
      printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
      return -1;
    }
  }

  /* ---- 第6步：开启视频流采集 ---- */
  // VIDIOC_STREAMON: 启动视频流，驱动开始向已入队的缓冲区填充图像数据
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
    printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
    return -1;
  }

  /* ---- 第7步：主采集循环 ---- */
  // 创建 RGB 图像矩阵，用于存储颜色转换后的图像
  // 注意：此处尺寸硬编码为 1280x720，但实际采集格式为 640x480，
  // cvtColor 会根据输入图像的实际尺寸自动调整输出
  cv::Mat rgb(cv::Size(1280, 720), CV_8UC3);
  uint32_t count = 0;          // 已采集帧计数器
  ros::Rate loop_rate(50);     // 设置循环频率为 50Hz（每帧间隔约 20ms）
  while (ros::ok()) {
    /* -- 从采集队列中取出一帧数据 -- */
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    // VIDIOC_DQBUF: 从输出队列中取出已填充数据的缓冲区（出队）
    // 如果队列中没有就绪的帧，此调用会阻塞等待
    if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
      printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
      return -1;
    }

    // （可选功能）保存图像到文件 —— 以下代码默认被注释
    // buf.timestamp 包含帧的时间戳信息，可用于文件命名
    // std::cout << "image timestamp: " << (buf.timestamp.tv_sec + buf.timestamp.tv_usec * 1e-6);
    // std::cout << " length: " << buf.length << std::endl;
    // std::string img_path = "/home/venus/workspace/record_data_tools/install/images/";
    // img_path.append(std::to_string(buf.timestamp.tv_sec));
    // img_path.append("_");
    // img_path.append(std::to_string(buf.timestamp.tv_usec));
    // img_path.append(".bmp");
    // std::cout << "image path: " << img_path.c_str() << std::endl;

    /* -- 将 YUYV 原始数据转换为 RGB 图像 -- */
    // 用 mmap 映射的缓冲区地址构造 OpenCV Mat 对象
    // YUYV 格式：每像素 2 字节（CV_8UC2），宽度为 fmt.fmt.pix.width
    cv::Mat yuyv(cv::Size(fmt.fmt.pix.width, fmt.fmt.pix.height), CV_8UC2, buffers[buf.index].start);
    // YUYV -> RGB 颜色空间转换
    // COLOR_YUV2RGB_YUYV: OpenCV 中 YUYV (YUV 4:2:2) 到 RGB 的转换标识
    cv::cvtColor(yuyv, rgb, cv::COLOR_YUV2RGB_YUYV);
    // cv::imwrite(img_path, rgb);  // 可选：保存为图片文件

    /* -- 将 OpenCV 图像转换为 ROS 消息并发布 -- */
    // 构造原始图像消息：cv_bridge 将 OpenCV Mat 转换为 sensor_msgs::Image
    // "rgb8" 表示 8 位 RGB 编码格式
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb).toImageMsg();
    // 构造 JPEG 压缩图像消息
    sensor_msgs::CompressedImagePtr compressed_msg = 
        cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb).toCompressedImageMsg(cv_bridge::Format::JPEG);
    // 发布原始图像和压缩图像到对应话题
    img_pub.publish(img_msg);
    compressed_pub.publish(compressed_msg);

    count ++;  // 帧计数器递增

    /* -- 将处理完的缓冲区重新加入采集队列 -- */
    // VIDIOC_QBUF: 缓冲区使用完毕后重新入队，供驱动继续填充新帧数据
    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
      printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
      return -1;
    }

    ros::spinOnce();   // 处理 ROS 回调（如订阅者连接/断开等事件）
    loop_rate.sleep(); // 休眠至下一个周期，维持 50Hz 的发布频率
  }

  std::cout << "v4l2 over" << std::endl;

  /* ---- 第8步：停止采集，释放资源 ---- */
  // VIDIOC_STREAMOFF: 停止视频流采集
  if (ioctl(fd, VIDIOC_STREAMOFF, &type)) {
    printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
    return -1;
  }
  // 解除所有缓冲区的 mmap 映射，释放用户空间虚拟内存
  for (size_t i = 0; i < req.count; i++) {
    munmap(buffers[i].start, buffers->length);
  }

  // 关闭设备文件描述符
  close(fd);
  return 0;
}
