#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <memory.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>

#define CAP_BUF_NUM 4

struct video_buffer {
  void* start;
  size_t length;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "v4l2_example");

  ros::NodeHandle node("camera");
  ros::NodeHandle private_nh("~");

  ros::Publisher img_pub = node.advertise<sensor_msgs::Image>("video0", 2);
  ros::Publisher compressed_pub = node.advertise<sensor_msgs::CompressedImage>("video0/compressed", 2);

  int fd = open("/dev/video0", O_RDWR);
  if (-1 == fd) {
    std::cerr << "Open the device /dev/video0 failed." << std::endl;
    return -1;
  }

  // 获取设备capability
  struct v4l2_capability cap = {0};
  int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
  if (ret < 0) {
    if(EINVAL == errno){   /*EINVAL为返回的错误值*/
      std::cerr << "/dev/video0 is no V4L2 device" << std::endl;
      return -1;
    } else {
      std::cerr << "/dev/video0 is not V4L2 device,unknow error" << std::endl;
      return -1;
    }
  }

  printf("cap.driver = %s \n", cap.driver);
  printf("cap.card = %s \n", cap.card);
  printf("cap.bus_info = %s \n", cap.bus_info);
  printf("cap.version = %d \n", cap.version);
  printf("cap.capabilities = %x \n", cap.capabilities);
  printf("cap.device_caps = %x \n", cap.device_caps);
  printf("cap.reserved = %x %x %x \n", cap.reserved[0], cap.reserved[1], cap.reserved[2]);

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    std::cerr << "/dev/video0 is no video capture device" << std::endl;
    return -1;
  }
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    std::cerr << "/dev/video0 does not support streaming i/o" << std::endl;
    return -1;
  }

  // 查询设备支持的输出格式
  struct v4l2_fmtdesc fmtdesc = {0};
  fmtdesc.index = 0; // 从第一个输出格式开始查询
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 查询图片的输出格式

  struct v4l2_frmsizeenum frmsize = {0};

  while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
    printf("pixelformat = %c%c%c%c, description = %s \n",
           fmtdesc.pixelformat & 0xFF, (fmtdesc.pixelformat >> 8) & 0xFF, (fmtdesc.pixelformat >> 16) & 0xFF,
           (fmtdesc.pixelformat >> 24) & 0xFF, fmtdesc.description);

    frmsize.index = 0;
    frmsize.pixel_format = fmtdesc.pixelformat;
    while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) != -1) {
      if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        printf("line:%d %dx%d\n", __LINE__, frmsize.discrete.width, frmsize.discrete.height);
      } else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
        printf("line:%d %dx%d\n", __LINE__, frmsize.discrete.width, frmsize.discrete.height);
      }
      frmsize.index ++;
    }

    fmtdesc.index ++;
  }

  // 设备视频捕获格式
  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.width = 640;// 1280;
  fmt.fmt.pix.height = 480;// 720;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
    std::cerr << "set format failed." << std::endl;
    return -1;
  }

  // 申请帧缓冲区
  struct v4l2_requestbuffers req = {0};
  req.count = CAP_BUF_NUM;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
    if (EINVAL == errno) {
      std::cerr << "/dev/video0 does not support memory mapping" << std::endl;
      return -1;
    } else {
      std::cerr << "/dev/video0 does not support memory mapping, unknow error" << std::endl;
      return -1;
    }
  }
  if (req.count < 2) {
    std::cerr << "Insufficient buffer memory on /dev/video0" << std::endl;
    return -1;
  }

  // 映射缓冲区，并加入队列
  struct video_buffer* buffers = (struct video_buffer*)calloc(req.count, sizeof(*buffers));
  struct v4l2_buffer buf;
  for (size_t i = 0; i < req.count; i++) {
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
      std::cerr << "query memory mapping failed" << std::endl;
      return -1;
    }

    buffers[i].length = buf.length;
    buffers[i].start = 
        mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (buffers[i].start == MAP_FAILED) {
      std::cerr << "map memory failed" << std::endl;
      return -1;
    }

    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
      printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
      return -1;
    }
  }

  // 打开设备视频流
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
    printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
    return -1;
  }

  cv::Mat rgb(cv::Size(1280, 720), CV_8UC3);
  uint32_t count = 0;
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
      printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
      return -1;
    }

    // std::cout << "image timestamp: " << (buf.timestamp.tv_sec + buf.timestamp.tv_usec * 1e-6);
    // std::cout << " length: " << buf.length << std::endl;
    // std::string img_path = "/home/venus/workspace/record_data_tools/install/images/";
    // img_path.append(std::to_string(buf.timestamp.tv_sec));
    // img_path.append("_");
    // img_path.append(std::to_string(buf.timestamp.tv_usec));
    // img_path.append(".bmp");
    // std::cout << "image path: " << img_path.c_str() << std::endl;

    cv::Mat yuyv(cv::Size(fmt.fmt.pix.width, fmt.fmt.pix.height), CV_8UC2, buffers[buf.index].start);
    cv::cvtColor(yuyv, rgb, cv::COLOR_YUV2RGB_YUYV);
    // cv::imwrite(img_path, rgb);

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb).toImageMsg();
    sensor_msgs::CompressedImagePtr compressed_msg = 
        cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb).toCompressedImageMsg(cv_bridge::Format::JPEG);
    img_pub.publish(img_msg);
    compressed_pub.publish(compressed_msg);

    count ++;

    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
      printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
      return -1;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "v4l2 over" << std::endl;

  // 停止视频采集，解除映射
  if (ioctl(fd, VIDIOC_STREAMOFF, &type)) {
    printf("ERROR: VIDIOC_QBUF[%s], FUNC[%s], LINE[%d]\n", 
             "/dev/video0", __FUNCTION__, __LINE__);
    return -1;
  }
  for (size_t i = 0; i < req.count; i++) {
    munmap(buffers[i].start, buffers->length);
  }

  close(fd);
  return 0;
}
