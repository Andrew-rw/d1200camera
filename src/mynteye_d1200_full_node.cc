#include <ros/ros.h>
// PCL and OpenCV specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
// ROS includes
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

// MYNT EYE includes
#include "mynteyed/camera.h"
#include "mynteyed/util/rate.h"
#include "mynteyed/utils.h"

MYNTEYE_USE_NAMESPACE
namespace enc = sensor_msgs::image_encodings;

std::string cameraColorTopic;
std::string cameraColorFrame;
std::string cameraDepthTopic;
std::string cameraDepthFrame;
std::string pointsTopic;
std::string pointsFrame;
int irIntensity;
float camFactor;
bool lowRes = false;
ros::Publisher pointCloudPub;

inline CameraIntrinsics getCameraIntrinsics(const Camera& camera) {
  auto stream_mode = camera.GetOpenParams().stream_mode;
  return camera.GetStreamIntrinsics(stream_mode).left;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud(
    const cv::Mat& rgb, const cv::Mat& depth, const CameraIntrinsics camIn,
    float cam_factor) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (int m = 0; m < depth.rows; m++) {
    for (int n = 0; n < depth.cols; n++) {
      std::uint16_t d = depth.ptr<std::uint16_t>(m)[n];
      if (d == 0) continue;
      pcl::PointXYZRGBA p;
      p.x = static_cast<float>(d) / cam_factor;
      p.y = ((depth.cols - n) - camIn.cx) * p.x / camIn.fx;
      p.z = ((depth.rows - m) - camIn.cy) * p.x / camIn.fy;
      // p.z = static_cast<float>(d) / cam_factor;
      // p.x = (n - camIn.cx) * p.z / camIn.fx;
      // p.y = (m - camIn.cy) * p.z / camIn.fy;
      p.b = rgb.ptr<uchar>(m)[n * 3];
      p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
      p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
      cloud->points.push_back(p);
    }
  }
  return cloud;
}

sensor_msgs::CameraInfoPtr createCameraInfo(const CameraIntrinsics& in,
                                            const std::string& frame_id,
                                            ros::Time& stamp) {
  // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
  sensor_msgs::CameraInfo* cameraInfo = new sensor_msgs::CameraInfo();
  auto cameraInfoPtr = sensor_msgs::CameraInfoPtr(cameraInfo);

  cameraInfo->width = in.width;
  cameraInfo->height = in.height;

  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  cameraInfo->K.at(0) = in.fx;
  cameraInfo->K.at(2) = in.cx;
  cameraInfo->K.at(4) = in.fy;
  cameraInfo->K.at(5) = in.cy;
  cameraInfo->K.at(8) = 1;

  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  cameraInfo->P.at(0) = in.p[0];
  cameraInfo->P.at(2) = in.p[2];
  cameraInfo->P.at(3) = in.p[3];
  cameraInfo->P.at(5) = in.p[5];
  cameraInfo->P.at(6) = in.p[6];
  cameraInfo->P.at(10) = in.p[10];

  cameraInfo->distortion_model = "plumb_bob";

  // D of plumb_bob: (k1, k2, t1, t2, k3)
  for (int i = 0; i < 5; i++) {
    cameraInfo->D.push_back(in.coeffs[i]);
  }

  // R to identity matrix
  cameraInfo->R.at(0) = in.r[0];
  cameraInfo->R.at(1) = in.r[1];
  cameraInfo->R.at(2) = in.r[2];
  cameraInfo->R.at(3) = in.r[3];
  cameraInfo->R.at(4) = in.r[4];
  cameraInfo->R.at(5) = in.r[5];
  cameraInfo->R.at(6) = in.r[6];
  cameraInfo->R.at(7) = in.r[7];
  cameraInfo->R.at(8) = in.r[8];

  cameraInfo->header.stamp = stamp;
  cameraInfo->header.frame_id = frame_id;

  return cameraInfoPtr;
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "mynteye_d1200_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  image_transport::ImageTransport iTransportMynteye(nh);

  priv_nh.param<std::string>("rgb_topic", cameraColorTopic, "camera_color");
  priv_nh.param<std::string>("rgb_frame", cameraColorFrame, "color_frame");
  priv_nh.param<std::string>("depth_topic", cameraDepthTopic, "camera_depth");
  priv_nh.param<std::string>("depth_frame", cameraDepthFrame, "depth_frame");
  priv_nh.param<std::string>("points_topic", pointsTopic, "");
  priv_nh.param<std::string>("points_frame", pointsFrame, "points_frame");
  priv_nh.param<bool>("low_resolution", lowRes, false);
  priv_nh.param<float>("camera_factor", camFactor, 500.0);
  priv_nh.param("ir_intensity", irIntensity, 0);

  image_transport::CameraPublisher pubColor = iTransportMynteye.advertiseCamera(cameraColorTopic, 1);
  image_transport::CameraPublisher pubDepth = iTransportMynteye.advertiseCamera(cameraDepthTopic, 1);
  if (!pointsTopic.empty()) {
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>(pointsTopic, 1);
  }

  // Initialize MYNTEYE camera
  Camera cam;
  DeviceInfo devInfo;
  if (!util::select(cam, &devInfo)) {
    return 1;
  }

  // Set parameters for D1200 camera
  OpenParams params(devInfo.index);
  params.color_stream_format = StreamFormat::STREAM_MJPG;
  params.depth_stream_format = StreamFormat::STREAM_YUYV;
  params.color_mode = ColorMode::COLOR_RECTIFIED;
  if (lowRes) {
    params.stream_mode = StreamMode::STREAM_640x480;
  } else {
    params.stream_mode = StreamMode::STREAM_1280x720;
  }
  params.framerate = 20;

  if (irIntensity > 10) irIntensity = 10;
  if (irIntensity < 0) irIntensity = 0;
  params.ir_intensity = irIntensity;

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Failed to open camera" << std::endl;
    return 1;
  }
  std::cout << "Opened " << devInfo.name << " device." << std::endl;

  CameraIntrinsics camIntr = getCameraIntrinsics(cam);

  Rate rate(params.framerate);
  // loop
  ros::Rate loopRate(params.framerate);

  cv::Mat color;
  cv::Mat depth;
  sensor_msgs::PointCloud2 msgPoints;

  while (nh.ok()) {
    auto imageColor = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
    if (imageColor.img) {
      color = imageColor.img->To(ImageFormat::COLOR_BGR)->ToMat();
    }
    auto imageDepth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
    if (imageDepth.img) {
      depth = imageDepth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
    }

    if (!color.empty() || !depth.empty()) {
      auto timestamp = ros::Time().now();
      if (!pointsTopic.empty()) {
        auto cloud = getPointCloud(color, depth, camIntr, camFactor);
        pcl::toROSMsg(*cloud, msgPoints);
        msgPoints.header.stamp = timestamp;
        msgPoints.header.frame_id = pointsFrame;
        pointCloudPub.publish(msgPoints);
      }

      std_msgs::Header header;
      header.stamp = timestamp;
      header.frame_id = cameraColorFrame;
      auto&& msg_color = cv_bridge::CvImage(header, enc::RGB8, color).toImageMsg();
      pubColor.publish(msg_color, createCameraInfo(camIntr, cameraColorFrame, timestamp));

      std_msgs::Header headerDepth;
      headerDepth.stamp = timestamp;
      headerDepth.frame_id = cameraDepthFrame;
      auto&& msg_depth = cv_bridge::CvImage(headerDepth, enc::MONO16, depth).toImageMsg();
      pubDepth.publish(msg_depth, createCameraInfo(camIntr, cameraDepthFrame, timestamp));
    }

    loopRate.sleep();
  }

  cam.Close();
  return 0;
}