#ifndef CAM_INFO_PUB_NODE_H
#define CAM_INFO_PUB_NODE_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

static std::string camera_calibration_path = "package://cam_info_publisher/calibrations/${NAME}.yaml";

class CamInfoPubNode {
 public:
  explicit CamInfoPubNode(ros::NodeHandle &nh_);
  ~CamInfoPubNode();
  void callback(const sensor_msgs::ImageConstPtr& image_left_msg, const sensor_msgs::ImageConstPtr& image_right_msg);
 
private:
  ros::NodeHandle leftNs;
  ros::NodeHandle rightNs;

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> *sync;

  message_filters::Subscriber<sensor_msgs::Image> *image_sub_left;
  message_filters::Subscriber<sensor_msgs::Image> *image_sub_right;

  image_transport::CameraPublisher camPubLeft;
  image_transport::CameraPublisher camPubRight;

  camera_info_manager::CameraInfoManager *cinfoLeft;
  camera_info_manager::CameraInfoManager *cinfoRight;

  sensor_msgs::CameraInfo leftCamInfo;
  sensor_msgs::CameraInfo rightCamInfo;
};

#endif
