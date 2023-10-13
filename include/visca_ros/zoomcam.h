#pragma once
#include <ros/ros.h>
#include <visca_ros/ViscaService.h>
#include <visca_ros/CameraStatus.h>
#include <visca/libvisca.h>

typedef visca_ros::ViscaServiceRequest ZRequest;
typedef visca_ros::ViscaServiceResponse ZResponse;
typedef visca_ros::CameraStatus ZStatus;

class ZoomCam
{
public:
  ZoomCam(const ros::NodeHandle &, const ros::NodeHandle &);
  ~ZoomCam(){ close_interface(); }

  bool serverCB(ZRequest &, ZResponse &);
  void run();

private:
  bool open_interface(const char *);
  void close_interface();
  bool get_zoom_value(uint16_t &);

  uint32_t max_zoom_;
  VISCAInterface_t iface_;
  VISCACamera_t camera_;
  ros::NodeHandle gnh_, pnh_;
  ros::ServiceServer service_;
  ros::Publisher cam_pub_;
};

