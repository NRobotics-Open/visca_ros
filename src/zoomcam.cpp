#include <visca_ros/zoomcam.h>

ZoomCam::ZoomCam(const ros::NodeHandle &pnh, const ros::NodeHandle &gnh):
  gnh_(gnh), pnh_(pnh)
{
  max_zoom_ = gnh.param<int>("max_zoom_value", 31424);
  std::string port;
  port = gnh.param<std::string>("port", "/dev/ttyAMA1");
  if(open_interface(port.c_str()))
  {
    ROS_INFO("Camera initialisation successful");
  }
  else
  {
    ROS_ERROR("Camera initialisation failed. Exiting !");
    exit(-1);
  }

  service_ = gnh_.advertiseService<ZRequest, ZResponse>("/ZoomCamServer",
                                boost::bind(&ZoomCam::serverCB, this, _1, _2));
  std::string cam_name;
  cam_name = gnh.param<std::string>("camera_name", "zoomcamera");
  cam_pub_ = gnh_.advertise<ZStatus>(std::string("/") + cam_name + "/status", 5, false);
}

bool ZoomCam::open_interface(const char *ttydev)
{
  int camera_num;
  if (VISCA_open_serial(&iface_, ttydev)!=VISCA_SUCCESS) {
    fprintf(stderr,"visca-cli: unable to open serial device %s\n",ttydev);
    return false;
  }

  VISCA_set_address(&iface_, &camera_num);
  if(VISCA_set_address(&iface_, &camera_num)!=VISCA_SUCCESS) {
#ifdef WIN
    _RPTF0(_CRT_WARN,"unable to set address\n");
#endif
    fprintf(stderr,"visca-cli: unable to set address\n");
    VISCA_close_serial(&iface_);
    return false;
  }

  camera_.address=1;

  if(VISCA_clear(&iface_, &camera_)!=VISCA_SUCCESS) {
#ifdef WIN
    _RPTF0(_CRT_WARN,"unable to clear interface\n");
#endif
    fprintf(stderr,"visca-cli: unable to clear interface\n");
    VISCA_close_serial(&iface_);
    return false;
  }
  if(VISCA_get_camera_info(&iface_, &camera_)!=VISCA_SUCCESS) {
#ifdef WIN
    _RPTF0(_CRT_WARN,"unable to oget camera infos\n");
#endif
    fprintf(stderr,"visca-cli: unable to oget camera infos\n");
    VISCA_close_serial(&iface_);
    return false;
  }

  return true;
}

void ZoomCam::close_interface()
{
  // read the rest of the data: (should be empty)
  unsigned char packet[3000];
  uint32_t buffer_size = 3000;

  VISCA_usleep(2000);

  if (VISCA_unread_bytes(&iface_, packet, &buffer_size)!=VISCA_SUCCESS)
  {
    uint32_t i;
    fprintf(stderr, "ERROR: %u bytes not processed", buffer_size);
    for (i=0;i<buffer_size;i++)
      fprintf(stderr,"%2x ",packet[i]);
    fprintf(stderr,"\n");
  }

  VISCA_close_serial(&iface_);
}

bool ZoomCam::get_zoom_value(uint16_t &res)
{
  uint16_t value16;
  if (VISCA_get_zoom_value(&iface_, &camera_, &value16)!=VISCA_SUCCESS) {
    return false;
  }
  res = value16;
  return true;
}

bool ZoomCam::serverCB(ZRequest &req, ZResponse &res)
{
  res.ack.data = true;
  switch(req.type)
  {
  case req.SET_RESOLUTION:
    ROS_WARN("Command Type not Implemented!");
    break;
  case req.SET_ZOOM:
    if(req.command > max_zoom_)
      req.command = max_zoom_;
    if(req.command < 0)
      req.command = 0;
    if(VISCA_set_zoom_value(&iface_, &camera_, req.command)!=VISCA_SUCCESS) {
      res.ack.data = false;
    }
    break;
  case req.SET_WIDE_TELE:
    if(req.command == 0)
    {
      if (VISCA_set_zoom_wide_speed(&iface_, &camera_, 4)!=VISCA_SUCCESS) {
        res.ack.data = false;
      }
    }
    else
    {
      if (VISCA_set_zoom_tele_speed(&iface_, &camera_, 4)!=VISCA_SUCCESS) {
        res.ack.data = false;
      }
    }
    break;
  default:
    ROS_WARN("Command Type Unknown!");
  }

  return true;
}

void ZoomCam::run()
{
  uint16_t zoom_val = 0;
  ZStatus status_msg;
  ros::Rate r(15);
  while(not ros::isShuttingDown())
  {
    if(get_zoom_value(zoom_val))
    {
      status_msg.header.stamp = ros::Time::now();
      status_msg.current_zoom = zoom_val;
      cam_pub_.publish(status_msg);
    }
    ros::spinOnce();
    r.sleep();
  }
}
