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
  uint8_t input = 0x0;
  res.ack.data = true;
  switch(req.type)
  {
  case req.SET_RESOLUTION:
    switch (req.command[0])
    {
    case 2:
      input = EW9500H_REGISTER_VIDEO_1080I_60;
      break;
    case 4:
      input = EW9500H_REGISTER_VIDEO_1080I_50;
      break;
    case 6:
      input = EW9500H_REGISTER_VIDEO_1080P_29;
      break;
    case 7:
      input = EW9500H_REGISTER_VIDEO_1080P_30;
      break;
    case 8:
      input = EW9500H_REGISTER_VIDEO_1080P_25;
      break;
    case 9:
      input = EW9500H_REGISTER_VIDEO_720P_59;
      break;
    case 10:
      input = EW9500H_REGISTER_VIDEO_720P_60;
      break;
    case 12:
      input = EW9500H_REGISTER_VIDEO_720P_50;
      break;
    case 14:
      input = EW9500H_REGISTER_VIDEO_720P_29;
      break;
    case 15:
      input = EW9500H_REGISTER_VIDEO_720P_30;
      break;
    case 17:
      input = EW9500H_REGISTER_VIDEO_720P_25;
      break;
    case 19:
      input = EW9500H_REGISTER_VIDEO_1080P_59;
      break;
    case 20:
      input = EW9500H_REGISTER_VIDEO_1080P_50;
      break;
    case 21:
      input = EW9500H_REGISTER_VIDEO_1080P_60;
      break;
    case 26:
      input = EW9500H_REGISTER_VIDEO_480P_60;
      break;
    case 27:
      input = EW9500H_REGISTER_VIDEO_480P_59;
      break;
    case 28:
      input = EW9500H_REGISTER_VIDEO_576P_50;
      break;
    case 37:
      input = EW9500H_REGISTER_VIDEO_2160P_60;
      break;
    case 38:
      input = EW9500H_REGISTER_VIDEO_2160P_60_XL;
      break;
    case 39:
      input = EW9500H_REGISTER_VIDEO_2160P_50;
      break;
    case 40:
      input = EW9500H_REGISTER_VIDEO_2160P_30;
      break;
    case 41:
      input = EW9500H_REGISTER_VIDEO_2160P_29;
      break;
    case 42:
      input = EW9500H_REGISTER_VIDEO_2160P_25;
      break;
    case 43:
      input = EW9500H_REGISTER_VIDEO_2160P_30_XL;
      break;
    case 44:
      input = EW9500H_REGISTER_VIDEO_2160P_25_XL;
      break;
    default:
      ROS_INFO("Using default resolution: 1080I_59");
      input = EW9500H_REGISTER_VIDEO_1080I_59;
      break;
    }
    if (VISCA_set_register(&iface_, &camera_, EW9500H_REGISTER_VIDEO_SIGNAL, input)!=VISCA_SUCCESS)
    { //Reflected after camera reset
      res.ack.data = false;
    }
    //ROS_WARN("Command Type not Implemented!");
    break;
  case req.SET_DIGITAL_OUTPUT:
    switch (req.command[0])
    {
    case 1:
      input = EW9500H_REGISTER_DIGITAL_OUTPUT_HDMI_RGB;
      break;
    case 2:
      input = EW9500H_REGISTER_DIGITAL_OUTPUT_DVI_RGB;
      break;
    default:
      ROS_INFO("Using default digital out: HDMI_YUV");
      input = EW9500H_REGISTER_DIGITAL_OUTPUT_HDMI_YUV;
      break;
    }
    if (VISCA_set_register(&iface_, &camera_, EW9500H_REGISTER_DIGITAL_OUTPUT, input)!=VISCA_SUCCESS) {
      res.ack.data = false;
    }
    break;
  case req.SET_ZOOM_VALUE:
    if(req.command[0] > max_zoom_)
      req.command[0] = max_zoom_;
    if(req.command[0] < 0)
      req.command[0] = 0;
    if(VISCA_set_zoom_value(&iface_, &camera_, req.command[0])!=VISCA_SUCCESS) {
      res.ack.data = false;
    }
    break;
  case req.SET_ZOOM_WIDE_SPEED:
    if (VISCA_set_zoom_wide_speed(&iface_, &camera_, (uint8_t)req.command[0])!=VISCA_SUCCESS) {
      res.ack.data = false;
    }
    break;
  case req.SET_ZOOM_TELE_SPEED:
    if (VISCA_set_zoom_tele_speed(&iface_, &camera_, (uint8_t)req.command[0])!=VISCA_SUCCESS) {
      res.ack.data = false;
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
