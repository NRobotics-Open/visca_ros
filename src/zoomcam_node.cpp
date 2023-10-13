#include <visca_ros/zoomcam.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zoomcam_node");
  ros::NodeHandle gnh, pnh("~");

  ZoomCam camera(gnh, pnh);
  camera.run();

  return 0;
}
