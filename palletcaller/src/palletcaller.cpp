#include "ros/ros.h"
#include <palletdetector/Capture3Dservice.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "palletcaller");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<palletdetector::Capture3Dservice>("/PalletFinderService");
  palletdetector::Capture3Dservice srv;
  srv.request.CaptureNext = 1;
  ROS_INFO("Asking pallet detector for pose2D. Waiting for response ...");
  if (client.call(srv))
  {
    ROS_INFO("Pallet Detector found a pallet here: [x = %6.3f, y = %6.3f, Theta = %6.3f]", srv.response.pose.x,srv.response.pose.y, srv.response.pose.theta);
  }
  else
  {
    ROS_ERROR("Failed to call pallet detector service.");
    return 1;
  }

  return 0;
}
