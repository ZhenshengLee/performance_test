#include "ros/ros.h"


int main(int argc, char**argv)
{
  ros::init(argc, argv, "check_master");
  if (!ros::master::check()) {
    return 1;
  }
  return 0;
}
