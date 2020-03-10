#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"


#include "path_planning/AddTwoInts.h"
#include <cstdlib>

bool add(path_planning::AddTwoInts::Request  &req,
         path_planning::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response2: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ROS_INFO("222 ");
  ros::init(argc, argv, "goap");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("add_two_ints_2", add);
  ROS_INFO("Ready to add two ints 2.");
  ros::spin();

  return 0;
}