#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"

#include "path_planning/path.h"
#include "path_planning/AddTwoInts.h"
#include <cstdlib>




int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");

  ros::NodeHandle n;
  
///  ros::Subscriber agent_sub = n.subscribe("")
  ros::ServiceClient client = n.serviceClient<path_planning::path>("add_two_ints_1");
  path_planning::path srv;
  srv.request.my_pos_x = 3 ;
  srv.request.my_pos_y = 3 ;
  srv.request.enemy1_x = 20 ;
  srv.request.enemy1_y = 10 ;
  srv.request.enemy2_x = 10 ;
  srv.request.enemy2_y = 35 ;
  srv.request.ally_x = 50 ;
  srv.request.ally_y = 28 ;   
  srv.request.goal_pos_x = 45;
  srv.request.goal_pos_y = 30;

  while(ros::ok()){
    ROS_INFO("333 ");
    double begin_time =ros::Time::now().toSec();

    if (client.call(srv))
    {
    double clustering_time = ros::Time::now().toSec () - begin_time; 
    ROS_INFO ("%f secs for path plan .", clustering_time);
    ROS_INFO("next_pos_x: %ld", (long int)srv.response.next_pos_x);
    ROS_INFO("next_pos_y: %ld", (long int)srv.response.next_pos_y);
    }
    else
    {
    ROS_ERROR("Failed to call service add_two_ints1");
    }

    ros::spinOnce();
  }
  return 0;
}