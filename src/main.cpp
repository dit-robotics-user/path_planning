#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include <std_msgs/Int32MultiArray.h>

#include "path_planning/path.h"
#include "path_planning/path_planning.h"

#include "path_planning/AddTwoInts.h"
#include <cstdlib>

int my_pos_x_ = 200 ;
int my_pos_y_ = 200 ;

void callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  my_pos_x_ = msg->data[0] ;
  my_pos_y_ = msg->data[1] ;
  ROS_INFO("next_pos_x: %d", my_pos_x_);
  ROS_INFO("next_pos_y: %d", my_pos_y_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");

  ros::NodeHandle n;
  
  ros::ServiceClient client = n.serviceClient<path_planning::path>("add_two_ints_1");


  //test v1
  ros::Subscriber sub = n.subscribe("rxst1", 1, callback);
  ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("txST1", 1);

  while(ros::ok()){
    ROS_INFO("333 ");
    double begin_time =ros::Time::now().toSec();
    path_planning::path srv;
    srv.request.my_pos_x = my_pos_x_ ;
    srv.request.my_pos_y = my_pos_y_ ;
/*
    srv.request.enemy1_x = 1000 ;
    srv.request.enemy1_y = 500 ;
    srv.request.enemy2_x = 500 ;
    srv.request.enemy2_y = 1750 ;
    srv.request.ally_x = 1800 ;
    srv.request.ally_y = 1400 ; 
    */  
    srv.request.goal_pos_x = 1600;
    srv.request.goal_pos_y = 1500;

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

    std_msgs::Int32MultiArray msg_ ;
    msg_.data.push_back(0x4000);
    msg_.data.push_back(srv.response.next_pos_x);
    msg_.data.push_back(srv.response.next_pos_y);
    msg_.data.push_back(90);

    pub.publish(msg_);

    ros::spinOnce();
  }
  return 0;
}
