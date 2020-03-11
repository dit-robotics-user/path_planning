#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray"

#include "path_planning/path.h"
#include "path_planning/path_planning.h"

#include "path_planning/AddTwoInts.h"
#include <cstdlib>

int my_pos_x_ = 0 ;
int my_pos_y_ = 0 ;

void callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  my_pos_x_ = msg->data[0] ;
  my_pos_y_ = msg->data[1] ;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");

  ros::NodeHandle n;
  
  ros::ServiceClient client = n.serviceClient<path_planning::path>("add_two_ints_1");


  //test v1
  ros::Subscriber sub = n.subscribe("rxst1", 1, callback);
  ros::Publisher pub = n.advertise<std_msg::Int32MultiArray>("txST1", 1);

  while(ros::ok()){
    ROS_INFO("333 ");
    double begin_time =ros::Time::now().toSec();
    path_planning::path srv;
    srv.request.my_pos_x = my_pos_x_ ;
    srv.request.my_pos_y = my_pos_y_ ;
    srv.request.enemy1_x = 20 ;
    srv.request.enemy1_y = 10 ;
    srv.request.enemy2_x = 10 ;
    srv.request.enemy2_y = 35 ;
    srv.request.ally_x = 50 ;
    srv.request.ally_y = 28 ;   
    srv.request.goal_pos_x = 45;
    srv.request.goal_pos_y = 30;

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

    std_msgs::Int32MultiArray msg ;
    msg[0] = 4000;
    msg[1] = srv.response.next_pos_x;
    msg[2] = srv.response.next_pos_y;
    msg[3] = 90;

    pub.publish(msg);

    ros::spinOnce();
  }
  return 0;
}