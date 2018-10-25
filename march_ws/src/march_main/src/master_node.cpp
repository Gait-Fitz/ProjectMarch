//
// Created by tim on 25-10-18.
//

#include "master_node.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "enum/gait_enum.h"
#include <march_custom_msgs/Gait.h>
#include <march_custom_msgs/GaitInstruction.h>

ros::ServiceClient client;

void developerInputCallBack(const march_custom_msgs::Gait::ConstPtr &msg) {
  ROS_INFO("I heard: [gait: %ld]", msg->gait);
  march_custom_msgs::GaitInstruction srv;
  srv.request.gait = msg->gait;
  if (client.call(srv)) {
    const char *output = srv.response.result.c_str();
    ROS_INFO("MasterNode %s", output);
  } else {
    ROS_ERROR("Failed to call service gait_instructions");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("developer_input", 1000, developerInputCallBack);
  ros::ServiceClient client = n.serviceClient<march_custom_msgs::GaitInstruction>("gait_instructions");
  ros::spin();
  return 0;
}