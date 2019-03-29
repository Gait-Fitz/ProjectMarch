// Copyright 2018 Project March.
#include <cmath>

#include <unistd.h>

#include <ros/ros.h>

#include <bitset>
#include <map>

#include <march_hardware/Joint.h>
#include <march_hardware/March4.h>
#include "sensor_msgs/JointState.h"
#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/EtherCAT/EthercatSDO.h>
#include <march_hardware/PDOmap.h>

int main(int argc, char** argv)
{

    march4cpp::PDOmap pdoMapMISO = march4cpp::PDOmap();
    pdoMapMISO.addObject("StatusWord");
    pdoMapMISO.addObject("ActualPosition");
    pdoMapMISO.addObject("DCLinkVoltage");
    pdoMapMISO.addObject("DetailedErrorRegister");
    std::map<std::string, int> map = pdoMapMISO.map(1, march4cpp::dataDirection::miso);

    //print map
    ROS_INFO("Byte offsets:");
    std::map<std::string, int>::iterator i;
    for (i = map.begin(); i != map.end(); i++){
        ROS_INFO("%s byte offset: %i", i->first.c_str(), i->second);
    }

//   march4cpp::MARCH4 march4 = march4cpp::MARCH4();
//   march4.startEtherCAT();

//   if (!march4.isEthercatOperational())
//   {
//     ROS_FATAL("EtherCAT is not operational");
//     return 0;
//   }

//   ros::init(argc, argv, "dummy");
//   ros::NodeHandle nh;
//   ros::Rate rate(10);

//   // Uncomment to allow actuation.
//   march4.getJoint("test_joint").getIMotionCube().goToOperationEnabled();
//   ROS_INFO("march4 initialized");

//   ROS_INFO_STREAM("Angle: " << march4.getJoint("test_joint").getAngleRad());
//   march4.getJoint("test_joint").getIMotionCube().actuateRadFixedSpeed(0.6, 0.1);
//   march4.getJoint("test_joint").getIMotionCube().actuateRadFixedSpeed(1, 0.2);
//   march4.getJoint("test_joint").getIMotionCube().actuateRadFixedSpeed(0.6, 0.3);

//   // Publish and print joint position
//   //    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("march/joint_states", 5);
//   //    angleVal = march4.getJoint("test_joint").getAngleRad();
//   //    printf("imc get: %f\n", angleVal);
//   //    sensor_msgs::JointState joint_state;
//   //    joint_state.header.stamp = ros::Time::now();
//   //    joint_state.name = {"test_joint"};
//   //    joint_state.position = {angleVal};
//   //    pub.publish(joint_state);

//   // Print final status
//   sleep(1);
//   march4.getJoint("test_joint")
//       .getIMotionCube()
//       .parseStatusWord(march4.getJoint("test_joint").getIMotionCube().getStatusWord());
//   march4.getJoint("test_joint")
//       .getIMotionCube()
//       .parseMotionError(march4.getJoint("test_joint").getIMotionCube().getMotionError());
//   march4.getJoint("test_joint")
//       .getIMotionCube()
//       .parseDetailedError(march4.getJoint("test_joint").getIMotionCube().getDetailedError());

//   march4.stopEtherCAT();
}
