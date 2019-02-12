//
// Initialises and runs ROS and the EtherCAT master
// Developed by MARCH IV
//

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "EthercatMaster.h"
#include "TemplateGES.h"
#include "IMC.h"
#include "PDB.h"

std::vector<Slave*> slaveList;

// Callback function to set data sent to Template GES
// Triggered when a message is published on /march/template/data
void SetTemplateDataCB(std_msgs::UInt8 msg)
{
  for (int i = 0; i < slaveList.size(); i++)
  {
    std::string slaveType = slaveList[i]->getType();
    if (slaveType == "TEMPLATEGES")
    {
      TemplateGES* tmpTemplateGES = (TemplateGES*)slaveList[i];
      tmpTemplateGES->setLedCommand(msg.data);
      printf("Set LED command to GES with name %s to value %i\n", tmpTemplateGES->getName().c_str(), msg.data);
    }
  }
}

// Creates slave objects from dictionary in launch file
std::vector<Slave*> initSlaves(ros::NodeHandle nh)
{
  std::vector<Slave*> slaves;
  std::map<std::string, std::string> slaveMap;
  nh.getParam(ros::this_node::getName() + "/slaves", slaveMap);
  std::map<std::string, std::string>::iterator i;
  for (i = slaveMap.begin(); i != slaveMap.end(); i++)
  {
    uint16 slaveNumber = (uint16)std::distance(slaveMap.begin(), i);
    if (i->second == "TEMPLATEGES")
    {
      slaves.push_back(new TemplateGES(i->first, slaveNumber));
    }
    else if (i->second == "IMC")
    {
      slaves.push_back(new IMC(i->first, slaveNumber));
    }
    else if (i->second == "PDB")
    {
      slaves.push_back(new PDB(i->first, slaveNumber));
    }
    else
    {
      slaves.push_back(new Slave(i->first, slaveNumber));
    }
  }
  return slaves;
}

// Initialize subscribers and publishers based on if certain slave types are present
void initTopics(ros::NodeHandle nh)
{
  // Find amount of each type of slave
  int nrofTEMPLATEGES, nrofPDB, nrofIMC;
  nrofTEMPLATEGES = nrofPDB = nrofIMC = 0;
  for (int i = 1; i < slaveList.size(); i++)
  {
    std::string slaveType = slaveList[i]->getType();
    if (slaveType == "IMC")
    {
      nrofIMC++;
    }
    if (slaveType == "PDB")
    {
      nrofPDB++;
    }
    if (slaveType == "TEMPLATEGES")
    {
      nrofTEMPLATEGES++;
    }
  }
  // Initialize subscribers and publishers based on presence/amount of slave types
  if (nrofTEMPLATEGES > 0)
  {
    new ros::Subscriber(nh.subscribe("march/template/data", 50, &SetTemplateDataCB));
  }
}

//--------------------------------------------------------------------
// Main
//--------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "EthercatNode");
  ros::NodeHandle nh;

  // Create all slaves based on launch file rosparams
  slaveList = initSlaves(nh);

  // Print all slave types to see if correctly initialized all slaves
  for (int i = 0; i < slaveList.size(); i++)
  {
    printf("%s\n", slaveList[i]->getType().c_str());
  }

  initTopics(nh);

  EthercatMaster ethercatMaster = EthercatMaster(nh, slaveList);

  // Set ROS rate from cycle time in launch file
  int EthercatCycleTime, EthercatFrequency;
  nh.getParam(ros::this_node::getName() + "/EthercatCycleTime", EthercatCycleTime);
  EthercatFrequency = 1000 / EthercatCycleTime;
  ros::Rate rate(EthercatFrequency);

  while (ros::ok() && ethercatMaster.inOP)
  {
    ethercatMaster.SendProcessData();
    ethercatMaster.ReceiveProcessData();
    ethercatMaster.PublishProcessData();
    ethercatMaster.MonitorSlaveConnection();
    ros::spinOnce();
    rate.sleep();
  }

  printf("End of EthercatNode\n");
  return (0);
}
