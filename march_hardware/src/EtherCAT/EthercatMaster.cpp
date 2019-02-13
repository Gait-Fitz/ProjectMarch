//
// EtherCAT master class source. Interfaces with SOEM
//

#include <ros/ros.h>
#include <march_hardware/EtherCAT/EthercatMaster.h>
#include <march_hardware/Joint.h>
#include <thread>

extern "C" {
#include "ethercat.h"
}

// Constructor
EthercatMaster::EthercatMaster(std::vector<Joint> jointList)
{
  this->jointList = jointList;

  // TODO(Isha, Martijn) make this variable or configure at runtime?
  ifname = "enp3s0";

  inOP = false;

  ROS_INFO("Starting ethercat\n");

  // Initialise SOEM, bind socket to ifname
  if (!ec_init(ifname.c_str()))
  {
    ROS_ERROR("No socket connection on %s", ifname.c_str());
    return;
  }
  ROS_INFO("ec_init on %s succeeded.\n", ifname.c_str());

  // Find and auto-config slaves
  if (ec_config_init(FALSE) <= 0)
  {
    ROS_ERROR("No slaves found, shutting down");
    return;
  }
  ROS_DEBUG("%d slaves found and configured.\n", ec_slavecount);

  // Request and wait for slaves to be in preOP state
  ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);

  //  int ecatCycleTime = 200; //TODO(Martijn) make this less magic-numberesqe
  for (int i = 0; i < jointList.size(); i++)
  {
    jointList[i].initialize();
  }

  // Configure the EtherCAT message structure depending on the PDO mapping of all the slaves
  ec_config_map(&IOmap);

  ec_configdc();

  // Wait for all slaves to reach SAFE_OP state
  ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

  ROS_DEBUG("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
            ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

  ROS_DEBUG("Request operational state for all slaves\n");
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ROS_DEBUG("Calculated workcounter %d\n", expectedWKC);
  ec_slave[0].state = EC_STATE_OPERATIONAL;

  // send one valid process data to make outputs in slaves happy
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  // request OP state for all slaves
  ec_writestate(0);
  int chk = 40;

  /* wait for all slaves to reach OP state */
  do
  {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if (ec_slave[0].state == EC_STATE_OPERATIONAL)
  {
    // All slaves in operational state
    ROS_DEBUG("Operational state reached for all slaves.\n");
    inOP = true;
    // TODO(Martijn) create parallel thread
    std::thread EcatThread(&EthercatMaster::EthercatLoop, this);

  }
  else
  {
    // Not all slaves in operational state
    ROS_ERROR("Not all slaves reached operational state");
    ec_readstate();
    for (int i = 1; i <= ec_slavecount; i++)
    {
      if (ec_slave[i].state != EC_STATE_OPERATIONAL)
      {
        ROS_DEBUG("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                  ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
      }
    }
  }
}

EthercatMaster::~EthercatMaster()
{
  inOP = false;
  ROS_DEBUG("Deconstructing EthercatMaster object\n");
  ROS_DEBUG("Request init state for all slaves\n");
  ec_slave[0].state = EC_STATE_INIT;
  ec_writestate(0);
  ROS_DEBUG("Closing EtherCAT\n");
  ec_close();
  ROS_DEBUG("Shutting down ROS\n");
//  ros::shutdown();
}

void EthercatMaster::SendProcessData()
{
  ec_send_processdata();
}

int EthercatMaster::ReceiveProcessData()
{
  return ec_receive_processdata(EC_TIMEOUTRET);
}

void EthercatMaster::PublishProcessData()
{
  // Publish for all slaves except the master (slave 0)
  //  for (int i = 1; i < jointList.size(); i++)
  //  {
  //    // TODO(Isha, BaCo)
  //    //  Determine how to publish EtherCAT process data
  //    //  Add other slave publisher implementations
  //    std::string slaveType = jointList[i]->getType();
  //    if (slaveType == "TEMPLATEGES")
  //    {
  //      TemplateGES* tmpTemplateGES = (TemplateGES*)slaveList[i];
  //      tmpTemplateGES->publish();
  //    }
  //    else if (slaveType == "IMC")
  //    {
  //    }
  //    else if (slaveType == "PDB")
  //    {
  //    }
  //    else if (slaveType == "GES")
  //    {
  //    }
  //    else
  //    {
  //      ROS_DEBUG("Error when getting GES data! Unknown GES name\n");
  //    }
  //  }
}

void EthercatMaster::MonitorSlaveConnection()
{
  // TODO(Martijn)
  //  Integrate this within EthercatMaster and Slave classes
  //  Determine how to notify developer/user
  //  ethercat_safety::monitor_slave_connection();
}

void EthercatMaster::EthercatLoop()
{
  // Parallel thread
  //    while (&& inOP)
  //      {
  //        ethercatMaster.SendProcessData();
  //        ethercatMaster.ReceiveProcessData();
  //        ethercatMaster.PublishProcessData();
  //        ethercatMaster.MonitorSlaveConnection();
  //        ros::spinOnce();
  //        rate.sleep();
  while (1)
  {
    printf("Parallel\n");
  }
}