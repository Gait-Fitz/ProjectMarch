//
// EtherCAT master class header. Interfaces with SOEM
//

#ifndef PROJECT_ETHERCAT_H
#define PROJECT_ETHERCAT_H

#include <ros/node_handle.h>
#include "ethercat_io.h"
#include "Slave.h"

extern "C" {
#include "osal.h"
}

class EthercatMaster
{
  std::string ifname;  // Network interface name, check ifconfig
  char IOmap[4096];    // Holds the mapping of the SOEM message
  int expectedWKC;     // Expected working counter
  std::vector<Slave*> slaveList;    // Contains all slaves

public:
  bool inOP;  // Is SOEM in operational state

  // Constructor
  explicit EthercatMaster(ros::NodeHandle nh, std::vector<Slave*> slaves);
  // Destructor
  ~EthercatMaster();
  // Send Process Data over EtherCAT
  void SendProcessData();
  // Receive Process Data over EtherCAT
  int ReceiveProcessData();
  // Publish received Process Data per slave
  void PublishProcessData();
  // Monitor slave connection
  void MonitorSlaveConnection();

};

#endif  // PROJECT_ETHERCAT_H
