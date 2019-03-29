// Copyright 2019 Project March.

#ifndef MARCH4CPP__MARCH4_H
#define MARCH4CPP__MARCH4_H

#include <string>
#include <vector>
#include <stdint.h>

#include <march_hardware/Joint.h>

#include <march_hardware/EtherCAT/EthercatMaster.h>

namespace march4cpp
{
class MarchRobot
{
private:
  std::unique_ptr<EthercatMaster> ethercatMaster;

  enum class AllowedRobot
  {
    testsetup,
    march4,
    march3
  };

public:
  ::std::vector<Joint> jointList;

  MarchRobot(::std::vector<Joint> jointList, ::std::string ifName, int ecatCycleTime);

  void startEtherCAT();

  void stopEtherCAT();

  int getMaxSlaveIndex();

  bool hasValidSlaves();

  bool isEthercatOperational();

  Joint getJoint(::std::string jointName);
};
}  // namespace march4cpp
#endif
