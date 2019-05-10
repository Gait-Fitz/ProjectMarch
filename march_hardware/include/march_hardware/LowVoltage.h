// Copyright 2019 Project March.
#ifndef MARCH_FAKE_SENSOR_DATA_LOWVOLTAGE_H
#define MARCH_FAKE_SENSOR_DATA_LOWVOLTAGE_H

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/NetDriverOffsets.h>
#include <march_hardware/NetMonitorOffsets.h>

namespace march4cpp
{
class LowVoltage
{
private:
  int slaveIndex;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;

  uint8 getNetsOperational();

public:
  LowVoltage(int slaveIndex, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets);
  LowVoltage();

  float getNetCurrent(int netNumber);
  bool getNetOperational(int netNumber);
  void setNetOnOff(bool on, int netNumber);

  /** @brief Override comparison operator */
  friend bool operator==(const LowVoltage& lhs, const LowVoltage& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const LowVoltage& lowVoltage)
  {
    return os << "LowVoltage(slaveIndex: " << lowVoltage.slaveIndex << ")";
  }

};

}  // namespace march4cpp
#endif  // MARCH_FAKE_SENSOR_DATA_LOWVOLTAGE_H
