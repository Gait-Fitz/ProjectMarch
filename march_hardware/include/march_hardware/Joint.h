#ifndef MARCH4CPP__JOINT_H
#define MARCH4CPP__JOINT_H

#include <string>

#include <march_hardware/IMotionCube.h>
#include <march_hardware/TemperatureSensor.h>
namespace march4cpp
{
class Joint
{
private:
  std::string name;
  IMotionCube iMotionCube;
  TemperatureSensor temperatureSensor;

public:
  Joint(std::string name, TemperatureSensor temperatureSensor, IMotionCube iMotionCube);
  Joint(std::string name, TemperatureSensor temperatureSensor);
  Joint(std::string name, IMotionCube iMotionCube);

  void initialize(int ecatCycleTime);
  void actuateRad(float targetPositionRad);

  float getAngleRad();
  float getTemperature();

  std::string getName();
  int getTemperatureSensorSlaveIndex();
  int getIMotionCubeSlaveIndex();
  IMotionCube getIMotionCube();

  bool hasIMotionCube();
  bool hasTemperatureSensor();
};
}  // namespace march4cpp
#endif
