// Copyright 2019 Project March.

#include <march_hardware_builder/HardwareBuilder.h>
#include <march_hardware_builder/HardwareConfigExceptions.h>

HardwareBuilder::HardwareBuilder(std::string yamlPath)
{
  this->yamlPath = yamlPath;
  this->robotConfig = YAML::LoadFile(yamlPath);
}

HardwareBuilder::HardwareBuilder(AllowedRobot robotName)
{
  this->yamlPath = getFilePathFromRobot(robotName);
  this->robotConfig = YAML::LoadFile(yamlPath);
}

HardwareBuilder::HardwareBuilder() = default;

march4cpp::Encoder HardwareBuilder::createEncoder(YAML::Node EncoderConfig)
{
  this->validateRequiredKeysExist(EncoderConfig, this->ENCODER_REQUIRED_KEYS, "encoder");

  int resolution = EncoderConfig["resolution"].as<int>();
  int minPositionIU = EncoderConfig["minPositionIU"].as<int>();
  int maxPositionIU = EncoderConfig["maxPositionIU"].as<int>();
  int zeroPositionIU = EncoderConfig["zeroPositionIU"].as<int>();
  auto safetyMarginRad = EncoderConfig["safetyMarginRad"].as<float>();
  return march4cpp::Encoder(resolution, minPositionIU, maxPositionIU, zeroPositionIU, safetyMarginRad);
}

march4cpp::MarchRobot HardwareBuilder::createMarchRobot(YAML::Node marchRobotConfig)
{
  std::string robotName = marchRobotConfig.begin()->first.as<std::string>();
  ROS_INFO("Starting creation of robot %s", robotName.c_str());

  std::string ifName = marchRobotConfig[robotName]["ifName"].as<std::string>();
  int ecatCycleTime = marchRobotConfig[robotName]["ecatCycleTime"].as<int>();

  YAML::Node jointListConfig = marchRobotConfig[robotName]["joints"];

  std::vector<march4cpp::Joint> jointList;

  for (std::size_t i = 0; i < jointListConfig.size(); i++)
  {
    YAML::Node jointConfig = jointListConfig[i];
    std::string jointName = jointConfig.begin()->first.as<std::string>();

    jointList.push_back(this->createJoint(jointConfig[jointName], jointName));
  }

  return march4cpp::MarchRobot(jointList, ifName, ecatCycleTime);
}

march4cpp::MarchRobot HardwareBuilder::createMarchRobot()
{
  ROS_ASSERT_MSG(this->robotConfig.Type() != YAML::NodeType::Null,
                 "Trying to create a MarchRobot without specifying a .yaml file. Please do so in the constructor of "
                 "the HardwareBuilder or in the function createMarchRobot");
  return this->createMarchRobot(robotConfig);
}

march4cpp::Joint HardwareBuilder::createJoint(YAML::Node jointConfig, std::string jointName)
{
  ROS_INFO("Starting creation of joint %s", jointName.c_str());

  march4cpp::IMotionCube imc;
  march4cpp::TemperatureGES temperatureGes;

  this->validateRequiredKeysExist(jointConfig, this->JOINT_REQUIRED_KEYS, "joint");

  if (jointConfig["imotioncube"].Type() == YAML::NodeType::Undefined)
  {
    ROS_WARN("Joint %s does not have a configuration for an IMotionCube", jointName.c_str());
  }
  else
  {
    imc = this->createIMotionCube(jointConfig["imotioncube"]);
  }

  if (jointConfig["temperatureges"].Type() == YAML::NodeType::Undefined)
  {
    ROS_WARN("Joint %s does not have a configuration for a TemperatureGes", jointName.c_str());
  }
  else
  {
    temperatureGes = this->createTemperatureGES(jointConfig["temperatureges"]);
  }

  return march4cpp::Joint(jointName, temperatureGes, imc);
}

march4cpp::IMotionCube HardwareBuilder::createIMotionCube(YAML::Node iMotionCubeConfig)
{
  this->validateRequiredKeysExist(iMotionCubeConfig, this->IMOTIONCUBE_REQUIRED_KEYS, "imotioncube");

  YAML::Node encoderConfig = iMotionCubeConfig["encoder"];
  int slaveIndex = iMotionCubeConfig["slaveIndex"].as<int>();
  return march4cpp::IMotionCube(slaveIndex, this->createEncoder(encoderConfig));
}

march4cpp::TemperatureGES HardwareBuilder::createTemperatureGES(YAML::Node temperatureGESConfig)
{
  this->validateRequiredKeysExist(temperatureGESConfig, this->TEMPERATUREGES_REQUIRED_KEYS, "temperatureges");

  int slaveIndex = temperatureGESConfig["slaveIndex"].as<int>();
  int byteOffset = temperatureGESConfig["byteOffset"].as<int>();
  return march4cpp::TemperatureGES(slaveIndex, byteOffset);
}

void HardwareBuilder::validateRequiredKeysExist(YAML::Node config, std::vector<std::string> keyList,
                                                const std::string& objectName)
{
  for (std::vector<std::string>::size_type i = 0; i != keyList.size(); i++)
  {
    if (config[keyList.at(i)].Type() == YAML::NodeType::Undefined)
    {
      throw MissingKeyException(keyList.at(i), objectName);
    }
  }
}

std::string HardwareBuilder::getFilePathFromRobot(AllowedRobot robotName)
{
  switch (robotName)
  {
    case AllowedRobot::testsetup:
      return "test";
    case AllowedRobot::march3:
      return "";
    default:
      return "";
  }
}
