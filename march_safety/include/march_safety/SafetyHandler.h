// Copyright 2019 Project March.
#ifndef MARCH_WS_SAFETYHANDLER_H
#define MARCH_WS_SAFETYHANDLER_H

#include <string>
#include <ros/ros.h>
#include <march_shared_resources/Error.h>

class SafetyHandler {
  ros::NodeHandle* n;
  ros::Publisher* error_publisher;

public:
  SafetyHandler(ros::Publisher* errorPublisher, ros::NodeHandle* n);

  void publishError(int8_t errorSeverity, std::string message);
};



#endif  // MARCH_WS_SAFETYHANDLER_H
