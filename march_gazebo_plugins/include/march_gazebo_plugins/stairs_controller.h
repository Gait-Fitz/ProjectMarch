// Copyright 2019 Project March.

#include <march_gazebo_plugins/obstacle_controller.h>

#ifndef MARCH_GAZEBO_PLUGINS_STAIRS_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_STAIRS_CONTROLLER_H

namespace gazebo
{
class StairsController : public ObstacleController
{
public:
  explicit StairsController(physics::ModelPtr model);
  void getGoalPosition(double time_since_start, double& goal_position_x, double& goal_position_y) override;
};
}  // namespace gazebo

#endif  // MARCH_RQT_GAIT_GENERATOR_OBSTACLECONTROLLER_H
