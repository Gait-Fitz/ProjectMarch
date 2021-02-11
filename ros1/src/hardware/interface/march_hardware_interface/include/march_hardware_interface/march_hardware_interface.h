// Copyright 2019 Project March
#ifndef MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
#define MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
#include "march_hardware_interface/march_pdb_state_interface.h"
#include "march_hardware_interface/march_temperature_sensor_interface.h"
#include "march_hardware_interface/power_net_type.h"

#include <memory>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

#include <march_hardware/march_robot.h>
#include <march_hardware_builder/hardware_builder.h>
#include <march_shared_msgs/AfterLimitJointCommand.h>
#include <march_shared_msgs/ImcState.h>
#include <march_shared_msgs/PressureSolesData.h>

template <typename T>
using RtPublisherPtr = std::unique_ptr<realtime_tools::RealtimePublisher<T>>;

/**
 * @brief HardwareInterface to allow ros_control to actuate our hardware.
 * @details Register an interface for each joint such that they can be actuated
 *     by a controller via ros_control.
 */
class MarchHardwareInterface : public hardware_interface::RobotHW
{
public:
  MarchHardwareInterface(std::unique_ptr<march::MarchRobot> robot, bool reset_imc);

  /**
   * @brief Initialize the HardwareInterface by registering position interfaces
   * for each joint.
   */
  bool init(ros::NodeHandle& nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * Reads (in realtime) the state from the march robot.
   *
   * @param time Current time
   * @param elapsed_time Duration since last write action
   */
  void read(const ros::Time& time, const ros::Duration& elapsed_time) override;

  /**
   * @brief Perform all safety checks that might crash the exoskeleton.
   */
  void validate();

  /**
   * Writes (in realtime) the commands from the controllers to the march robot.
   *
   * @param time Current time
   * @param elapsed_time Duration since last write action
   */
  void write(const ros::Time& time, const ros::Duration& elapsed_time) override;

  /**
   * Returns the ethercat cycle time in milliseconds.
   */
  int getEthercatCycleTime() const;

  /**
   * Wait for received PDO.
   */
  void waitForPdo();

private:
  void uploadJointNames(ros::NodeHandle& nh) const;
  /**
   * Uses the num_joints_ member to resize all vectors
   * in order to avoid allocation at runtime.
   */
  void reserveMemory();
  void updatePowerNet();
  void updateHighVoltageEnable();
  void updatePowerDistributionBoard();
  void updateAfterLimitJointCommand();
  void updateIMotionCubeState();
  void updatePressureSoleData();
  void outsideLimitsCheck(size_t joint_index);
  bool MotorControllerStateCheck(size_t joint_index);
  static void getSoftJointLimitsError(const std::string& name, const urdf::JointConstSharedPtr& urdf_joint,
                                      joint_limits_interface::SoftJointLimits& error_soft_limits);

  /* Limit of the change in effort command over one cycle, can be overridden by safety controller */
  static constexpr double MAX_EFFORT_CHANGE = 5000;

  /* March hardware */
  std::unique_ptr<march::MarchRobot> march_robot_;

  /* Interfaces */
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  joint_limits_interface::PositionJointSoftLimitsInterface position_joint_soft_limits_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_soft_limits_interface_;

  MarchTemperatureSensorInterface march_temperature_interface_;
  MarchPdbStateInterface march_pdb_interface_;

  /* Shared memory */
  size_t num_joints_ = 0;

  std::vector<double> joint_position_;
  std::vector<double> joint_position_command_;

  std::vector<double> joint_velocity_;
  std::vector<double> joint_velocity_command_;

  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_last_effort_command_;

  std::vector<double> joint_temperature_;
  std::vector<double> joint_temperature_variance_;

  std::vector<joint_limits_interface::SoftJointLimits> soft_limits_;
  std::vector<joint_limits_interface::SoftJointLimits> soft_limits_error_;

  PowerNetOnOffCommand power_net_on_off_command_;
  bool master_shutdown_allowed_command_ = false;
  bool enable_high_voltage_command_ = true;
  bool reset_imc_ = false;

  bool has_actuated_ = false;

  /* Real time safe publishers */
  RtPublisherPtr<march_shared_msgs::AfterLimitJointCommand> after_limit_joint_command_pub_;
  RtPublisherPtr<march_shared_msgs::ImcState> imc_state_pub_;
  RtPublisherPtr<march_shared_msgs::PressureSolesData> pressure_sole_data_pub_;
};

#endif  // MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
