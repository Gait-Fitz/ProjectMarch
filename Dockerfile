################################################################
#
# IMPORTANT!
#
# This file has been automatically generated by .scripts/generate_dockerfile.sh
# You should NOT change this file manually!
# If you have added a new dependency, running .scripts/generate_dockerfile.sh from
# the root of the repository should be enough.
#
################################################################


FROM ros:foxy-ros1-bridge-focal
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && apt upgrade -y && apt install -y apt-utils && apt install -y ros-noetic-ros-base ros-foxy-ros-base
RUN apt update && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator build-essential python3-colcon-common-extensions python3-pip python3-pip python3-catkin-pkg python3-catkin-lint clang-tidy clang libssl-dev wget cmake git git-lfs libbullet-dev python3-pytest-cov python3-setuptools python3-vcstool ssh clang-format parallel tmux && apt install -y --no-install-recommends libasio-dev libtinyxml2-dev libcunit1-dev
RUN python3 -m pip install mock argcomplete pytest-repeat pytest-rerunfailures pytest Flask==1.1.2 numpy_ringbuffer==0.2.1 pyqtgraph==0.11.0  flake8 flake8-codeclimate flake8-black flake8-2020 flake8-assertive flake8-bandit flake8-blind-except flake8-bugbear flake8-builtins flake8-commas flake8-comprehensions flake8-deprecated flake8-eradicate
RUN bash -c "source /opt/ros/foxy/local_setup.bash && rosdep update && apt update && apt install -y gazebo11 libboost-all-dev libyaml-cpp-dev python3-mock python3-numpy python3-parameterized python3-pytest python3-scipy python3-setuptools python3-setuptools ros-foxy-actionlib-msgs ros-foxy-ament-cmake ros-foxy-ament-cmake-gtest ros-foxy-ament-cmake-pytest ros-foxy-ament-cmake-python ros-foxy-ament-lint-auto ros-foxy-ament-lint-common ros-foxy-builtin-interfaces ros-foxy-control-msgs ros-foxy-diagnostic-aggregator ros-foxy-diagnostic-msgs ros-foxy-gazebo-msgs ros-foxy-geometry-msgs ros-foxy-kdl-parser ros-foxy-launch-ros ros-foxy-launch-testing-ament-cmake ros-foxy-orocos-kdl ros-foxy-rclcpp ros-foxy-rclcpp-components ros-foxy-rclcpp-lifecycle ros-foxy-rclpy ros-foxy-ros-testing ros-foxy-ros2test ros-foxy-rosgraph-msgs ros-foxy-rosidl-default-generators ros-foxy-rosidl-default-runtime ros-foxy-rqt-gui ros-foxy-rqt-gui-py ros-foxy-rqt-robot-monitor ros-foxy-sensor-msgs ros-foxy-std-msgs ros-foxy-tf2-ros ros-foxy-trajectory-msgs ros-foxy-urdf ros-foxy-urdfdom-headers ros-foxy-urdfdom-py ros-foxy-xacro ros-noetic-actionlib ros-noetic-angles ros-noetic-camera-info-manager ros-noetic-catkin ros-noetic-code-coverage ros-noetic-control-msgs ros-noetic-control-toolbox ros-noetic-controller-interface ros-noetic-controller-manager ros-noetic-dynamic-reconfigure ros-noetic-gazebo-msgs ros-noetic-gazebo-plugins ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs ros-noetic-geometry-msgs ros-noetic-hardware-interface ros-noetic-image-transport ros-noetic-imu-filter-madgwick ros-noetic-joint-limits-interface ros-noetic-joint-state-controller ros-noetic-joint-state-publisher ros-noetic-joy ros-noetic-message-generation ros-noetic-message-runtime ros-noetic-moveit-commander ros-noetic-moveit-fake-controller-manager ros-noetic-moveit-kinematics ros-noetic-moveit-planners-ompl ros-noetic-moveit-ros-benchmarks ros-noetic-moveit-ros-move-group ros-noetic-moveit-ros-visualization ros-noetic-moveit-ros-warehouse ros-noetic-moveit-setup-assistant ros-noetic-pcl-conversions ros-noetic-pcl-ros ros-noetic-pluginlib ros-noetic-realsense2-camera ros-noetic-realtime-tools ros-noetic-robot-state-publisher ros-noetic-roscpp ros-noetic-roslib ros-noetic-rospy ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-rosserial-python ros-noetic-rostest ros-noetic-rosunit ros-noetic-rqt-gui ros-noetic-rqt-gui-py ros-noetic-rviz ros-noetic-sensor-msgs ros-noetic-soem ros-noetic-sound-play ros-noetic-std-msgs ros-noetic-std-srvs ros-noetic-tf ros-noetic-tf2-geometry-msgs ros-noetic-tf2-ros ros-noetic-trajectory-msgs ros-noetic-urdf ros-noetic-urdfdom-py ros-noetic-visualization-msgs ros-noetic-warehouse-ros-mongo ros-noetic-xacro"

