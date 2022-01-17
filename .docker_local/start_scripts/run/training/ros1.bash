#!/bin/bash
# shellcheck disable=SC2034  # It is passed along to gait_default/ros1.bash.
GAIT_TYPE_ARGS="ground_gait:=true gain_tuning:=training"
. ./../gait_default/ros1.bash