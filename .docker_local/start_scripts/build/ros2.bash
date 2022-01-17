#!/bin/bash

SLEEP_TIME=120
SLEEP_COUNTER=0
DONE_FILE=install/.done
cd "${HOME}"/march/ros2 || exit
if [ -f "${DONE_FILE}" ]; then rm "${DONE_FILE}"; fi;
source /opt/ros/foxy/setup.bash

while ! [ -f "${HOME}"/march/ros1/"${DONE_FILE}" ]; do
  if [ "$SLEEP_COUNTER" -gt "$SLEEP_TIME" ]; then
    SLEEP_COUNTER=0
    echo "Ros1 is not done yet, you get this message again in ${SLEEP_TIME} seconds. (do 'touch ~/march/ros1/${DONE_FILE}' if it should be done)";
  fi
  ((SLEEP_COUNTER++))
  sleep 1;
done;

colcon build
exec touch ${DONE_FILE}
