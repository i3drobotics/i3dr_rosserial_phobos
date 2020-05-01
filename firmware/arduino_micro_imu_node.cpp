/*
 * Arduino micro IMU node
 */


#include <Arduino.h>
#include "arduino_micro_ros.h"
#include "rosserial_phobos.hpp"


ros::NodeHandle node_handle;
i3dr_rosserial_phobos::RosserialPhobos ros_phobos(&node_handle, 20UL, 1000UL, 100UL);


void setup()
{
  node_handle.initNode();
  ros_phobos.setup();
}


void loop()
{
  node_handle.spinOnce();
  ros_phobos.spinOnce();
}
