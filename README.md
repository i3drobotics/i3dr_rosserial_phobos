# Rosserial Phobos control node (Arduino Micro with Adafruit BNO055)

This package builds a rosserial compatible USB IMU from an Arduino Micro and an Adafruit BNO055 IMU. The Arduino and the BNO055 breakout board communicate via I2C, so you'll have to connect the 5V, GND, SDA and SCL of the breakout board to your Arduino Micro (D2 = SDA, D3 = SCL). Adapted from Rosserial IMU node from [Vijfendertig](https://github.com/Vijfendertig/rosserial_adafruit_bno055)

The rosserial node provides two publishers:

- `/bno055/imu` with the IMU's measurements (at 50 Hz) and
- `/bno055/calib_status` with the IMU's calibration status (at 1 Hz).

Both publishers can be enabled or disabled by sending a `true` or `false` bool message to `/bno055/enable`.

When the IMU is disabled while it is fully calibrated, the calibration offsets are stored in the Arduino's EEPROM memory. If stored offsets are available, they are restored after a reset.

Camera triggers on digital pin 12 will be triggered at a rate of 10Hz. Change this by adjusting the last agument of the class initalisation in the file 'arduino_micro_imu_node.cpp' line 12:
```
i3dr_rosserial_phobos::RosserialPhobos ros_phobos(&node_handle, 20UL, 1000UL, 100UL);
```
*Note: this value should 1000/fps*

To change this pin number used on the arduino edit 'rosserial_phobos.hpp' lines 40-41:
```
int CAMERA_TRIGGER_PIN_1 = 12;
int CAMERA_TRIGGER_PIN_2 = 12;
```

## Dependencies

- [ROS](http://www.ros.org/). I used Melodic Morenia on Ubuntu 18.04 LTS, but other versions might work too.
- [rosserial_arduino](http://wiki.ros.org/rosserial_arduino).
- [Arduino SDK](https://www.arduino.cc/en/main/software). I used version 1.8.8. I downloaded the application and added it to a [git repository](https://github.com/i3drobotics/arduino-linux.git) for easy access and portability. 
- [arduino-cmake](https://github.com/queezythegreat/arduino-cmake). I [forked the repository and added a patch](https://github.com/Vijfendertig/arduino-cmake) to use `avr-gcc` and `avr-g++` from the Arduino IDE rather than the one provided with Ubuntu 18.04 LTS.
- [Adafruit_BNO055](https://github.com/adafruit/Adafruit_BNO055). The angular velocity is measured in deg/s, although the documentation states that it is expressed in rad/s ([issue](https://github.com/adafruit/Adafruit_BNO055/issues/50)). Because [REP 103](www.ros.org/reps/rep-0103.html) specifies to use radians and the sensor can be set up in rad/s, I [forked the repository and added a patch](https://github.com/Vijfendertig/Adafruit_BNO055) to read the angular velocity in rad/s directly rather than converting it afterwards.
- [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor). Used by Adafruit_BNO055.

The (patched) Adafruit_BNO055, Adafruit_Sensor and (patched) arduino-cmake dependencies are included as git submodules.

## Building

Include the package in a ROS workspace. Both building (messages, firmware...) and uploading the firmware is done using catkin_make.

Due to some internal details of rosserial_arduino's make_libraries.py script, building the package isn't as straightforward as I would like it to be. The problem is that to create our custom messages in the Arduino ros_lib library, rosserial_arduino's make_libraries.py script needs to source the workspace's setup script, which isn't available until the build is finished. See [https://github.com/ros-drivers/rosserial/issues/239] for more details. 
The most elegant workaround I found is to exclude the firmware from the default catkin_make (or CMake) target and build it manually afterwards.

So, to build the package including the firmware for the Arduino Micro, run:

- `catkin_make -DARDUINO_SDK_PATH=PATH_TO_REPO/tools/arduino-linux` (to build everything except the firmware)
- `. ./devel/setup.bash` (or the setup script for your favourite shell)
- `catkin_make i3dr_rosserial_phobos_firmware_arduino_micro` (to build the firmware)
- `catkin_make i3dr_rosserial_phobos_firmware_arduino_micro-upload` (to upload the firmware to your Arduino Micro)

## Running

Copy udev rules for usb permission:

```bash
sudo chmod a+rw /dev/ttyACM0
sudo cp udev/72-micro-devel.rules /etc/udev/rules.d/
```

Just source the workspace's setup script and run `rosrun rosserial_python serial_node.py /dev/ttyACM0`. Start the `/bno055/imu` and `/bno055/calib_status` publishers by sending a `std_msgs/Bool` `true` message to the `/bno055/enable` subscriber. The `imu_publisher_node` subscribes to the compact rosserial_adafruit_bno055/Imu messages and publishes full sensor_msgs/Imu messages (including covariances).

There is also a `i3dr_rosserial_phobos.launch` file that launches both the rosserial node and a republisher node and sends an enable command to the IMU node. The launch file accepts two parameters: `bno055_port` which specifies the IMU node's device and `bno055_frame_id` which specifies the frame_id used in the full sensor_msgs/Imu message.

The calibration status of the system, accelerometer, gyroscope and magnetometer is given with integers from 0 to 3, where 0 means uncalibrated and 3 means fully calibrated.

To visualize the orientation vector, you can use rqt's pose view plugin. Just open the plugin (`Plugins`, `Visualization`, `Pose View`) and the topic monitor (`Plugins`, `Topics`, `Topic Monitor`) and drag the `/bno055/imu` topic (if you are running the republisher node too, you can also use the `/imu` topic) from the topic monitor to the pose view.

## Calibration

Calibration of this sensor is performed automatically. When a sucessfull calibration is acheived this is saved to EEPROM on the arduino for re-loading after power-off. 
Calibrate status can be monitored by viewing the topic /bno055/calib_status

A method for movements needed to get a sucessfull calibration is detailed below:
- Power-off.
- Place sensor motionless on table, any orientation.
- Power-on.
- Wait for 0 3 0 0 or 0 3 1 0 (gyro calibrated).
- Wait 3 seconds.
- Gently pick up sensor.
- Rotate it smoothly (taking about 2 seconds) to some wild 3D orientation, hold steady for 2 seconds.
- Rotate it smoothly (taking about 2 seconds) to a different wild 3D orientation, hold steady for 2 seconds.
- Rotate it smoothly (taking about 2 seconds) to a different wild 3D orientation, hold steady for 2 seconds.
- Rotate it smoothly (taking about 2 seconds) to a different wild 3D orientation, hold steady for 2 seconds.
- Rotate it smoothly (taking about 2 seconds) to a different wild 3D orientation, hold steady for 2 seconds.
- Rotate it smoothly (taking about 2 seconds) to a different wild 3D orientation, hold steady for 2 seconds.
- If status doesn't read 3 3 1 3, go to previous step. If lucky you won't have to.
- Rotate it smoothly (taking about 2 seconds) to horizontal, place gently on table, let go, wait 2 seconds.
- 3 3 3 3 - Yay! Maybe.

Hints:
Perform rotations by hand as smoothly as possible. No bouncing or shaking or rapid twirl.
All those wild 3D orientations need to be unique. Don't repeat any orientation.
It may read 3 3 1 3 before completing all six rotations, but ignore it and do all six.

## License

MIT license, see LICENSE.md for details.

Git submodules:

- [arduino-cmake](https://github.com/queezythegreat/arduino-cmake): Unknown
- [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor): Apache License, Version 2.0
- [Adafruit_BNO055](https://github.com/adafruit/Adafruit_BNO055): MIT license
