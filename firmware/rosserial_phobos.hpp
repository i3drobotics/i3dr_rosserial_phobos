#ifndef __ROS_ADAFRUIT_BNO055__
#define __ROS_ADAFRUIT_BNO055__


#include "arduino_micro_ros.h"
#include <std_msgs/Bool.h>
#include <i3dr_rosserial_phobos/Imu.h>
#include <i3dr_rosserial_phobos/CalibrationStatus.h>
#include <Adafruit_BNO055.h>


namespace i3dr_rosserial_phobos {

  class RosserialPhobos {
    private:  // Data types and member variables.
      ros::NodeHandle * node_handle_;
      Adafruit_BNO055 sensor_;
      Imu measurements_message_;
      CalibrationStatus calibration_status_message_;
      ros::Subscriber<std_msgs::Bool, RosserialPhobos> enable_subscriber_;
      ros::Publisher measurements_publisher_;
      ros::Publisher calibration_status_publisher_;
      bool enable_;
      unsigned long int measurements_publish_interval_;
      unsigned long int measurements_last_published_;
      unsigned long int calibration_status_publish_interval_;
      unsigned long int calibration_status_last_published_;
      unsigned long int trigger_interval_;
      unsigned long int trigger_last_;
      struct StoredCalibrationData {
        // Add a valid field before and after the calibration data to detect interrupted writes.
        uint8_t signature_front;
        adafruit_bno055_offsets_t data;
        ros::Time timestamp;
        uint8_t signature_rear;
      };
      static constexpr uint16_t calibration_slots_address_ = 0U;
      static constexpr uint8_t calibration_slots_count_ = 8U;
      static constexpr uint8_t calibration_signature_ = 55U;
      int CAMERA_TRIGGER_PIN_1 = 12;
      int CAMERA_TRIGGER_PIN_2 = 12; //change this if the pin numbers used change
      int8_t current_calibration_slot_;
    public:  // Member functions.
      RosserialPhobos(ros::NodeHandle * node_handle, unsigned long int measurements_publish_interval, unsigned long int calibration_status_publish_interval, unsigned long int trigger_interval);
      ~RosserialPhobos() = default;
      void setup();
      void enable();
      void disable();
      void spinOnce();
    private:  // Member functions.
      void enableCallback(const std_msgs::Bool & message);
      void getAndPublishMeasurements();
      void getAndPublishCalibrationStatus();
      void resetStoredCalibrationData(StoredCalibrationData & data);
      void loadCalibrationFromEeprom();
      void saveCalibrationToEeprom();
      void initCameraTrigger();
      void triggerCamera();
  };
  
}


#endif
