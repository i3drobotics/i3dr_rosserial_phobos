//! Imu (re)publisher for Arduino Micro Adafruit BNO055 rosserial node.
/*!
 *  This node republishes the compact (excluding covariances) adafruit_bno055/Imu message from the Arduino Micro
 *  Adafruit BNO055 rosserial node as standard sensor_msgs/Imu messages, taking into account the calibration status of
 *  the BNO055 sensor.
 * 
 *  \file
 * 
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "imu_publisher.hpp"

namespace rosserial_adafruit_bno055 {

  ImuPublisher::ImuPublisher(const std::string & frame_id, const std::string & ns, bool publish_tf):
    frame_id_{frame_id},
    ns_{ns},
    publish_tf_{publish_tf},
    subscriber_compact_imu_{node_handle_.subscribe("/bno055/imu", 16, &ImuPublisher::compactImuCallback, this)},
    subscriber_calibration_status_{node_handle_.subscribe("/bno055/calib_status", 16, 
                                                          &ImuPublisher::calibrationStatusCallback, this)},
    publisher_full_imu_{node_handle_.advertise<sensor_msgs::Imu>(ns+"/imu/data_raw", 16)},
    publisher_imu_mag_{node_handle_.advertise<sensor_msgs::MagneticField>(ns+"/imu/mag", 16)},
    publisher_mag_marker_{node_handle_.advertise<sensor_msgs::MagneticField>(ns+"/imu/mag", 16)}
  {
    // Reset cached calibration status.
    cached_calibration_status_.system = 0;
    cached_calibration_status_.accelerometer = 0;
    cached_calibration_status_.gyroscope = 0;
    cached_calibration_status_.magnetometer = 0;
    cached_calibration_status_.last_saved = ros::Time();
  }

  void ImuPublisher::compactImuCallback(const Imu::ConstPtr & compact_message) {
    sensor_msgs::Imu full_message;
    full_message.header.seq = compact_message->header.seq;
    full_message.header.stamp = compact_message->header.stamp;
    full_message.header.frame_id = frame_id_;
    full_message.orientation = compact_message->orientation;
    full_message.angular_velocity = compact_message->angular_velocity;
    full_message.linear_acceleration = compact_message->linear_acceleration;

    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.seq = compact_message->header.seq;
    mag_msg.header.stamp = compact_message->header.stamp;
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field = compact_message->magnetometer;
    //magnetic_field_covariance.x = 0.000000000001;
    //magnetic_field_covariance.y = 0.000000000001;
    //magnetic_field_covariance.z = 0.000000000001;

    // Covariances. The Bosch BNO055 datasheet is pretty useless regarding the sensor's accuracy.
    // - The accuracy of the magnetometer is +-2.5deg. Users on online forums agree on that number.
    // - The accuracy of the gyroscope is unknown. I use the +-3deg/s zero rate offset. To be tested.
    // - The accuracy of the accelerometer is unknown. Based on the typical and maximum zero-g offset (+-80mg and
    //   +-150mg) and the fact that my graphs look better than that, I use 80mg. To be tested.
    // Cross-axis errors are not (yet) taken into account. To be tested.
    for(unsigned row = 0; row < 3; ++ row) {
      for(unsigned col = 0; col < 3; ++ col) {
        mag_msg.magnetic_field_covariance[row * 3 + col] = (row == col? 0.000000000001: 0.);  // +-0.000000000001
        full_message.orientation_covariance[row * 3 + col] = (row == col? 0.002: 0.);  // +-2.5deg
        full_message.angular_velocity_covariance[row * 3 + col] = (row == col? 0.003: 0.);  // +-3deg/s
        full_message.linear_acceleration_covariance[row * 3 + col] = (row == col? 0.60: 0.);  // +-80mg
      }
    }
    publisher_full_imu_.publish(full_message);
    publisher_imu_mag_.publish(mag_msg);
    if (publish_tf_)
      publishTransform(compact_message);
  }

  void ImuPublisher::publishTransform(const Imu::ConstPtr & compact_message)
  {
    static tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = compact_message->header.stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = frame_id_;
    transform.transform.rotation.w = compact_message->orientation.w;
    transform.transform.rotation.x = compact_message->orientation.x;
    transform.transform.rotation.y = compact_message->orientation.y;
    transform.transform.rotation.z = compact_message->orientation.z;
    tf_broadcaster_.sendTransform(transform);
  }

  void ImuPublisher::calibrationStatusCallback(const CalibrationStatus::ConstPtr & message) {
    cached_calibration_status_ = *message;
  }

}