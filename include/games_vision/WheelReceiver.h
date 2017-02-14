#ifndef WHEELRECEIVER_H
#define WHEELRECEIVER_H

#include <std_msgs/Int16.h>
#include <ros/ros.h>
#include "vision_utils/timer.h"

class WheelReceiver {
public:
  typedef int Speed;
  typedef int Angle;
  //! the error code when we could not read speed
  const static Speed ERROR_SPEED = - 1000;
  //! the error code when we could not read angle
  const static Angle ERROR_ANGLE = - 1000;
  const static Angle WHEEL_ANGLE_MAX = 180;
  const static Speed WHEEL_SPEED_MODULE_MIN = -30;
  const static Speed WHEEL_SPEED_MODULE_MAX = 100;

  /*! the time after which we consider the reception as bad
    and return 0 for speed and angle */
  const static Timer::Time TIMEOUT_MS = 1000;

  WheelReceiver()  {
    speed_module = 0;
    wheel_angle = 0;
    // suscribe
    ros::NodeHandle nh_pubic;
    _angle_sub = nh_pubic.subscribe("DRIVINGWHEEL_SET_ANGLE", 1, &WheelReceiver::angle_cb, this);
    _speed_sub = nh_pubic.subscribe("DRIVINGWHEEL_SET_SPEED", 1, &WheelReceiver::speed_cb, this);
  }

  ~WheelReceiver() {}

  /*! returns stg between WHEEL_SPEED_MODULE_MIN and WHEEL_SPEED_MODULE_MAX*/
  Speed get_speed_module() const {
    if (time_last_data.time() > TIMEOUT_MS)
      return 0;
    return speed_module;
  }

  /*! returns stg between WHEEL_ANGLE_MAX and WHEEL_ANGLE_MAX*/
  Angle get_wheel_angle() const {
    if (time_last_data.time() > TIMEOUT_MS)
      return 0;
    return wheel_angle;
  }

protected:
  void speed_cb(const std_msgs::Int16ConstPtr & msg) {
    set_speed_module(msg->data);
  }
  Speed set_speed_module(Speed d) {
    //ROS_INFO("set_speed_module(%i)", d);

    // emergency stop if needed
    if (d == ERROR_SPEED) {
      speed_module = 0;
      wheel_angle = 0;
      return 0;
    }

    time_last_data.reset();
    if (d < WHEEL_SPEED_MODULE_MIN)
      speed_module = WHEEL_SPEED_MODULE_MIN;
    else if (d > WHEEL_SPEED_MODULE_MAX)
      speed_module = WHEEL_SPEED_MODULE_MAX;
    else
      speed_module = d;
    return speed_module;
  }

  void angle_cb(const std_msgs::Int16ConstPtr & msg) {
    set_wheel_angle(msg->data);
  }
  Angle set_wheel_angle(Angle angle) {
    //ROS_INFO("set_wheel_angle(%i)", angle);

    // emergency stop if needed
    if (angle == ERROR_ANGLE) {
      speed_module = 0;
      wheel_angle = 0;
      return 0;
    }

    time_last_data.reset();
    if (angle < -WHEEL_ANGLE_MAX)
      wheel_angle = -WHEEL_ANGLE_MAX;
    else if (angle > WHEEL_ANGLE_MAX)
      wheel_angle = WHEEL_ANGLE_MAX;
    else
      wheel_angle = angle;
    return wheel_angle;
  }

  ros::Subscriber _angle_sub, _speed_sub;
  Timer time_last_data;
  int speed_module;
  int wheel_angle;
};

#endif // WHEELRECEIVER_H
