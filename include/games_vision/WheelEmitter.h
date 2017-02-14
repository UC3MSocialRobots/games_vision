#ifndef WHEELEMITTER_H
#define WHEELEMITTER_H

#include <std_msgs/Int16.h>
#include <ros/ros.h>

class WheelEmitter {
public:
  typedef int Speed;
  typedef int Angle;
  //! the error code when we could not read speed
  const static Speed ERROR_SPEED = - 1000;
  //! the error code when we could not read angle
  const static Angle ERROR_ANGLE = - 1000;

  /*!                        0
                            ---+----
                           /   |    \
                          /    |     \
                         |     |      |
      -WHEEL_ANGLE_MAX < +-----+------+ > +WHEEL_ANGLE_MAX

      */
  const static Angle WHEEL_ANGLE_MAX = 180;
  const static Speed WHEEL_SPEED_MODULE_MIN = -30;
  const static Speed WHEEL_SPEED_MODULE_MAX = 100;

  //////////////////////////////////////////////////////////////////////////////

  WheelEmitter() {
    ros::NodeHandle nh_pubic;
    _angle_pub = nh_pubic.advertise<std_msgs::Int16>("DRIVINGWHEEL_SET_ANGLE", 1);
    _speed_pub = nh_pubic.advertise<std_msgs::Int16>("DRIVINGWHEEL_SET_SPEED", 1);
  }

  //////////////////////////////////////////////////////////////////////////////

  Angle emit_angle(Angle wheel_angle0) {
    // ROS_INFO("emit_angle(%i)", wheel_angle0);
    Angle wheel_angle = wheel_angle0;
    if (wheel_angle < -WHEEL_ANGLE_MAX)
      wheel_angle = -WHEEL_ANGLE_MAX;
    else if (wheel_angle > WHEEL_ANGLE_MAX)
      wheel_angle = WHEEL_ANGLE_MAX;

    std_msgs::Int16 msg; msg.data = wheel_angle;
    _angle_pub.publish(msg);
    return wheel_angle;
  }

  //////////////////////////////////////////////////////////////////////////////

  Speed emit_speed(Speed speed_module0) {
    // ROS_INFO("emit_speed(%i)", speed_module0);
    Speed speed_module = speed_module0;
    if (speed_module < WHEEL_SPEED_MODULE_MIN)
      speed_module = WHEEL_SPEED_MODULE_MIN;
    else if (speed_module > WHEEL_SPEED_MODULE_MAX)
      speed_module = WHEEL_SPEED_MODULE_MAX;

    std_msgs::Int16 msg; msg.data = speed_module;
    _speed_pub.publish(msg);
    return speed_module;
  }

private:
  ros::Publisher _angle_pub, _speed_pub;
}; // end class WheelEmitter

#endif // WHEELEMITTER_H
