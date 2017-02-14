/*!
  \file        launcher_driving_receiver_base.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/6/27

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file

*/

#include "vision_utils/nano_skill.h"
#include "games_vision/WheelReceiver.h"
#include "geometry_msgs/Twist.h"

class DrivingReceiverBase : public NanoSkill, public WheelReceiver {
public:
  DrivingReceiverBase() :
    NanoSkill("DRIVING_RECEIVER_BASE_START", "DRIVING_RECEIVER_BASE_STOP") {
    set_speed_module(0);
    set_wheel_angle(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  void proceso() {
    ROS_INFO("proceso()");
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    ROS_INFO("create_subscribers_and_publishers()");
    _vel_publisher  =_nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 0);
  }

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    ROS_INFO("shutdown_subscribers_and_publishers()");
    _vel_publisher.shutdown();
    set_speed_module(0);
    set_wheel_angle(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  void send_orders() {
    ROS_INFO("send_orders()");
    // make the conversion between the weird units of Wheel and real units
    double transla_speed_conv = get_speed_module();
    if (transla_speed_conv > 0)
      transla_speed_conv = 1.f * transla_speed_conv * 200 /
          WheelReceiver::WHEEL_SPEED_MODULE_MAX;
    else
      transla_speed_conv = 1.f * transla_speed_conv * 200 /
          -WheelReceiver::WHEEL_SPEED_MODULE_MIN;

    double angular_speed_conv = get_wheel_angle();
    if (angular_speed_conv > 0)
      angular_speed_conv = 1.f * angular_speed_conv * 200 /
          WheelReceiver::WHEEL_ANGLE_MAX;
    else
      angular_speed_conv = 1.f * angular_speed_conv * 200 /
          WheelReceiver::WHEEL_ANGLE_MAX;

    // send them
    geometry_msgs::Twist msg; // everything is set to 0 in this message
    msg.linear.x = transla_speed_conv;
    msg.angular.z = angular_speed_conv;
    _vel_publisher.publish(msg);
    // colocaVelocidadMotores(transla_speed_conv, angular_speed_conv);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! the base
  //CmoveBase base;
  ros::Publisher _vel_publisher;
  pthread_t thread_id;
}; // end class DrivingReceiverBase

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "launcher_driving_receiver_base");
  DrivingReceiverBase skill;
  ros::Rate rate(10);
  while(ros::ok()) {
    skill.send_orders();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

