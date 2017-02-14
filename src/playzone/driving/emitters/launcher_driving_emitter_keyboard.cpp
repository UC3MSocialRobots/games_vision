/*!
  \file        launcher_driving_emitter_keyboard.cpp
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
#include "games_vision/WheelEmitter.h"
#include "vision_utils/nano_skill.h"
#include <opencv2/highgui/highgui.hpp>

class DrivingEmitterKeyboard : public NanoSkill, public WheelEmitter {
public:
  DrivingEmitterKeyboard() :
    NanoSkill("DRIVING_EMITTER_KEYBOARD_START", "DRIVING_EMITTER_KEYBOARD_STOP")
  {
    ROS_INFO("DrivingEmitterKeyboard ctor");
    angle = 0;
    speed = 0;
    interface.create(50, 200);
    interface.setTo(255);
    vision_utils::draw_text_centered(interface, "DrivingEmitterKeyboard",
                                    cv::Point(interface.cols / 2, interface.rows / 2),
                                    cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0) );
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    ROS_INFO("create_subscribers_and_publishers()");
    cvStartWindowThread();
    cv::namedWindow("DrivingEmitterKeyboard");
    cv::imshow("DrivingEmitterKeyboard", interface);
  } // end create_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

  void proceso() {
    // ROS_INFO("proceso()");
    char c = cv::waitKey(20);
    if (c == 27)
      stop();
    // angle
    if (c == 81) // left
      angle = emit_angle(angle - 5);
    else if (c == 83) // right
      angle = emit_angle(angle + 5);
    else
      angle = emit_angle(angle);
    // speeds
    if (c == 82)
      speed = emit_speed(speed + 5);
    else if (c == 84)
      speed = emit_speed(speed - 5);
    else
      speed = emit_speed(speed);
  }

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers()  {
    ROS_INFO("shutdown_subscribers_and_publishers()");
    cvDestroyWindow("DrivingEmitterKeyboard");
  } // end shutdown_subscribers_and_publishers()

protected:
  int angle;
  int speed;
  cv::Mat3b interface;
}; // end class DrivingEmitterKeyboard

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "launcher_driving_emitter_keyboard");
  DrivingEmitterKeyboard skill;
  skill.check_autostart();
  ros::spin();
  return 0;
}

