/*!
  \file        launcher_driving_emitter_joypad.cpp
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
#include "games_vision/joypadlistener.h"

class DrivingEmitterJoypad : public WheelEmitter, JoypadListener {
public:
  DrivingEmitterJoypad() {
    ROS_INFO("DrivingEmitterJoypad ctor");
  }

  /*! for a x in -1..1, will widen the range of values around 0
     and squeeze the ones around -1 and 1 */
  inline float widen_low_value(float x) {
    int signum_x = (x < 0 ? -1 : 1);
    return pow(fabs(x), 2) * signum_x;
    //return fmin( signum_x * x * x * 2, 1);
  }

  virtual void event_key_pressed(int key_id) {
  }

  virtual void event_axe_moved(int axe_idx, float axe_x, float axe_y) {
    ROS_INFO("event_axe_moved(axe:%i, x:%f, y:%f)",
                 axe_idx, axe_x, axe_y);
    if (axe_idx == 0) { // left
      emit_angle( widen_low_value(axe_x) * WHEEL_ANGLE_MAX );
    }
    else if (axe_idx == 1) { // right
      if (axe_y > 0) // forwards
        emit_speed( widen_low_value(axe_y) * WHEEL_SPEED_MODULE_MAX );
      else // backwards
        emit_speed( widen_low_value(axe_y) * -WHEEL_SPEED_MODULE_MIN );
    }
  }
}; // end class DrivingEmitterJoypad

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "launcher_driving_emitter_joypad");
  DrivingEmitterJoypad skill;
  ros::spin();
  return 0;
}


