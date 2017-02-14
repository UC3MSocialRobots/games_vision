/*!
  \file
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/4/15

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

\section Parameters
  - \b "foo"
        [string] (default: "bar")
        Description of the parameter.

\section Subscriptions
  - \b "/foo"
        [xxx]
        Descrption of the subscription

\section Publications
  - \b "~foo"
        [xxx]
        Descrption of the publication

 */

#ifndef JOYPADLISTENER_H
#define JOYPADLISTENER_H

#include <sensor_msgs/Joy.h>
#include <ros/node_handle.h>

class JoypadListener {
public:
  JoypadListener() {
    ros::NodeHandle _nh_public, _nh_private("~");
    _joy_sub = _nh_public.subscribe<sensor_msgs::Joy>
        ("joy", 2,  &JoypadListener::joy_callback, this);
    _nh_private.param("axis_analog_x", axis_analog_x, 3);
    _nh_private.param("axis_analog_y", axis_analog_y, 2);
    _nh_private.param("axis_digital_x", axis_digital_x, 1);
    _nh_private.param("axis_digital_y", axis_digital_y, 0);
  }

  ~JoypadListener() {}

  /*!
     \fn event_key_pressed
     \param key_id number of the button pressed
    */
  virtual void event_key_pressed(int key_id) = 0;

  /*!
     \fn event_axe_moved
     \param axe_idx 0 for the left stick, 1 for the right one
     \param axe_x between -1 .. 1,   -1 = left
     \param axe_y between -1 .. 1,   -1 = down
    */
  virtual void event_axe_moved(int axe_idx, float axe_x, float axe_y) = 0;

private:
  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {
    // ROS_INFO("joy_callback()");

    /* A super nice drawing for the code of the buttons on the
       Logitech RumblePad 2:
       ---------------
      |  [6]     [7]  |
      |  [4]     [5]  |
       ---------------
      |   |      (3)  |
      |  -+-   (0) (2)|
      |   |      (1)  |
      / /-----------\ \
     / /             \ \
      */
    for (unsigned int button_idx = 0; button_idx < 7; ++button_idx)
      if (joy->buttons[button_idx])
        event_key_pressed(button_idx);

    // left stick
    if(joy->axes[axis_analog_x] || joy->axes[axis_analog_y])
      event_axe_moved(0, joy->axes[axis_analog_x], joy->axes[axis_analog_y]);

    // right stick
    if(joy->axes[axis_digital_x] || joy->axes[axis_digital_y])
      event_axe_moved(1, joy->axes[axis_digital_x], joy->axes[axis_digital_y]);
  } // end joy_callback();

  //////////////////////////////////////////////////////////////////////////////

  int axis_analog_x, axis_analog_y;
  int axis_digital_x, axis_digital_y;
  ros::Subscriber _joy_sub;
};

#endif // JOYPADLISTENER_H

