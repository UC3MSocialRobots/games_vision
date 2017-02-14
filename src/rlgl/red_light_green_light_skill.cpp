/*!
  \file        red_light_green_light_skill.cpp
  \author      Irene Pérez, Arnaud Ramey
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/13

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

A simple instantation of the RedLightGreenLightSkill.
 */
#include "games_vision/red_light_green_light_skill.h"
int main(int argc, char** argv) {
  ros::init(argc, argv, "red_light_green_light_skill");
  RedLightGreenLightSkill skill;
  skill.check_autostart();
  ros::spin();
}
