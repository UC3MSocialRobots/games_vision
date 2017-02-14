#include "games_vision/playzone_find_skill.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "playzone_find_skill");
  PlayzoneFindSkill skill;
  skill.check_autostart();
  ros::spin();
  return 0;
}
