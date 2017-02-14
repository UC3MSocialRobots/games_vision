#include "games_vision/tic_tac_toe_skill.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tic_tac_toe_skill");
  TicTacToeSkill skill;
  skill.check_autostart();
  ros::spin();
  return 0;
}
