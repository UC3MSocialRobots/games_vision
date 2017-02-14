#include "games_vision/drawer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "drawer");
  Drawer drawer;
  drawer.start();
  ros::spin();
}
