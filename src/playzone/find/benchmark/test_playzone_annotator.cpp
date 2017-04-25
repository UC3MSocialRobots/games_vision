#include "games_vision/playzone_annotator.h"
#include "games_vision/playzone_find_skill.h"
#include <ros/ros.h>

void test_interface() {
  PlayzoneAnnotator annotator;
  annotator.from_xml_file(PLAYZONE_DIR, "pz.xml");
}

int main() {
  ROS_INFO("main()");
  test_interface();
  return 0;
}

