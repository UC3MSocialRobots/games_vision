#include "games_vision/playzone_annotator.h"
#include <vision_utils/img_path.h>
#include <ros/ros.h>

void test_interface() {
  PlayzoneAnnotator annotator;
  annotator.from_xml_file(vision_utils::IMG_DIR()+"pz/", "pz.xml");
}

int main() {
  ROS_INFO("main()");
  test_interface();
  return 0;
}

