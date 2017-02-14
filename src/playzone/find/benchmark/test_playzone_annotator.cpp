#include "games_vision/playzone_annotator.h"
#include <vision_utils/img_path.h>

void test_interface() {
  PlayzoneAnnotator annotator;
  annotator.from_xml_file(IMG_DIR "pz/", "pz.xml");
}

/** tests */
int main() {
  ROS_INFO("main()");
  test_interface();
  return 0;
}

