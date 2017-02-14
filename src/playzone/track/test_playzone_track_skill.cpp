#include "games_vision/playzone_track_skill.h"
#include <vision_utils/img_path.h>


/** a test of the Habilidad */
void test_skill(int argc, char** argv,
                const PlayzoneTrackSkill::Mode mode,
                const std::string & filename) {
  ros::init(argc, argv, "PlayzoneTrackSkill");
  ROS_WARN("Playzone output size :");
  ROS_WARN("1 : 240x240");
  ROS_WARN("2 : 600x600");
  int choice;
  std::cin >> choice;
  int w_pz = 240, h_pz = 240;
  if (choice == 2)
    w_pz = h_pz = 600;

  PlayzoneTrackSkill skill(w_pz, h_pz);
  skill.set_mode(mode, filename);
  skill.check_autostart();
  ros::spin();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ROS_WARN("1 : MODE_ONLY_TRACK");
  ROS_WARN("2 : MODE_REPROJECT_IMAGE 1");
  ROS_WARN("3 : MODE_REPROJECT_IMAGE 2");
  ROS_WARN("4 : MODE_REPROJECT_VIDEO");

  ROS_WARN("Choice ?");
  int choice;
  std::cin >> choice;
  if (choice == 1)
    test_skill(argc, argv, PlayzoneTrackSkill::MODE_ONLY_TRACK, "");

  else if (choice == 2)
    test_skill(argc, argv, PlayzoneTrackSkill::MODE_REPROJECT_IMAGE,
               IMG_DIR "powerXML/power2.png");

  else if (choice == 3)
    test_skill(argc, argv, PlayzoneTrackSkill::MODE_REPROJECT_IMAGE,
               IMG_DIR "frenadol.png");

  else if (choice == 4)
    test_skill(argc, argv, PlayzoneTrackSkill::MODE_REPROJECT_VIDEO,
               IMG_DIR "snoopy.avi");
  return 0;
}

