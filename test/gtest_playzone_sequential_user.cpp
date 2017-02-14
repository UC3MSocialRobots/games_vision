/*!
  \file        gtest_playzone_sequential_user.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/16

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
// Bring in gtest
#include <gtest/gtest.h>
#include <vision_utils/rosmaster_alive.h>
#include "games_vision/playzone_sequential_user.h"
#include "games_vision/playzone_find_skill.h"
#include <vision_utils/img_path.h>
#define ASSERT_TRUE_TIMEOUT(cond, timeout) { vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

//#define DISPLAY

//! a foo skill that will get the playzone when it gets the "FOO_ACTION" message
class FooPlayzoneSequentialUser : public PlayzoneSequentialUser{
public:
  FooPlayzoneSequentialUser() :
    PlayzoneSequentialUser("FOO_START", "FOO_STOP") {
    is_action_cb_terminated = false;
  }

  void create_subscribers_and_publishers_playzone() {
    printf("FooPlayzoneSequentialUser::create_subscribers_and_publishers_playzone()\n");
    _action_sub = _nh_public.subscribe("FOO_ACTION", 1, &FooPlayzoneSequentialUser::action_cb, this);
  }

  void shutdown_subscribers_and_publishers_playzone() {
    printf("FooPlayzoneSequentialUser::shutdown_subscribers_and_publishers_playzone()\n");
  }

  //! get the verdammte playzone!
  void action_cb(const std_msgs::Int16ConstPtr &) {
    printf("FooPlayzoneSequentialUser::action_cb()\n");
    get_and_process_pz();
    is_action_cb_terminated = true;
  }

  bool process_pz(const cv::Mat3b & pz) {
    printf("FooPlayzoneSequentialUser::process_pz()\n");
    _playzone_illus = cv::Scalar::all(255) - pz;
    return true;
  }

  void do_stuff_before_get_playzone() {
    printf("FooPlayzoneSequentialUser::do_stuff_before_get_playzone()\n");
  }

  void do_stuff_after_get_playzone(bool was_find_and_process_success) {
    printf("FooPlayzoneSequentialUser::do_stuff_after_get_playzone(success:%i)\n",
           was_find_and_process_success);
  }

  ros::Subscriber _action_sub;
  bool is_action_cb_terminated;
}; // end class FooPlayzoneSequentialUser

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ctor) {
  if (!vision_utils::rosmaster_alive()) return;
  FooPlayzoneSequentialUser playzone_user;
  ASSERT_FALSE(playzone_user.has_playzone_service());
  ASSERT_FALSE(playzone_user.is_playzone_detection_success());
  ASSERT_FALSE(playzone_user.is_playzone_processing_success());
  ASSERT_TRUE(playzone_user.get_status() == PlayzoneSequentialUser::NEVER_RUN);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, no_PlayzoneFind) {
  if (!vision_utils::rosmaster_alive()) return;
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  FooPlayzoneSequentialUser playzone_user;
  playzone_user.start(); // create_subscribers_and_publishers() contains a sleep(1)
  ASSERT_FALSE(playzone_user.has_playzone_service()); // no PlayzoneFind

  // start the foo skill to try and obtain the pz
  ros::Publisher action_pub = nh_public.advertise<std_msgs::Int16>("FOO_ACTION", 1);
  std_msgs::Int16 action_msg;
  action_pub.publish(action_msg);
  ASSERT_TRUE_TIMEOUT(playzone_user.get_status() == PlayzoneSequentialUser::NEVER_RUN, 1);
  ASSERT_TRUE_TIMEOUT(playzone_user.is_action_cb_terminated, 10);
  ASSERT_TRUE_TIMEOUT(playzone_user.get_status() == PlayzoneSequentialUser::FAILURE_NO_PZ, 10);
  ASSERT_FALSE(playzone_user.is_playzone_detection_success());
  ASSERT_FALSE(playzone_user.is_playzone_processing_success());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, comm_with_pz_finder_no_img) {
  if (!vision_utils::rosmaster_alive()) return;
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  PlayzoneFindSkill pz_finder;
  FooPlayzoneSequentialUser playzone_user;
  playzone_user.start(); // create_subscribers_and_publishers() contains a sleep(1)
  ASSERT_TRUE(playzone_user.has_playzone_service());

  // start the foo skill to try and obtain the pz
  ros::Publisher action_pub = nh_public.advertise<std_msgs::Int16>("FOO_ACTION", 1);
  std_msgs::Int16 action_msg;
  action_pub.publish(action_msg);
  ASSERT_TRUE_TIMEOUT(playzone_user.get_status() == PlayzoneSequentialUser::NEVER_RUN, 1);
  ASSERT_TRUE_TIMEOUT(playzone_user.is_action_cb_terminated, 10);
  ASSERT_TRUE_TIMEOUT(playzone_user.get_status() == PlayzoneSequentialUser::FAILURE_NO_PZ, 10);
  ASSERT_FALSE(playzone_user.is_playzone_detection_success());
  ASSERT_FALSE(playzone_user.is_playzone_processing_success());
}

////////////////////////////////////////////////////////////////////////////////

void test_single_img(const std::string & filename,
                     PlayzoneSequentialUser::Status exp_status) {
  if (!vision_utils::rosmaster_alive()) return;
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  // create pubs / subs
  PlayzoneFindSkill pz_finder;
  FooPlayzoneSequentialUser playzone_user;
  ros::Publisher action_pub = nh_public.advertise<std_msgs::Int16>("FOO_ACTION", 1);
  image_transport::ImageTransport transport(nh_public);
  image_transport::Publisher rgb_pub = transport.advertise(pz_finder.get_image_topic(), 1, true); // latch
  // start the playzone user
  playzone_user.start(); // create_subscribers_and_publishers() contains a sleep(1)
  ASSERT_TRUE(playzone_user.has_playzone_service());
  // start the foo skill to try and obtain the pz
  std_msgs::Int16 action_msg;
  action_pub.publish(action_msg);
  // publish image
  cv_bridge::CvImage rgb_bridge;
  rgb_bridge.image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  rgb_bridge.encoding = sensor_msgs::image_encodings::BGR8;

  vision_utils::Timer timer;
  while (timer.getTimeSeconds() < 10 && playzone_user.get_status() != exp_status) {
    printf("Publishing image on '%s'\n", rgb_pub.getTopic().c_str());
    rgb_bridge.header.stamp = ros::Time::now();
    rgb_pub.publish(rgb_bridge.toImageMsg());
    ros::spinOnce();
    usleep(100 * 1000);
  }

#ifdef DISPLAY
  playzone_user.display();
  pz_finder.display();
  cv::waitKey(0);
#endif
  ASSERT_TRUE(playzone_user.get_status() == exp_status)
      << "status:" << playzone_user.get_status() << ", exp:status" << exp_status;
}

// failures
TEST(TestSuite, test_single_img_nopz) {
  test_single_img(IMG_DIR "balloon.png", PlayzoneSequentialUser::FAILURE_NO_PZ);
}
TEST(TestSuite, test_single_img39) {
  test_single_img(IMG_DIR "pz/pz39.jpg", PlayzoneSequentialUser::FAILURE_NO_PZ);
}
//successes
TEST(TestSuite, test_single_img01) {
  test_single_img(IMG_DIR "pz/pz01.jpg", PlayzoneSequentialUser::SUCCESS_FOUND_AND_PROCESSED);
}
TEST(TestSuite, test_single_img08) {
  test_single_img(IMG_DIR "pz/pz08.jpg", PlayzoneSequentialUser::SUCCESS_FOUND_AND_PROCESSED);
}
TEST(TestSuite, test_single_img09) {
  test_single_img(IMG_DIR "pz/pz09.jpg", PlayzoneSequentialUser::SUCCESS_FOUND_AND_PROCESSED);
}
TEST(TestSuite, test_single_img10) {
  test_single_img(IMG_DIR "pz/pz10.jpg", PlayzoneSequentialUser::SUCCESS_FOUND_AND_PROCESSED);
}
TEST(TestSuite, test_single_img11) {
  test_single_img(IMG_DIR "pz/pz11.jpg", PlayzoneSequentialUser::SUCCESS_FOUND_AND_PROCESSED);
}
TEST(TestSuite, test_single_img12) {
  test_single_img(IMG_DIR "pz/pz12.jpg", PlayzoneSequentialUser::SUCCESS_FOUND_AND_PROCESSED);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  ros::init(argc, argv, "gtest_PlayzoneSequentialUser");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
