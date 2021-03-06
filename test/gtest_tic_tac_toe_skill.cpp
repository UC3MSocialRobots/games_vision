/*!
  \file        gtest_tic_tac_toe_skill.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/17

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
#include "games_vision/tic_tac_toe_skill.h"
#include "games_vision/playzone_find_skill.h"
#include <vision_utils/img_path.h>
#define ASSERT_TRUE_TIMEOUT(cond, timeout) { vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

//#define DISPLAY

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ctor) {
  TicTacToeSkill tictac;
  ASSERT_FALSE(tictac.can_have_playzone());
  ASSERT_FALSE(tictac.is_playzone_detection_success());
  ASSERT_FALSE(tictac.is_playzone_processing_success());
  ASSERT_EQ(tictac.get_status(), PlayzoneSequentialUser::NEVER_RUN);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, no_PlayzoneFind) {
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  TicTacToeSkill tictac;
  tictac.start(); // create_subscribers_and_publishers() contains a sleep(1)
  ASSERT_FALSE(tictac.can_have_playzone()); // no PlayzoneFind
  ASSERT_TRUE_TIMEOUT(tictac.get_status() == PlayzoneSequentialUser::NEVER_RUN, 1)
      << "status:" << tictac.get_status();

  // start the foo skill to try and obtain the pz
  ros::Publisher touch_pub = nh_public.advertise<std_msgs::String>("capacitive_touch", 1);
  std_msgs::String touch_msg;
  touch_pub.publish(touch_msg);
  ASSERT_TRUE_TIMEOUT(tictac.get_status() == PlayzoneSequentialUser::FAILURE_NO_PZ, 10)
      << "status:" << tictac.get_status();
  ASSERT_FALSE(tictac.is_playzone_detection_success());
  ASSERT_FALSE(tictac.is_playzone_processing_success());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, comm_with_pz_finder_no_img) {
  if (!vision_utils::rosmaster_alive()) return;
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  PlayzoneFindSkill pz_finder;
  pz_finder.start();
  TicTacToeSkill tictac;
  tictac.start(); // create_subscribers_and_publishers() contains a sleep(1)
  ASSERT_TRUE_TIMEOUT(tictac.can_have_playzone(), 1);

  // start the foo skill to try and obtain the pz
  ros::Publisher touch_pub = nh_public.advertise<std_msgs::String>("capacitive_touch", 1);
  std_msgs::String touch_msg;
  touch_pub.publish(touch_msg);
  ASSERT_TRUE_TIMEOUT(tictac.get_status() == PlayzoneSequentialUser::FAILURE_NO_PZ, 10)
      << "status:" << tictac.get_status();
  ASSERT_FALSE(tictac.is_playzone_detection_success());
  ASSERT_FALSE(tictac.is_playzone_processing_success());
}

////////////////////////////////////////////////////////////////////////////////

void test_single_img(const std::string & filename,
                     PlayzoneSequentialUser::Status exp_status,
                     const std::string & exp_board = "") {
  if (!vision_utils::rosmaster_alive()) return;
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  // create pubs / subs
  PlayzoneFindSkill pz_finder;
  pz_finder.start();
  TicTacToeSkill tictac;
  ros::Publisher touch_pub = nh_public.advertise<std_msgs::String>("capacitive_touch", 1);
  image_transport::ImageTransport transport(nh_public);
  image_transport::Publisher rgb_pub = transport.advertise(pz_finder.get_image_topic(), 1, false); // no latch
  // start the playzone tictac
  tictac.start(); // create_subscribers_and_publishers() contains a sleep(1)
  ASSERT_TRUE_TIMEOUT(tictac.can_have_playzone(), 5)
      << "server:" << pz_finder.get_playzone_service()
      << ", client:" << tictac.get_playzone_service();
  // start the foo skill to try and obtain the pz
  std_msgs::String touch_msg;
  touch_pub.publish(touch_msg);
  // publish image
  cv_bridge::CvImage rgb_bridge;
  rgb_bridge.image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  ASSERT_FALSE(rgb_bridge.image.empty()) << "reading '" << filename << "' failed";
  rgb_bridge.encoding = sensor_msgs::image_encodings::BGR8;

  vision_utils::Timer timer;
  while (timer.getTimeSeconds() < 10 && tictac.get_status() != exp_status) {
    printf("Publishing image on '%s'\n", rgb_pub.getTopic().c_str());
    rgb_bridge.header.stamp = ros::Time::now();
    rgb_pub.publish(rgb_bridge.toImageMsg());
    ros::spinOnce();
    usleep(500 * 1000);
  }
  printf("tictac:'%s'\n", tictac.get_board_as_string().c_str());
  if (!exp_board.empty())
    ASSERT_TRUE(tictac.get_board_as_string() == exp_board)
        << "board:" << tictac.get_board_as_string() << ", exp:board" << exp_board;

#ifdef DISPLAY
  tictac.display();
  pz_finder.display();
  cv::waitKey(0);
#endif
  ASSERT_EQ(tictac.get_status(), exp_status);
}

TEST(TestSuite, test_single_no_tic_tac_toe_game) {
  test_single_img(TICTACTOE_DIR + "pz01.jpg",
                  PlayzoneSequentialUser::FAILURE_PROCESSING_FAILED);
}
TEST(TestSuite, test_single_empty_grid) {
  test_single_img(TICTACTOE_DIR + "pz40.jpg",
                  PlayzoneSequentialUser::SUCCESS_FOUND_AND_PROCESSED,
                  "---------");
}
TEST(TestSuite, test_single_full_grid) {
  test_single_img(TICTACTOE_DIR + "sample_frame.jpg",
                  PlayzoneSequentialUser::SUCCESS_FOUND_AND_PROCESSED,
                  "XO-OOXXXO");
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, pingpong) {
  if (!vision_utils::rosmaster_alive()) return;
  // create pubs / subs
  TicTacToeSkill ping, pong;
  pong.set_drawer_out_topic("pong2ping");
  ping.set_drawer_in_topic("pong2ping");
  ping.set_drawer_out_topic("ping2pong");
  pong.set_drawer_in_topic("ping2pong");
  ping.set_Maggie_pions(TicTacToeSolver::CROSS);
  pong.set_Maggie_pions(TicTacToeSolver::ROUND);
  ping.start();
  pong.start();
  ros::spinOnce();
  ASSERT_TRUE_TIMEOUT(ping.can_have_drawer(), 1)
      << "server:" << pong.get_drawer_out_topic()
      << ", client:" << ping.get_drawer_in_topic();
  ASSERT_TRUE_TIMEOUT(pong.can_have_drawer(), 1)
      << "server:" << ping.get_drawer_out_topic()
      << ", client:" << pong.get_drawer_in_topic();
  int nspins = 0;
  while(nspins++ < 10) {
    printf("\n\nnspins:%i\n", nspins);
    ros::spinOnce();
    sleep(1);
#ifdef DISPLAY
    ping.display();
    pong.display();
    cv::waitKey(0);
#endif
  } // end while (spins)
  ASSERT_TRUE(ping.get_game_status() == TicTacToeSolver::ANSWER_MAGGIE_WON
              || ping.get_game_status() == TicTacToeSolver::ANSWER_PLAYER_WON
              || ping.get_game_status() == TicTacToeSolver::ANSWER_DRAW)
      << "ping.get_game_status():" << ping.get_game_status();
  ASSERT_TRUE(pong.get_game_status() == TicTacToeSolver::ANSWER_MAGGIE_WON
              || pong.get_game_status() == TicTacToeSolver::ANSWER_PLAYER_WON
              || pong.get_game_status() == TicTacToeSolver::ANSWER_DRAW)
      << "pong.get_game_status():" << pong.get_game_status();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  ros::init(argc, argv, "gtest_TicTacToeSkill");
  ros::NodeHandle nh; // otherwise bug: ROS_INFO() not displayed...
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
