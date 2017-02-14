/*!
  \file        gtest_playzone_find.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/15

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
#include <opencv2/highgui/highgui.hpp>
#include "games_vision/playzone_find.h"
#include <vision_utils/img_path.h>

//#define DISPLAY

TEST(TestSuite, ctor) {
  if (!vision_utils::rosmaster_alive()) return;
  PlayzoneFind finder(100, 100);
  ASSERT_TRUE(finder.get_current_status() == PlayzoneFind::NEVER_RUN);
}


////////////////////////////////////////////////////////////////////////////////

void test_fail(const cv::Mat3b & test_img, PlayzoneFind::Status exp_status) {
  if (!vision_utils::rosmaster_alive()) return;
  PlayzoneFind finder(100, 100);
  finder.set_input(&test_img);
  ASSERT_FALSE(finder.find_playzone());
  ASSERT_TRUE(finder.get_current_status() == exp_status);
}

TEST(TestSuite, empty_img) { test_fail(cv::Mat3b(0, 0),
                                       PlayzoneFind::FAILURE_NO_GOOD_COMP); }
TEST(TestSuite, black_img) { test_fail(cv::Mat3b(640, 480, cv::Vec3b(0, 0, 0)),
                                       PlayzoneFind::FAILURE_NO_GOOD_COMP); }
TEST(TestSuite, white_img) { test_fail(cv::Mat3b(640, 480, cv::Vec3b(255, 255, 255)),
                                       PlayzoneFind::FAILURE_NO_GOOD_COMP); }
TEST(TestSuite, arnaud001) { test_fail(cv::imread(vision_utils::IMG_DIR() + "arnaud001.png", CV_LOAD_IMAGE_COLOR),
                                       PlayzoneFind::FAILURE_NO_GOOD_COMP); }

////////////////////////////////////////////////////////////////////////////////
inline void test_success(const std::string filename = vision_utils::IMG_DIR() + "pz/pz05.jpg") {
  if (!vision_utils::rosmaster_alive()) return;
  cv::Mat3b test_img = cv::imread(filename);
  PlayzoneFind finder (300, 300);
  finder.set_input(&test_img);
  bool find_playzone_success = finder.find_playzone();
  ROS_INFO("time:'%s'", finder.get_current_times().to_string().c_str());
#ifdef DISPLAY
  cv::imshow("orig", test_img);
  finder.display();
  cv::waitKey(0);
#endif

  ASSERT_TRUE(find_playzone_success);
  std::vector<cv::Point2i> corners;
  finder.get_corner_list(corners);
  ASSERT_TRUE(corners.size() == 4);
}

TEST(TestSuite, pz01) { test_success(vision_utils::IMG_DIR() + "pz/pz01.jpg"); }
TEST(TestSuite, pz08) { test_success(vision_utils::IMG_DIR() + "pz/pz08.jpg"); }
TEST(TestSuite, pz09) { test_success(vision_utils::IMG_DIR() + "pz/pz09.jpg"); }
TEST(TestSuite, pz10) { test_success(vision_utils::IMG_DIR() + "pz/pz10.jpg"); }
TEST(TestSuite, pz11) { test_success(vision_utils::IMG_DIR() + "pz/pz11.jpg"); }
TEST(TestSuite, pz12) { test_success(vision_utils::IMG_DIR() + "pz/pz12.jpg"); }
TEST(TestSuite, pz40) { test_success(vision_utils::IMG_DIR() + "pz/pz40.jpg"); }
TEST(TestSuite, tictactoe_frame) { test_success(vision_utils::IMG_DIR() + "tictactoe/sample_frame.jpg"); }
// fails
TEST(TestSuite, pz39) { test_fail(cv::imread(vision_utils::IMG_DIR() + "pz/pz39.jpg"),
                                  PlayzoneFind::FAILURE_NO_CORNERS_FOUND); }

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
