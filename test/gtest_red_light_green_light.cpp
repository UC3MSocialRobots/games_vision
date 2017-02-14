/*!
  \file        gtest_red_light_green_light.cpp
  \author      Irene Pérez, Arnaud Ramey
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/13

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

  Simple tests for assessing the comparison of contours
 */
//#define DISPLAY

// Bring in gtest
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include "games_vision/red_light_green_light.h"
#include "vision_utils/images2ppl.h"
#include <vision_utils/img_path.h>
#define ASSERT_TRUE_TIMEOUT(cond, timeout) { vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)
#define DISTWIN RedLightGreenLight::DEFAULT_DIST_THRESHOLD_FOR_WINNING / 2.
#define DISTSTART RedLightGreenLight::DEFAULT_START_LINE_DIST + 1
#define DISTMID (RedLightGreenLight::DEFAULT_START_LINE_DIST * .5 + RedLightGreenLight::DEFAULT_DIST_THRESHOLD_FOR_WINNING * .5)

#define ASSERT_GAME_STATUS(game, exp_status) { \
  RedLightGreenLight::GameStatus status = game.get_game_status(); \
  ASSERT_TRUE(status == exp_status) << "Game status:" << RedLightGreenLight::game_status2string(status) << ", expected:" << RedLightGreenLight::game_status2string(exp_status); \
  }

#define ASSERT_PLAYER_STATUS(game, player_name_in_last_frame, exp_status) { \
  RedLightGreenLight::PlayerStatus status = game.get_player_status(player_name_in_last_frame); \
  ASSERT_TRUE(status == exp_status) << "Status player '" << player_name_in_last_frame << "':" << RedLightGreenLight::player_status2string(status) << ", expected:" << RedLightGreenLight::player_status2string(exp_status); \
  }

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_rlgl) {
  RedLightGreenLight game;
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_STOPPED);
  std::vector<RedLightGreenLight::PlayerName> player_names;
  player_names.push_back("Alice");
  game.start_game(player_names);
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);
  game.start_looking_players();
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);
  game.start_looking_wall();
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_WALL);
  game.stop_game();
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_STOPPED);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_rlgl_timeout) {
  RedLightGreenLight game;
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_STOPPED);
  std::vector<RedLightGreenLight::PlayerName> player_names;
  std::vector<RedLightGreenLight::PlayerDistance> distances;
  std::vector<RedLightGreenLight::PlayerMask> player_masks;
  std::vector<cv::Point> masks_offsets;
  game.start_game(player_names);
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);
  // wait for players to go to start -> wall
  ASSERT_TRUE_TIMEOUT(game.update(player_names, distances, player_masks, masks_offsets)
                      && game.get_game_status() == RedLightGreenLight::GAME_WATCHING_WALL,
                      RedLightGreenLight::TIME_WATCHING_PLAYERS_SECONDS + 1);
  // wall -> players
  ASSERT_TRUE_TIMEOUT(game.update(player_names, distances, player_masks, masks_offsets)
                      && game.get_game_status() == RedLightGreenLight::GAME_WATCHING_PLAYERS,
                      RedLightGreenLight::TIME_WATCHING_WALL_SECONDS + 1);
  // players -> wall
  ASSERT_TRUE_TIMEOUT(game.update(player_names, distances, player_masks, masks_offsets)
                      && game.get_game_status() == RedLightGreenLight::GAME_WATCHING_WALL,
                      RedLightGreenLight::TIME_WATCHING_PLAYERS_SECONDS + 1);
  game.stop_game();
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_STOPPED);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_game) {
  RedLightGreenLight game;
  std::vector<RedLightGreenLight::PlayerName> player_names;
  game.start_game(player_names);
  game.start_looking_players();
  std::vector<RedLightGreenLight::PlayerMask> player_masks;
  std::vector<RedLightGreenLight::PlayerDistance> distances;
  std::vector<cv::Point> masks_offsets;
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
  //  assert_true_tim
  ASSERT_TRUE(game.get_players().empty());
  game.stop_game();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, corrupted_data) {
  std::vector<RedLightGreenLight::PlayerName> player_names;
  std::vector<RedLightGreenLight::PlayerDistance> distances;
  std::vector<RedLightGreenLight::PlayerMask> player_masks;
  std::vector<cv::Point> masks_offsets;
  unsigned int nplayers = 2;
  player_names.push_back("Alice");
  player_names.push_back("Bobby");
  RedLightGreenLight game;
  game.start_game(player_names);
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_MUST_GO2START);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_MUST_GO2START);

  // check sizes mismatches (player_masks empty)
  ASSERT_FALSE(game.update(player_names, distances, player_masks, masks_offsets));
  unsigned   int circle_radius = 15;
  cv::Mat1b mask(100, 100, (uchar) 0);
  cv::Point circle_center(50, 50);
  cv::circle(mask, circle_center, circle_radius, CV_RGB(255, 255, 255), -1);
  player_masks.push_back(mask); // for "Alice"
  player_masks.push_back(mask.clone()); // for "Bobby"
  // check sizes mismatches (masks_offsets empty)
  ASSERT_FALSE(game.update(player_names, distances, player_masks, masks_offsets));
  masks_offsets.push_back(cv::Point(0,0));
  masks_offsets.push_back(cv::Point(0,0));
  // check sizes mismatches (distances empty)
  ASSERT_FALSE(game.update(player_names, distances, player_masks, masks_offsets));

  // give distances => this time it should work
  distances.push_back(DISTSTART);
  distances.push_back(DISTSTART);
  std::vector<RedLightGreenLight::Player> players;
  for (unsigned int var = 0; var < 10; ++var) {
    ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
    players = game.get_players();
    ASSERT_TRUE(players.size() == nplayers) << "players:" << vision_utils::iterable_to_string(players);
    ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_OK_BEYOND_START);
    ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_OK_BEYOND_START);
  } // end for var

  // move players in field
  game.start_looking_wall();
  distances[0] = DISTMID;
  distances[1] = DISTMID;
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
  game.start_looking_players();
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_OK_IN_FIELD);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_OK_IN_FIELD);

  // now give some corrupted data for "Bobby"
  player_masks.back().setTo(0);
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets, 10));
  players = game.get_players();
  ASSERT_TRUE(players.size() == nplayers) << "players:" << vision_utils::iterable_to_string(players);
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_OK_IN_FIELD);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_COMPUTATION_ERROR);

  // again, with no image for "Alice"
  player_masks.front().release();
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets, 10));
  players = game.get_players();
  ASSERT_TRUE(players.size() == nplayers) << "players:" << vision_utils::iterable_to_string(players);
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_COMPUTATION_ERROR);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_COMPUTATION_ERROR);
  game.stop_game();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, simple_game) {
  // prepair data
  //unsigned int nplayers = 2;
  std::vector<RedLightGreenLight::PlayerName> player_names;
  player_names.push_back("Alice");
  player_names.push_back("Bobby");
  std::vector<RedLightGreenLight::PlayerDistance> distances(2, DISTMID);
  std::vector<RedLightGreenLight::PlayerMask> player_masks;
  unsigned int circle_radius = 15;
  cv::Mat1b mask(100, 100, (uchar) 0);
  cv::Point circle_center(50, 50);
  cv::circle(mask, circle_center, circle_radius, CV_RGB(255, 255, 255), -1);
  player_masks.push_back(mask); // for "Alice"
  player_masks.push_back(mask.clone()); // for "Bobby"
  std::vector<cv::Point> masks_offsets(2, cv::Point(0,0));
  // start a game - everybody should go to start
  RedLightGreenLight game;
  game.start_game(player_names);
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_MUST_GO2START);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_MUST_GO2START);
  // nothing should change after this update
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_MUST_GO2START);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_MUST_GO2START);

  // move user "Alice" behind starting line - he should be OK now
  distances[0] = DISTSTART;
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_OK_BEYOND_START);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_MUST_GO2START);

  // now move user "Bobby" behind starting line - both should be OK now
  distances[1] = DISTSTART;
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_OK_BEYOND_START);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_OK_BEYOND_START);

  // both users behind start -> the robot should now start to look at the wall
  distances[0] = DISTMID;
  distances[1] = DISTMID;
  ASSERT_TRUE_TIMEOUT(game.update(player_names, distances, player_masks, masks_offsets)
                      && game.get_game_status() == RedLightGreenLight::GAME_WATCHING_WALL,
                      RedLightGreenLight::TIME_WATCHING_PLAYERS_SECONDS + 1);

  // wait to look again at players
  ASSERT_TRUE_TIMEOUT(game.update(player_names, distances, player_masks, masks_offsets)
                      && game.get_game_status() == RedLightGreenLight::GAME_WATCHING_PLAYERS,
                      RedLightGreenLight::TIME_WATCHING_WALL_SECONDS + 1);
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_OK_IN_FIELD);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_OK_IN_FIELD);

  // move user "Alice" - should go to start
  player_masks.front().setTo(0);
  cv::circle(player_masks.front(), circle_center + cv::Point(30, 30), circle_radius, CV_RGB(255, 255, 255), -1);
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets, 10));
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_MUST_GO2START);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_OK_IN_FIELD);
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);

  // now move user "Bobby" - user "Alice" must still go to start
  player_masks.back().setTo(0);
  cv::circle(player_masks.back(), circle_center + cv::Point(30, 30), circle_radius, CV_RGB(255, 255, 255), -1);
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets, 10));
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_MUST_GO2START);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_MUST_GO2START);
  ASSERT_GAME_STATUS(game, RedLightGreenLight::GAME_WATCHING_PLAYERS);

  // move both users behind start
  distances[0] = DISTSTART;
  distances[1] = DISTSTART;
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_OK_BEYOND_START);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_OK_BEYOND_START);
  // everybody OK -> the robot should now start to look at the wall
  ASSERT_TRUE_TIMEOUT(game.update(player_names, distances, player_masks, masks_offsets)
                      && game.get_game_status() == RedLightGreenLight::GAME_WATCHING_WALL,
                      RedLightGreenLight::TIME_WATCHING_PLAYERS_SECONDS + 1);


  // while robot looking at the wall, bring user "Alice" closer
  distances[0] = DISTWIN;
  distances[1] = DISTSTART;
  // wait to look again at players
  ASSERT_TRUE_TIMEOUT(game.update(player_names, distances, player_masks, masks_offsets)
                      && game.get_game_status() == RedLightGreenLight::GAME_STOPPED,
                      RedLightGreenLight::TIME_WATCHING_WALL_SECONDS + 1);
  // user "Alice" should have won
  ASSERT_PLAYER_STATUS(game, "Alice", RedLightGreenLight::PLAYER_HAS_WON);
  ASSERT_PLAYER_STATUS(game, "Bobby", RedLightGreenLight::PLAYER_HAS_LOST);
  game.stop_game();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, simple_game_david_arnaud) {
  RedLightGreenLight game;
  std::vector<RedLightGreenLight::PlayerName> player_names;
  std::string DAVID = "1", ARNAUD = "2";
  player_names.push_back(DAVID);
  player_names.push_back(ARNAUD);
  game.start_game(player_names);
  game.start_looking_players();

  // read images
  vision_utils::Images2PPL images2ppl;
  std::vector<std::string> names;
  std::vector<cv::Mat1b> player_masks;
  std::vector<cv::Point> masks_offsets;
  std::vector<unsigned int> pp_indices;
  std::string filename_prefix = IMG_DIR "depth/david_arnaud1";
  ASSERT_TRUE(images2ppl.convert(filename_prefix));
  ASSERT_TRUE(vision_utils::vision_utils::convert(images2ppl.get_ppl(),
                                             NULL, NULL, &player_masks,
                                             &masks_offsets, &pp_indices));
  ASSERT_TRUE(vision_utils::vision_utils::indices2names(images2ppl.get_ppl(),
                                                   pp_indices, names));
  ASSERT_TRUE(names.size() == 2);

  // set players at start line
  std::vector<RedLightGreenLight::PlayerDistance> distances(2, DISTSTART);
  ASSERT_TRUE(game.update(names, distances, player_masks, masks_offsets));
  ASSERT_TRUE(game.get_players().size() == 2);
  ASSERT_PLAYER_STATUS(game, DAVID, RedLightGreenLight::PLAYER_OK_BEYOND_START);
  ASSERT_PLAYER_STATUS(game, ARNAUD, RedLightGreenLight::PLAYER_OK_BEYOND_START);

  // move players in field
  game.start_looking_wall();
  distances[0] = DISTMID;
  distances[1] = DISTMID;
  ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
  game.start_looking_players();
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(game.update(player_names, distances, player_masks, masks_offsets));
    ASSERT_TRUE(game.get_players().size() == 2);
    ASSERT_PLAYER_STATUS(game, DAVID, RedLightGreenLight::PLAYER_OK_IN_FIELD);
    ASSERT_PLAYER_STATUS(game, ARNAUD, RedLightGreenLight::PLAYER_OK_IN_FIELD);
  }

  // now use an image that is close
  filename_prefix = IMG_DIR "depth/david_arnaud3";
  ASSERT_TRUE(images2ppl.convert(filename_prefix));
  ASSERT_TRUE(vision_utils::vision_utils::convert(images2ppl.get_ppl(),
                                             NULL, NULL, &player_masks,
                                             &masks_offsets, &pp_indices));
  ASSERT_TRUE(vision_utils::vision_utils::indices2names(images2ppl.get_ppl(),
                                                   pp_indices, names));

  // player 1 should still go to start
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(game.update(names, distances, player_masks, masks_offsets));
    ASSERT_TRUE(game.get_players().size() == 2);
    ASSERT_PLAYER_STATUS(game, DAVID, RedLightGreenLight::PLAYER_MUST_GO2START);
    ASSERT_PLAYER_STATUS(game, ARNAUD, RedLightGreenLight::PLAYER_OK_IN_FIELD);
  }

#ifdef DISPLAY
  game.display(ppl_conv.names, ppl_conv.masks);
  cv::waitKey(0);
#endif // DISPLAY
  game.stop_game();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
