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

Implementation of the kid's game "red light, green light" for a
social robot with a kinect camera.
 */
#include "vision_utils/mask_acceleration.h"
#include "vision_utils/linear_assign.h"
#include "vision_utils/iterable_to_string.h"
#include "vision_utils/centroid.h"
#include "vision_utils/cmatrix.h"

class RedLightGreenLight {
public:
  typedef std::string PlayerName;
  typedef cv::Mat1b PlayerMask;
  typedef float PlayerDistance;

  static const unsigned int TIME_WATCHING_WALL_SECONDS = 6;
  static const unsigned int TIME_WATCHING_PLAYERS_SECONDS = 6;
  static const unsigned int TIME_PLAYERS2START_ORDER_TIMEOUT_SECONDS = 3;

  /*! if the norm of a MaskAcceleration::PixelAcceleration is above this value,
   * the player is considered to have moved. */
  static const double DEFAULT_ACC_THRESHOLD_FOR_MOTION = 30;
  /*! if the distance of a player (in meters) is above this threshold from one frame to another,
   * the player is considered to have moved. */
  static const double DEFAULT_DIST_THRESHOLD_FOR_MOTION = .2;
  /*! if a player comes closer from the camera than this distance (in meters),
   * the player is considered to have moved. */
  static const double DEFAULT_DIST_THRESHOLD_FOR_WINNING = 1;
  /*! distance of the the start line from the robot, in meters. */
  static const double DEFAULT_START_LINE_DIST = 8;
  //! the different statuses that can have a game
  enum GameStatus {
    GAME_STOPPED = 0,
    GAME_WATCHING_WALL = 1,
    GAME_WATCHING_PLAYERS = 2,
    GAME_WON_BY_PLAYER = 3
  };
  static const inline std::string game_status2string(GameStatus s) {
    switch (s) {
      case GAME_WATCHING_WALL: return "WATCHING_WALL";
      case GAME_WATCHING_PLAYERS: return "WATCHING_PLAYERS";
      case GAME_WON_BY_PLAYER: return "WON_BY_PLAYER";
      default:
      case GAME_STOPPED: return "STOPPED";
    }
  } // end game_status2string()

  //! the status of each player during a game
  enum PlayerStatus {
    PLAYER_UNKNWON = 0, // default value in set
    PLAYER_MUST_GO2START = 1,
    PLAYER_OK_IN_FIELD = 2,
    PLAYER_OK_BEYOND_START = 3,
    PLAYER_HAS_WON = 4,
    PLAYER_HAS_LOST = 5,
    PLAYER_COMPUTATION_ERROR = -1,
    PLAYER_LOST = -2,
  };
  static const inline std::string player_status2string(PlayerStatus s) {
    switch (s) {
      case PLAYER_MUST_GO2START: return "MUST_GO2START";
      case PLAYER_OK_IN_FIELD: return "OK_IN_FIELD";
      case PLAYER_OK_BEYOND_START: return "OK_BEYOND_START";
      case PLAYER_HAS_WON: return "HAS_WON";
      case PLAYER_HAS_LOST: return "HAS_LOST";
      case PLAYER_COMPUTATION_ERROR: return "COMPUTATION_ERROR";
      default:
      case PLAYER_UNKNWON: return "UNKNWON";
    }
  } // end player_status2string()
  //! the infos stored for each player
  struct Player {
    Player() : status(PLAYER_UNKNWON), prev_distance(-1), distance(-1) {}
    PlayerName name_at_beginning;
    PlayerName name_in_last_frame; //! for ID swaps
    PlayerStatus status;
    PlayerDistance prev_distance, distance;
    vision_utils::MaskAcceleration accel;
    vision_utils::Timer last_seen_timer;
    vision_utils::Timer last_go2start_timer;
    cv::Mat3b display_image;
    inline friend std::ostream& operator<<(std::ostream& s, const Player& p) {
      s << p.name_at_beginning
        << " (last seen " << p.last_seen_timer.getTimeSeconds()
        << " s ago with name " << p.name_in_last_frame
        << "), status:" << p.status;
      return s;
    }
  };

  //////////////////////////////////////////////////////////////////////////////

  //! ctor
  RedLightGreenLight() {
    // stop_game();
    _game_status = GAME_STOPPED;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void start_game(const std::vector<PlayerName> & player_names) {
    printf("start_game(%s)\n",
           vision_utils::iterable_to_string(player_names).c_str());
    // set PLAYER_MUST_GO2START for everyone?
    say_text("en:I am ready to play. Do you want to start?"
             "|es:Estoy listo para jugar. Quereis empezar?");
    // create Player vector
    unsigned int nplayers = player_names.size();
    _players.clear();
    _players.reserve(nplayers);
    for (unsigned int i = 0; i < nplayers; ++i) {
      Player p;
      p.name_at_beginning = player_names[i];
      p.name_in_last_frame = player_names[i];
      p.status = PLAYER_MUST_GO2START;
      _players.push_back(p);
    } // end for p
    // send players behind start and change status to GAME_WATCHING_PLAYERS
    _game_status = GAME_WATCHING_PLAYERS;
    check_all_players_behind_start(player_names);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! switch the state of the game to stopped
  inline void stop_game() {
    say_text("|en:OK, let's stop this game.|en:Game over."
             "|es:Ya dejamos el escondite ingles.|es:Fin de partido.");
    _game_status = GAME_STOPPED;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \return the number of players that were registered at the beginning of the game
  inline unsigned int nb_players() const { return _players.size(); }

  //////////////////////////////////////////////////////////////////////////////

  //! \return the current status of the game
  inline GameStatus get_game_status() const { return _game_status; }

  //////////////////////////////////////////////////////////////////////////////

  //! \return all the info about the players that were registered at the beginning of the game
  inline const std::vector<Player> & get_players() const {
    return _players;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline PlayerStatus get_player_status(const std::string & player_name_in_last_frame) {
    Player* player = get_player(player_name_in_last_frame);
    if (!player)
      return PLAYER_UNKNWON;
    return player->status;
  }

  //////////////////////////////////////////////////////////////////////////////

  Player* get_player(const std::string & player_name_in_last_frame) {
    unsigned int nplayers = nb_players();
    for (unsigned int i = 0; i < nplayers; ++i) {
      if (_players[i].name_in_last_frame == player_name_in_last_frame)
        return &(_players[i]);
    } // end for var
    printf("RedLightGreenLight: unknown player '%s'!\n", player_name_in_last_frame.c_str());
    return NULL;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool update(const std::vector<PlayerName> & player_names,
              const std::vector<PlayerDistance> & player_distances,
              const std::vector<PlayerMask> & player_masks,
              const std::vector<cv::Point> & masks_offsets,
              const double dist_threshold_for_motion = DEFAULT_DIST_THRESHOLD_FOR_MOTION,
              const double acc_threshold_for_motion = DEFAULT_ACC_THRESHOLD_FOR_MOTION) {
    printf("update(%s, status:'%s')\n",
           vision_utils::iterable_to_string(player_names).c_str(),
           game_status2string(_game_status).c_str());
    unsigned int ncurr_players = player_names.size();
    if (player_distances.size() != ncurr_players
        || player_masks.size() != ncurr_players
        || masks_offsets.size() != ncurr_players) {
      printf("RedLightGreenLight::set_player_masks(): corrupted data! "
             "%li names, %li distances, %li masks, %li offsets\n",
             player_names.size(), player_distances.size(),
             player_masks.size(), masks_offsets.size());
      return false;
    }
    if (!solve_id_swaps(player_names, player_masks, masks_offsets))
      return false;
    // update distances
    for (unsigned int i = 0; i < ncurr_players; ++i) {
      Player* player = get_player(player_names[i]);
      if (!player)
        continue;
      player->prev_distance = player->distance;
      player->distance = player_distances[i];
    } // end for i

    if (_game_status == GAME_WATCHING_WALL) {
      set_all_seen_player_to(PLAYER_OK_IN_FIELD, player_names);
      // check state changing GAME_WATCHING_WALL -> GAME_WATCHING_PLAYERS
      if (_green_light_timer.getTimeSeconds() >= TIME_WATCHING_WALL_SECONDS) {
        start_looking_players();
        return update(player_names, player_distances, player_masks, masks_offsets); // re-run the cb
      }
      check_players_win(player_names);
      return true;
    } // end GAME_LOOKING_WALL

    if (_game_status == GAME_WATCHING_PLAYERS) {
      // check if any of the players have won or moved
      for (unsigned int i=0; i< ncurr_players; i++) {
        check_player_still
            (player_names[i], player_masks[i], masks_offsets[i],
             dist_threshold_for_motion, acc_threshold_for_motion);
      }
      check_players_win(player_names);
      // check state changing GAME_WATCHING_PLAYERS -> GAME_WATCHING_WALL
      if (!check_all_players_behind_start(player_names))
        return true;
      // check state changing GAME_WATCHING_PLAYERS -> GAME_WATCHING_WALL
      if (_red_light_timer.getTimeSeconds() >= TIME_WATCHING_PLAYERS_SECONDS) {
        start_looking_wall();
        return update(player_names, player_distances, player_masks, masks_offsets); // re-run the cb
      }
      return true;
    } // end GAME_LOOKING_PLAYERS
    return true;
  } // end update()

  //////////////////////////////////////////////////////////////////////////////

  //! Start looking player
  virtual inline void start_looking_players(){
    printf("RedLightGreenLight: RED LIGHT! Looking at players, be still!\n");
    say_text("en:Red light!|es:Me doy la vuelta.");
    _game_status = GAME_WATCHING_PLAYERS;
    _red_light_timer.reset();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! Start looking wall
  virtual inline void start_looking_wall(){
    printf("RedLightGreenLight: GREEN LIGHT! Looking at wall, players can move.\n");
    say_text("en:Green light!"
             "|es:Un, dos, tres, escondite inglés. Sin mover las manos ni los pies.");
    _game_status = GAME_WATCHING_WALL;
    _green_light_timer.reset();
    // forget about users previous distances and accelerations
    for (unsigned int i = 0; i < nb_players(); ++i) {
      _players[i].accel = vision_utils::MaskAcceleration();
      _players[i].prev_distance = _players[i].distance = -1;
    }
  } // end start_looking_wall()

  //////////////////////////////////////////////////////////////////////////////

  void display(const std::vector<PlayerName> & player_names,
               const std::vector<PlayerMask> & player_masks) {
    unsigned int nplayers = std::min(player_names.size(), player_masks.size());
    for (unsigned int player_idx = 0; player_idx < nplayers; ++player_idx) {
      std::string player_name = player_names[player_idx];
      Player* player = get_player(player_names[player_idx]);
      if (!player)
        continue;

      // add color border according to status
      PlayerStatus status = player->status;
      cv::Scalar bg_color = cv::Scalar::all(255);
      switch (status) {
        case PLAYER_OK_IN_FIELD:
          bg_color = CV_RGB(0, 100, 0); break;
        case PLAYER_MUST_GO2START:
          bg_color = CV_RGB(100, 0, 0); break;
        case PLAYER_HAS_WON:
          bg_color = CV_RGB(100, 100, 0); break; // yellow
        default:
        case PLAYER_COMPUTATION_ERROR:
          bg_color = CV_RGB(100, 128, 0); break;// orange
      }
      // draw illus image for the MaskAcceleration
      cv::Mat3b* display_image = &(player->display_image);
      display_image->create(player_masks[player_idx].size());
      display_image->setTo(bg_color);
      vision_utils::MaskAcceleration* player_acc = &(player->accel);
      player_acc->draw_illus(*display_image,
                             vision_utils::MaskAcceleration::DEFAULT_ACC_DRAWING_SCALE,
                             bg_color);
      printf("Player '%s', mask size:%ix%i, display size:%ix%i\n",
             player_name.c_str(),
             player_masks[player_idx].cols, player_masks[player_idx].rows,
             display_image->cols, display_image->rows);
      // show image
      cv::imshow(player_name, *display_image);
    } // end for player_idx
    cv::waitKey(5);
  } // end display();

protected:
  //////////////////////////////////////////////////////////////////////////////

  bool solve_id_swaps(const std::vector<PlayerName> & player_names,
                      const std::vector<PlayerMask> & player_masks,
                      const std::vector<cv::Point> & masks_offsets) {
    std::vector<PlayerName> prev_names;
    std::vector<cv::Point> prev_centers, curr_centers;
    unsigned int prev_nplayers = nb_players(), curr_nplayers = player_names.size();
    for (unsigned int i = 0; i < prev_nplayers; ++i) {
      prev_names.push_back(_players[i].name_in_last_frame);
      prev_centers.push_back(_players[i].accel.get_centroid());
    } // end for i
    for (unsigned int i = 0; i < curr_nplayers; ++i) {
      curr_centers.push_back(vision_utils::centroidOfMonochromeImage(player_masks[i])
                             + masks_offsets[i]);
    } // end for i
    // generate cost matrix
    vision_utils::CMatrix<vision_utils::Cost> costs(prev_nplayers, curr_nplayers);
    costs.set_to_zero();
    for (unsigned int prev_i = 0; prev_i < prev_nplayers; ++prev_i) {
      for (unsigned int curr_i = 0; curr_i < curr_nplayers; ++curr_i) {
        if (prev_names[prev_i] != player_names[curr_i])
          costs[prev_i][curr_i] = 1 + vision_utils::distance_points
                                  (prev_centers[prev_i], curr_centers[curr_i]);
      } // end for curr_i
    } // end for prev_i
    vision_utils::MatchList best_assign;
    vision_utils::Cost best_cost;
    if (!vision_utils::linear_assign(costs, best_assign, best_cost))
      return false;
    return true;
  } // end solve_id_swaps()

  //////////////////////////////////////////////////////////////////////////////

  //! \return true if player has moved in the last frame
  bool check_player_still(const std::string & player_name,
                          const PlayerMask & player_mask,
                          const cv::Point & masks_offset,
                          const double dist_threshold_for_motion = DEFAULT_DIST_THRESHOLD_FOR_MOTION,
                          const double acc_threshold_for_motion = DEFAULT_ACC_THRESHOLD_FOR_MOTION) {
    Player* player = get_player(player_name);
    if (!player)
      return false;

    // check if player lost
    static const unsigned int PLAYER_LOST_TIMEOUT = 5; // seconds
    if (player->status == PLAYER_LOST) {
      return false;
    } // end if (player->status = PLAYER_LOST)
    else if (player->last_seen_timer.getTimeSeconds() > PLAYER_LOST_TIMEOUT) {
      printf("RedLightGreenLightSkill: Lost player '%s'\n", player_name.c_str());
      player->status = PLAYER_LOST;
      std::ostringstream sentence;
      sentence << "en:" << player_name << ", where are you?";
      sentence << "|es:" << player_name << ", donde estas?";
      say_text(sentence.str());
      return false;
    }

    if (player->status == PLAYER_MUST_GO2START) {
      return false;
    } // end if (player->status = PLAYER_MUST_GO2START)

    if (player->status == PLAYER_OK_BEYOND_START) {
      return false;
    } // end if (player->status = PLAYER_BEYOND_START)

    bool moved_in_last_frame = false;
    // check distance
    if (player->prev_distance > 0
        && fabs(player->prev_distance - player->distance) > dist_threshold_for_motion) {
      moved_in_last_frame = true;
      printf("RedLightGreenLight: player '%s' has moved (distance change of %f)!\n",
             player->name_at_beginning.c_str(), fabs(player->prev_distance - player->distance));
    }

    if (!moved_in_last_frame) { // check PixelAcceleration
      // convert masks into accelerations
      if (!player->accel.from_mask(player_mask, masks_offset)) {
        printf("RedLightGreenLightSkill: Problem with player '%s'\n", player_name.c_str());
        player->status = PLAYER_COMPUTATION_ERROR;
        return false;
      }
      const vision_utils::MaskAcceleration* player_acc = &(player->accel);
      std::vector<vision_utils::MaskAcceleration::PixelAcceleration> accs
          = player_acc->get_pixel_accelerations();
      unsigned int naccs = accs.size();
      for (unsigned int acc_idx = 0; acc_idx < naccs; ++acc_idx) {
        vision_utils::MaskAcceleration::PixelAcceleration* acc = &(accs[acc_idx]);
        if (acc->norm <= acc_threshold_for_motion) // no motion
          continue;
        moved_in_last_frame = true;
        printf("RedLightGreenLight: player '%s' has moved (mask acceleration of %f)!\n",
               player->name_at_beginning.c_str(), acc->norm);
        break;
      } // end for acc_idx
    } // end if (!still)

    if (moved_in_last_frame) { // return if has not moved
      player->status = PLAYER_MUST_GO2START;
      // speak: send player to start line
      printf("RedLightGreenLight: Player '%s' has moved!\n", player_name.c_str());
      vision_utils::Timer* last_go2start_timer = &(player->last_go2start_timer);
      if (last_go2start_timer->getTimeSeconds() < TIME_PLAYERS2START_ORDER_TIMEOUT_SECONDS) // shut up
        return false;
      last_go2start_timer->reset();
      std::ostringstream sentence;
      sentence << "|en:" << player_name
               << ", i've caught you! Please go back to the starting line";
      sentence << "|es:" << player_name
               << ", te he pillado! Tienes que volver a la salida.";
      say_text(sentence.str());
      return false;
    } // end if (moved_in_last_frame)

    player->status = PLAYER_OK_IN_FIELD;
    return true;
  } // end check_player_still();

  //////////////////////////////////////////////////////////////////////////////

  //! check if one of the players has won
  inline bool check_players_win(const std::vector<PlayerName> & player_names) {
    // check if any of the players have won or moved
    for (unsigned int i=0; i< player_names.size(); i++) {
      bool has_won = check_player_win(player_names[i]);
      if (!has_won)
        continue;
      // set all other players having lost
      set_all_seen_player_to(PLAYER_HAS_LOST, player_names);
      get_player(player_names[i])->status = PLAYER_HAS_WON;
      // stop game
      stop_game();
    }
    return true;
  } // end check_players_win()

  //////////////////////////////////////////////////////////////////////////////

  //! \return true if player has won in the last frame
  bool check_player_win(const std::string & player_name,
                        const double dist_threshold_for_winning = DEFAULT_DIST_THRESHOLD_FOR_WINNING) {
    Player* player = get_player(player_name);
    if (!player)
      return false;
    if (player->status != PLAYER_OK_IN_FIELD)
      return false;
    if (player->distance > dist_threshold_for_winning)
      return false;
    player->status = PLAYER_HAS_WON;
    printf("RedLightGreenLight: Player '%s' has won!\n", player_name.c_str());
    std::ostringstream sentence;
    sentence << "|en:" << player_name << ", you have won!";
    sentence << "|es:" << player_name << ", has ganado!";
    say_text(sentence.str());
    return true;
  } // end check_player_win();


  //////////////////////////////////////////////////////////////////////////////

  //! check if players that have moved are back at the start line
  inline bool check_all_players_behind_start(const std::vector<PlayerName> & player_names) {
    printf("check_all_players_behind_start(%s)\n",
           vision_utils::iterable_to_string(player_names).c_str());
    std::ostringstream names, sentence;
    unsigned int nplayers2send = 0, ncurr_players = player_names.size();
    for (unsigned int i = 0; i < ncurr_players; ++i) {
      Player* player = get_player(player_names[i]);
      if (!player)
        continue;
      if (player->status != PLAYER_MUST_GO2START)
        continue;
      // check if distance far enough
      if (player->distance > DEFAULT_START_LINE_DIST) {
        player->status = PLAYER_OK_BEYOND_START;
        continue;
      }
      names << player->name_at_beginning << ", ";
      ++nplayers2send;
    } // end for i

    if (nplayers2send == 0) {
      _last_players2start_list = "";
      return true;
    }

    // say sentence
    if (_last_players2start_list != names.str().c_str()
        && _last_players2start_order_sent.getTimeSeconds() > TIME_PLAYERS2START_ORDER_TIMEOUT_SECONDS) {
      _last_players2start_order_sent.reset();
      bool everybody_punished = (nplayers2send == nb_players());
      sentence <<  "en:" << (everybody_punished ? "Everybody" : _last_players2start_list)
                << " to the start line!";
      sentence <<  "|es:" << (everybody_punished ? "Todo el mundo" : _last_players2start_list)
                << " detras de la linea!";
      say_text(sentence.str());
    }
    _last_players2start_list = names.str().c_str();
    printf("Players not at the start line:'%s'\n", _last_players2start_list.c_str());
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! set all players to a given status
  bool set_all_seen_player_to(PlayerStatus status,
                              const std::vector<PlayerName> & player_names) {
    unsigned int ncurr_players = player_names.size();
    for (unsigned int player_idx=0; player_idx< ncurr_players; player_idx++) {
      Player* player = get_player(player_names[player_idx]);
      if (!player)
        continue;
      player->status = status;
      player->last_seen_timer.reset();
    } // end for player_idx
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline virtual void say_text(const std::string & sentence) {
    printf("\n***\n'%s'\n***\n", sentence.c_str());
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std::vector<Player> _players;
  GameStatus _game_status;
  vision_utils::Timer _green_light_timer, _red_light_timer, _last_players2start_order_sent;
  std::string _last_players2start_list;
}; // end RedLightGreenLight
