#ifndef TICTACTOE_SKILL_H
#define TICTACTOE_SKILL_H

/***************************************************************************//**
 * \class TicTacToeSkill
 *
 * \brief A skill to play to the game tic-tac-toe with the camera
 *
 * It uses the skill PlayzoneFindSkill
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 26/05/2009
 *******************************************************************************/

///// my imports
#include "games_vision/playzone_sequential_user.h"
#include "games_vision/tic_tac_toe_solver.h"
#include "vision_utils/image_comparer.h"

#define TICTACTOE_DIR  ros::package::getPath("games_vision") + "/data/tictactoe/"

class TicTacToeSkill: public PlayzoneSequentialUser {
public:
  static const double DISTANCE_MAX_BEFORE_D2 = 1.5;
  static const double DISTANCE_MAX_AFTER_D2 = 2.0;
  static const double EMPTYNESS_THRESHOLD = .92f;

  /** constructor */
  TicTacToeSkill();
  /** destructor */
  ~TicTacToeSkill();

  inline std::string get_board_as_string() const {
    if (get_status() == SUCCESS_FOUND_AND_PROCESSED)
      return TicTacToeSolver::board2string(board);
    else return "processing failed!";
  }
  inline void set_Maggie_pions(const TicTacToeSolver::CellContent ptype) {
    _solver.set_Maggie_pions(ptype);
  }
  inline TicTacToeSolver::AnswerType get_game_status() const {
    TicTacToeSolver::AnswerType type;
    TicTacToeSolver::CoordPair ans;
    _solver.get_answer(type, ans);
    return type;
  }

protected:
  /////
  ///// general functions
  /////
  /** main loop */
  bool process_pz(const cv::Mat3b & pz);
  /*! the function that will be called before trying to get the playzone */
  void do_stuff_before_get_playzone();
  /*! the function that will be called after trying to get the playzone */
  void do_stuff_after_get_playzone(bool was_find_and_process_success);

  virtual void first_move_hook(bool has_player_started);

  /** create the required subscribers to the playzone, etc. */
  virtual void create_subscribers_and_publishers_playzone();

  /** shutdown the required subscribers to the playzone, etc. */
  virtual void shutdown_subscribers_and_publishers_playzone();


private:

  /////
  ///// general fields
  /////
  //! the monochrome version of the acquired frame
  cv::Mat1b _playzone_monochrome;

  /////
  ///// parameters
  /////
  ///// playzone image parameters
  static const double MARGIN_RATE_HOR = .03;
  static const double MARGIN_RATE_VER = .03;
  int coord_left_cell(const cv::Size & pzs, const TicTacToeSolver::Coord col);
  int coord_up_cell(const cv::Size & pzs, const TicTacToeSolver::Coord row);
  double cell_width(const cv::Size & pzs);
  double cell_height(const cv::Size & pzs);
  double left_margin(const cv::Size & pzs);
  double up_margin(const cv::Size & pzs);

  /////
  ///// voice
  /////
  /** the results */
  void sayResult(int method, double t);

  /////
  ///// comparer
  /////
  vision_utils::ImageComparer comparer;

  /////
  ///// game
  /////
  TicTacToeSolver::Board board;
  TicTacToeSolver _solver;
  void illustrate_next_move(cv::Mat & pz_illus,
                            const TicTacToeSolver::Coord col,
                            const TicTacToeSolver::Coord row,
                            const TicTacToeSolver::CellContent new_move);

  /*!
     * \brief   a routine to analyze the results of the tic tac toe solver
     */
  void analyze_next_move();
};

#endif

