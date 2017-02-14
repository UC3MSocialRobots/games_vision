#ifndef TICTACTOESOLVER_H_
#define TICTACTOESOLVER_H_

/***************************************************************************//**
 * \class TicTacToeSolver
 *
 * \brief a solver of tic tac toe (tres in raya) game
 *
 * Used by the skill TicTacToeSkill
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 09/06/2009
 *******************************************************************************/

///// STL imports
#include <stdio.h>
#include <stdlib.h>         // for srand
#include <string>           // for strings
#include <sstream>          // for sstreams
#include <iostream>         // for cin, cout
#include <iomanip>          // for setw : leading zeros
#include <deque>            // for double ended queues
#include <vector>           // for vectors

class TicTacToeSolver {
public:
  enum CellContent {
    UNKNOWN = -1,
    ROUND = 0,
    CROSS = 1,
    EMPTY = 2
  };
  typedef std::vector<CellContent> Board;
  enum AnswerType {
    ANSWER_RUNNING = 20,
    ANSWER_MAGGIE_WON = 30,
    ANSWER_PLAYER_WON = 40,
    ANSWER_DRAW = 50,
    ANSWER_TOO_MANY_CROSSES = 100,
    ANSWER_TOO_MANY_ROUNDS = 110,
    ANSWER_IT_IS_PLAYER_TURN = 120
  };
  enum Reason {
    REASON_TO_WIN = 1,
    REASON_TO_BLOCK = 2,
    REASON_RANDOM = 3,
    REASON_EXPERT = 4
  };
  enum Level {
    NOVICE = 1,
    INTERMEDIATE = 2,
    ADVANCED = 3,
    EXPERT = 4
  };

  typedef short Coord;
  typedef std::pair<Coord, Coord> CoordPair; // col, row

  static const unsigned int NB_COLS = 3, NB_ROWS = 3;

  TicTacToeSolver();
  virtual ~TicTacToeSolver();

  Board board;
  void get_nb_of_pions(int & maggie, int & player);

  void set_board(const Board &newBoard);
  void set_board_string(const std::string & newBoard);

  void set_Maggie_level(Level level);
  Level maggie_level;
  std::string get_Maggie_level_sentence() const;

  void set_Maggie_pions(const CellContent ptype);
  inline CellContent get_Maggie_pions() const { return maggie_pions; }
  void set_player_pions(const CellContent ptype);
  inline CellContent get_player_pions() const { return player_pions; }
  CellContent maggie_pions;
  CellContent player_pions;
  std::string get_Maggie_pions_sentence() const;

  CoordPair find_next_move();
  AnswerType type_of_answer;
  CoordPair answer_point;
  void get_answer(AnswerType & type, CoordPair & ans) const;
  int reason_of_answer;

  void display_grid();
  void check_game_integrity();
  void find_winner();

  void compute_random_move();
  void look_for_winning_move();
  void block_losing_move();
  void look_for_moves_expert();
  bool has_played;

  void affect(Coord col, Coord row);
  bool test_match(const std::string &situation);
  bool test_match_vector(const std::vector<std::string> & situations);
  bool test_match_then_affect(const std::string & situation, Coord col, Coord row);

  //////
  ////// static functions
  //////
  static const std::string board2string(const Board & board) {
    std::ostringstream out;
    Board::const_iterator c = board.begin();
    for (unsigned int row = 0; row < NB_ROWS; ++row) {
      for (unsigned int col = 0; col < NB_COLS; ++col) {
        if (*c==CROSS)
          out << 'X';
        else if (*c==ROUND)
          out << 'O';
        else if (*c==EMPTY)
          out << '-';
        else if (*c==UNKNOWN)
          out << '?';
        else
          out << "*";
        ++c;
      } // end for (col)
    } // end for (row)
    return out.str();
  }
};

#endif /*TICTACTOESOLVER_H_*/

