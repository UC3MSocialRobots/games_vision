/*!
 * \file TicTacToeSolver.cpp
 *
 * TODO descr
 *
 * \date 17/11/2010
 * \author Arnaud Ramey
 */

#include "games_vision/tic_tac_toe_solver.h"

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   ROS_INFO(__VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

/*!
 * \brief   constructor
 */
TicTacToeSolver::TicTacToeSolver() {
  type_of_answer = ANSWER_RUNNING;
  set_Maggie_level(ADVANCED);
  set_Maggie_pions(CROSS);
  // clear board
  board.resize(NB_COLS * NB_ROWS, EMPTY);

  // init random numbers
  srand(time(NULL));
}

/*!
 * \brief   destructor
 */
TicTacToeSolver::~TicTacToeSolver() {
}

/*!
 * \brief   computes the number of pions of Maggie and the player
 *
 * \param   maggie the number of pions of maggie
 * \param   player the number of pions of the player
 */
void TicTacToeSolver::get_nb_of_pions(int & maggie, int & player) {
  Board::const_iterator curr_case = board.begin();
  maggie = 0;
  player = 0;

  for (int i = 0; i < 9; ++i) {
    if (*curr_case == maggie_pions)
      ++maggie;
    else if (*curr_case == player_pions)
      ++player;
    ++curr_case;
  }
}

/*!
 * \brief   change the data of the board
 */
void TicTacToeSolver::set_board(const Board & newBoard) {
  DEBUG_PRINT("TicTacToeSolver:set_board(size:%i)\n", newBoard.size());
  if (newBoard.size() != board.size()) {
    printf("TicTacToeSolver:set_board():incorrect size %li\n", newBoard.size());
    return;
  }
  for (unsigned int i = 0; i < board.size(); ++i)
    board[i] = newBoard[i];
}

/*!
 * \brief   change the data of the board
 */
void TicTacToeSolver::set_board_string(const std::string & newBoard) {
  DEBUG_PRINT("TicTacToeSolver:set_board_string()\n");
  if (newBoard.size() != board.size()) {
    printf("TicTacToeSolver:set_board_string('%s'):incorrect size %i\n",
           newBoard.c_str(), newBoard.size());
    return;
  }

  for (unsigned int i = 0; i < board.size(); ++i) {
    if (newBoard[i] == 'X')
      board[i] = CROSS;
    else if (newBoard[i] == 'O')
      board[i] = ROUND;
    else
      board[i] = EMPTY;
  }
}

/*!
 * \brief   change the playing level of Maggie
 *
 * \param   level a int among NOVICE INTERMEDIATE EXPERT ADVANCED
 * \return  a string to say it !
 */
void TicTacToeSolver::set_Maggie_level(TicTacToeSolver::Level level) {
  DEBUG_PRINT("TicTacToeSolver:set_Maggie_level(%i)\n", level);
  if (level != NOVICE && level != INTERMEDIATE && level
      != EXPERT && level != ADVANCED)
    return;
  maggie_level = level;
}

std::string TicTacToeSolver::get_Maggie_level_sentence() const {
  //printf("get_Maggie_level_sentence()");
  std::ostringstream sentence_es, sentence_en;
  sentence_es << "|es:Mi nivel esta en ";
  sentence_en << "|en:My level is ";
  if (maggie_level == NOVICE) {
    sentence_es << "principiante.";
    sentence_en << "easy.";
  }
  if (maggie_level == INTERMEDIATE) {
    sentence_es << "intermedio.";
    sentence_en << "medium.";
  }
  if (maggie_level == ADVANCED) {
    sentence_es << "avanzado.";
    sentence_en << "hard.";
  }
  if (maggie_level == NOVICE) {
    sentence_es << "experto.";
    sentence_en << "expert.";
  }
  std::ostringstream sentence;
  sentence << sentence_es.str() << sentence_en.str();
  return sentence.str();
}

std::string TicTacToeSolver::get_Maggie_pions_sentence() const {
  //printf("get_Maggie_pions_sentence()");
  if (maggie_pions == TicTacToeSolver::CROSS) {
    return "|es:Yo juego con cruces."
        "|en:I play with crosses.";
  } else {
    return "|es:Yo juego con circulos."
        "|en:I play with rounds.";
  }
}

/*!
 * \brief   change the pions of Maggie
 */
void TicTacToeSolver::set_Maggie_pions(const CellContent ptype) {
  DEBUG_PRINT("TicTacToeSolver:set_Maggie_pions(%i)\n", ptype);
  if (ptype != CROSS && ptype != ROUND)
    return;
  maggie_pions = ptype;
  player_pions = (ptype == ROUND ? CROSS : ROUND);
}

/*!
 * \brief   change the pions of the player
 */
void TicTacToeSolver::set_player_pions(const CellContent ptype) {
  DEBUG_PRINT("TicTacToeSolver:set_player_pions(%i)\n", ptype);
  if (ptype != CROSS || ptype != ROUND)
    return;
  player_pions = ptype;
  maggie_pions = (ptype == ROUND ? CROSS : ROUND);
}
/*!
 * \brief   display the grid in the console
 */
void TicTacToeSolver::display_grid() {
  DEBUG_PRINT("TicTacToeSolver:display_grid()\n");

  std::ostringstream m;
  m << std::endl;
  m << "  i 0 1 2" << std::endl;
  m << "j ---------" << std::endl;

  Board::const_iterator curr_case = board.begin();
  for (unsigned int row = 0; row < NB_ROWS; ++row) {
    m << row << " | ";
    for (unsigned int col = 0; col < NB_COLS; ++col) {
      if (*curr_case == UNKNOWN)
        m << "? ";
      if (*curr_case == EMPTY)
        m << ". ";
      if (*curr_case == ROUND)
        m << "O ";
      if (*curr_case == CROSS)
        m << "X ";
      ++curr_case;
    }
    m << "|" << std::endl;
  }
  m << "  ---------";
  printf("TicTacToeSolver:grid:\n'%s'\n", m.str().c_str() );
}

/*!
 * \brief   check that the data is OK
 */
void TicTacToeSolver::check_game_integrity() {
  DEBUG_PRINT("TicTacToeSolver:check_game_integrity()\n");

  int nb_pions_maggie, nb_pions_player;
  get_nb_of_pions(nb_pions_maggie, nb_pions_player);

  if (nb_pions_maggie > 1 + nb_pions_player) {
    if (maggie_pions == CROSS)
      type_of_answer = ANSWER_TOO_MANY_CROSSES;
    else
      type_of_answer = ANSWER_TOO_MANY_ROUNDS;
    return;
  }

  if (nb_pions_player > 1 + nb_pions_maggie) {
    if (maggie_pions == CROSS)
      type_of_answer = ANSWER_TOO_MANY_ROUNDS;
    else
      type_of_answer = ANSWER_TOO_MANY_CROSSES;
    return;
  }

  if (nb_pions_maggie + nb_pions_player != 9 && nb_pions_maggie
      > nb_pions_player) {
    type_of_answer = ANSWER_IT_IS_PLAYER_TURN;
    return;
  }
}

/*!
 * \brief   a routine to find the next move
 */
TicTacToeSolver::CoordPair TicTacToeSolver::find_next_move() {
  DEBUG_PRINT("TicTacToeSolver:find_next_move()\n");

  type_of_answer = ANSWER_RUNNING;
  check_game_integrity();
  if (type_of_answer == ANSWER_RUNNING || type_of_answer
      == ANSWER_IT_IS_PLAYER_TURN)
    find_winner();
  if (type_of_answer != ANSWER_RUNNING)
    return answer_point;

  has_played = 0;

  if (maggie_level == NOVICE)
    compute_random_move();

  if (maggie_level == INTERMEDIATE) {
    look_for_winning_move();
    if (!has_played)
      block_losing_move();
    if (!has_played)
      compute_random_move();
  }

  if (maggie_level == ADVANCED) {
    look_for_winning_move();
    if (!has_played)
      block_losing_move();
    if (!has_played && rand() % 100 > 40)
      look_for_moves_expert();
    if (!has_played)
      compute_random_move();
  }

  if (maggie_level == EXPERT) {
    look_for_winning_move();
    if (!has_played)
      block_losing_move();
    if (!has_played)
      look_for_moves_expert();
    if (!has_played)
      compute_random_move();
  }

  return answer_point;
}

/*!
 * \brief   copy the answers in the given data
 *
 * \param type will be among
 * ANSWER_RUNNING, ANSWER_MAGGIE_WON, ANSWER_PLAYER_WON, ANSWER_DRAW,
 * ANSWER_TOO_MANY_CROSSES, ANSWER_TOO_MANY_ROUNDS, ANSWER_IT_IS_PLAYER_TURN
 *
 * \param ans a int[2] which will contain the point if type == ANSWER_RUNNING
 */
void TicTacToeSolver::get_answer(AnswerType & type, CoordPair & ans) const {
  type = type_of_answer;
  ans = answer_point;
}

/*!
 * \brief   look for a winning combination,
 * for the computer or the player
 */
void TicTacToeSolver::find_winner() {
  DEBUG_PRINT("TicTacToeSolver:find_winner()\n");

  std::vector<std::string> v;
  v.clear();
  /* lines */
  v.push_back("MMM *** ***");
  v.push_back("*** MMM ***");
  v.push_back("*** *** MMM");
  /* columns */
  v.push_back("M** M** M**");
  v.push_back("*M* *M* *M*");
  v.push_back("**M **M **M");
  /* diagonals */
  v.push_back("M** *M* **M");
  v.push_back("**M *M* M**");
  if (test_match_vector(v)) {
    printf("TicTacToeSolver:Maggie has won\n");
    type_of_answer = ANSWER_MAGGIE_WON;
    return;
  }

  v.clear();
  /* lines */
  v.push_back("PPP *** ***");
  v.push_back("*** PPP ***");
  v.push_back("*** *** PPP");
  /* columns */
  v.push_back("P** P** P**");
  v.push_back("*P* *P* *P*");
  v.push_back("**P **P **P");
  /* diagonals */
  v.push_back("P** *P* **P");
  v.push_back("**P *P* P**");
  if (test_match_vector(v)) {
    printf("TicTacToeSolver:Player has won\n");
    type_of_answer = ANSWER_PLAYER_WON;
    return;
  }

  /* look for a draw */
  int nb_pions_maggie, nb_pions_player;
  get_nb_of_pions(nb_pions_maggie, nb_pions_player);
  if (nb_pions_maggie + nb_pions_player == 9) {
    printf("TicTacToeSolver:Draw\n");
    type_of_answer = ANSWER_DRAW;
    return;
  }
}

/*!
 * \brief   The novice simply places its mark in any empty square.
 * This stategy is very poor and almost never wins.
 */
void TicTacToeSolver::compute_random_move() {
  DEBUG_PRINT("TicTacToeSolver:compute_random_move()\n");
  reason_of_answer = REASON_RANDOM;

  bool OK = 0;
  int col, row;
  while (!OK) {
    col = rand() % NB_COLS;
    row = rand() % NB_ROWS;
    if (board[NB_COLS * row + col] == EMPTY)
      OK = 1;
  }
  affect(col, row);
}

/*!
 * \brief   Most Tic-Tac-Toe players start off as reactionary players.
 * Reactionary players will block their opponents three in a row, or take any three in a row that they can.
 */
void TicTacToeSolver::look_for_winning_move() {
  DEBUG_PRINT("TicTacToeSolver:look_for_winning_move()\n");
  reason_of_answer = REASON_TO_WIN;

  /* look for moves when Maggie can win */
  /* lines */
  if (!has_played)
    test_match_then_affect(".MM *** ***", 0, 0);
  if (!has_played)
    test_match_then_affect("M.M *** ***", 1, 0);
  if (!has_played)
    test_match_then_affect("MM. *** ***", 2, 0);
  if (!has_played)
    test_match_then_affect("*** .MM ***", 0, 1);
  if (!has_played)
    test_match_then_affect("*** M.M ***", 1, 1);
  if (!has_played)
    test_match_then_affect("*** MM. ***", 2, 1);
  if (!has_played)
    test_match_then_affect("*** *** .MM", 0, 2);
  if (!has_played)
    test_match_then_affect("*** *** M.M", 1, 2);
  if (!has_played)
    test_match_then_affect("*** *** MM.", 2, 2);

  /* colums */
  if (!has_played)
    test_match_then_affect(".** M** M**", 0, 0);
  if (!has_played)
    test_match_then_affect("M** .** M**", 0, 1);
  if (!has_played)
    test_match_then_affect("M** M** .**", 0, 2);
  if (!has_played)
    test_match_then_affect("*.* *M* *M*", 1, 0);
  if (!has_played)
    test_match_then_affect("*M* *.* *M*", 1, 1);
  if (!has_played)
    test_match_then_affect("*M* *M* *.*", 1, 2);
  if (!has_played)
    test_match_then_affect("**. **M **M", 2, 0);
  if (!has_played)
    test_match_then_affect("**M **. **M", 2, 1);
  if (!has_played)
    test_match_then_affect("**M **M **.", 2, 2);

  /* diagonals */
  if (!has_played)
    test_match_then_affect(".** *M* **M", 0, 0);
  if (!has_played)
    test_match_then_affect("M** *.* **M", 1, 1);
  if (!has_played)
    test_match_then_affect("M** *M* **.", 2, 2);
  if (!has_played)
    test_match_then_affect("**. *M* M**", 2, 0);
  if (!has_played)
    test_match_then_affect("**M *.* M**", 1, 1);
  if (!has_played)
    test_match_then_affect("**M *M* .**", 0, 2);
}

/*!
 * \brief   look for moves when Maggie can lose
 */
void TicTacToeSolver::block_losing_move() {
  DEBUG_PRINT( "block_losing_move()\n" );
  reason_of_answer = REASON_TO_BLOCK;

  /* lines */
  if (!has_played)
    test_match_then_affect(".PP *** ***", 0, 0);
  if (!has_played)
    test_match_then_affect("P.P *** ***", 1, 0);
  if (!has_played)
    test_match_then_affect("PP. *** ***", 2, 0);
  if (!has_played)
    test_match_then_affect("*** .PP ***", 0, 1);
  if (!has_played)
    test_match_then_affect("*** P.P ***", 1, 1);
  if (!has_played)
    test_match_then_affect("*** PP. ***", 2, 1);
  if (!has_played)
    test_match_then_affect("*** *** .PP", 0, 2);
  if (!has_played)
    test_match_then_affect("*** *** P.P", 1, 2);
  if (!has_played)
    test_match_then_affect("*** *** PP.", 2, 2);

  /* colums */
  if (!has_played)
    test_match_then_affect(".** P** P**", 0, 0);
  if (!has_played)
    test_match_then_affect("P** .** P**", 0, 1);
  if (!has_played)
    test_match_then_affect("P** P** .**", 0, 2);
  if (!has_played)
    test_match_then_affect("*.* *P* *P*", 1, 0);
  if (!has_played)
    test_match_then_affect("*P* *.* *P*", 1, 1);
  if (!has_played)
    test_match_then_affect("*P* *P* *.*", 1, 2);
  if (!has_played)
    test_match_then_affect("**. **P **P", 2, 0);
  if (!has_played)
    test_match_then_affect("**P **. **P", 2, 1);
  if (!has_played)
    test_match_then_affect("**P **P **.", 2, 2);

  /* diagonals */
  if (!has_played)
    test_match_then_affect(".** *P* **P", 0, 0);
  if (!has_played)
    test_match_then_affect("P** *.* **P", 1, 1);
  if (!has_played)
    test_match_then_affect("P** *P* **.", 2, 2);
  if (!has_played)
    test_match_then_affect("**. *P* P**", 2, 0);
  if (!has_played)
    test_match_then_affect("**P *.* P**", 1, 1);
  if (!has_played)
    test_match_then_affect("**P *P* .**", 0, 2);
}

/*
 * The experienced player knows the best starting moves.
 */
void TicTacToeSolver::look_for_moves_expert() {
  DEBUG_PRINT("TicTacToeSolver:look_for_moves_expert()\n");
  reason_of_answer = REASON_EXPERT;

  int nb_pions_maggie, nb_pions_player;
  get_nb_of_pions(nb_pions_maggie, nb_pions_player);

  /* empty grid : take a corner or the center */
  if (nb_pions_maggie + nb_pions_player == 0) {
    int choice = rand() % 5;
    if (choice == 0)
      affect(0, 0);
    if (choice == 1)
      affect(2, 0);
    if (choice == 2)
      affect(2, 2);
    if (choice == 3)
      affect(0, 2);
    if (choice == 4)
      affect(1, 1);
  }
  if (has_played)
    return;

  /* one previous shot : */
  if (nb_pions_maggie + nb_pions_player == 1) {
    if (board[0 * NB_COLS + 0] == player_pions)
      affect(1, 1);
    else if (board[0 * NB_COLS + 2] == player_pions)
      affect(1, 1);
    else if (board[2 * NB_COLS + 0] == player_pions)
      affect(1, 1);
    else if (board[2 * NB_COLS + 2] == player_pions)
      affect(1, 1);
    else if (board[1 * NB_COLS + 1] == player_pions) // choose a corner
      affect(2 * (rand() % 2), 2 * (rand() % 2));
  }
  if (has_played)
    return;

  /* more than one or two shots */
  /* look for moves when you can win in two shots */
  // when you got two corners and the third free, play it
  // for instance
  // ..M
  // M** => play 0, 0
  // .**
  if (!has_played)
    test_match_then_affect("M.. **M **.", 2, 0);
  if (!has_played)
    test_match_then_affect(".M. **. **M", 2, 0);
  if (!has_played)
    test_match_then_affect("..M M** .**", 0, 0);
  if (!has_played)
    test_match_then_affect(".M. .** M**", 0, 0);
  if (!has_played)
    test_match_then_affect(".** M** ..M", 0, 2);
  if (!has_played)
    test_match_then_affect("M** .** .M.", 0, 2);
  if (!has_played)
    test_match_then_affect("**. **M M..", 2, 2);
  if (!has_played)
    test_match_then_affect("**M **. .M.", 2, 2);

  // try to generate the situation of before
  // for instance
  // ...
  // .** => play 0,1
  // M**
  if (!has_played)
    test_match_then_affect("M.. **. **.", 2, 1);
  if (!has_played)
    test_match_then_affect("... **. **M", 1, 0);
  if (!has_played)
    test_match_then_affect("..M .** .**", 0, 1);
  if (!has_played)
    test_match_then_affect("... .** M**", 1, 0);
  if (!has_played)
    test_match_then_affect("M** .** ...", 1, 2);
  if (!has_played)
    test_match_then_affect(".** .** ..M", 0, 1);
  if (!has_played)
    test_match_then_affect("**M **. ...", 1, 2);
  if (!has_played)
    test_match_then_affect("**. **. M..", 2, 1);

}

/*!
 * \brief   test if the situation matches a special situation
 *
 * \param   situation the string describing the game
 * 'M':Maggie
 * 'O':the player
 * '.':an empty case
 */
bool TicTacToeSolver::test_match(const std::string & situation) {
  //DEBUG_PRINT(situation);
  std::string::const_iterator curr_letter = situation.begin();
  Board::const_iterator curr_case = board.begin();

  for (unsigned int row = 0; row < NB_ROWS; ++row) {
    for (unsigned int col = 0; col < NB_COLS; ++col) {
      //cout << "curr_letter :" << *curr_letter << " - curr_case:" << *curr_case << std::endl;
      /* Maggie pion */
      if (*curr_letter == 'M') {
        if (*curr_case != maggie_pions)
          return false;
      }
      /* other pion */
      else if (*curr_letter == 'P') {
        if (*curr_case != player_pions)
          return false;
      }
      /* space */
      else if (*curr_letter == '.') {
        if (*curr_case != EMPTY)
          return false;
      } // end of tests about letters
      ++curr_letter;
      ++curr_case;
    } // end loop i

    // skip the space between each line
    if (row < 3)
      ++curr_letter;
  } // end loop j

  std::ostringstream m;
  m << "Match with:" << situation;
  DEBUG_PRINT("TicTacToeSolver:'%s'\n", m.str().c_str() );
  return true;
}

/*!
 * \brief   call test_match for each one of the situations
 *
 * \param   situations the std::vector of situations to match
 * \return true as soon as it founds one match
 */
bool TicTacToeSolver::test_match_vector(const std::vector<std::string> &situations) {
  for (std::vector<std::string>::const_iterator it = situations.begin(); it
       < situations.end(); it++) {
    if (test_match(*it))
      return 1;
  }
  return 0;
}

/*!
 * \brief   affect the answer with new variables
 *
 * \param   i the abscissae
 * \param   j the coordinate
 */
void TicTacToeSolver::affect(Coord col, Coord row) {
  DEBUG_PRINT("TicTacToeSolver:Affecting col:%i, row:%i\n", col, row);
  answer_point.first = col;
  answer_point.second = row;
  has_played = 1;
}

/*!
 * \brief   combine test_match() and affect()
 */
bool TicTacToeSolver::test_match_then_affect(const std::string &situation,
                                             Coord col, Coord row) {
  if (test_match(situation)) {
    affect(col, row);
    return 1;
  } else
    return 0;
}

