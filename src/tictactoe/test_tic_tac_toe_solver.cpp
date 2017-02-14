#include "games_vision/tic_tac_toe_solver.h"


// opencv
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

void playTest() {
  TicTacToeSolver tictac;
  tictac.set_Maggie_pions(TicTacToeSolver::ROUND);

  std::ostringstream board;
  //".........";
  //    board << "X.O";
  //    board << ".X.";
  //    board << "..O";
  tictac.set_board_string(board.str());
  //cout << tictac.test_match( "*.P *M* ..*" ) << std::endl << std::endl;

  while (1) {
    std::cout << "****************************************" << std::endl;

    tictac.display_grid();
    TicTacToeSolver::CoordPair choice;
    for (int i = 0; i < 10; ++i) {
      choice = tictac.find_next_move();
      std::cout << std::endl;
    }
    if (tictac.type_of_answer != TicTacToeSolver::ANSWER_RUNNING)
      break;
    tictac.board[choice.second * 3 + choice.first] = tictac.maggie_pions;

    tictac.display_grid();
    int i = 0, j = 0;
    std::cout << " i? ";
    std::cin >> i;
    std::cout << " j? ";
    std::cin >> j;
    tictac.board[j * 3 + i] = tictac.player_pions;
  }

  std::cout << "Final status:" << tictac.type_of_answer << std::endl;
}

void playTest2() {
  TicTacToeSolver tictac;
  std::ostringstream board;
  //".........";
  board << "O.O";
  board << "XX.";
  board << "OXO";

  tictac.set_board_string(board.str());
  tictac.display_grid();
  tictac.find_winner();
  std::cout << "Final status:" << tictac.type_of_answer << std::endl;
}

/** tests */
int main(int argc, char** argv) {
  playTest();
  //playTest2();
  return 0;
}

