/***************************************************************************//**
 * \file TicTacToeSkill.cpp
 *
 * \brief The implementation of TicTacToeSkill.h
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 26/05/2009
 *******************************************************************************/

#include "games_vision/tic_tac_toe_skill.h"
// vision_utils
#include <vision_utils/img_path.h>
#include <vision_utils/infosimage.h>
#include "vision_utils/connected_comp_interface.h"
#include "vision_utils/timer.h"

TicTacToeSkill::TicTacToeSkill() :
  PlayzoneSequentialUser("TICTACTOE_START", "TICTACTOE_STOP") {
  DEBUG_PRINT("TicTacToeSkill:ctor");

  // init the solver
  board.resize(TicTacToeSolver::NB_COLS * TicTacToeSolver::NB_ROWS, TicTacToeSolver::UNKNOWN);
  _solver.set_Maggie_pions(TicTacToeSolver::CROSS);
  _solver.set_Maggie_level(TicTacToeSolver::ADVANCED);

  // init the comparer - ensure it is in the same order as CellContent
  std::vector<std::string> pawn_imgs;
  pawn_imgs.push_back(TICTACTOE_DIR + "emptyRound.png"); // index 0
  pawn_imgs.push_back(TICTACTOE_DIR + "cross.png"); //      index 1
  comparer.set_models(pawn_imgs, cv::Size(32, 32));
  assert(comparer.get_models_nb() == 2);
}

////////////////////////////////////////////////////////////////////////////////

TicTacToeSkill::~TicTacToeSkill() {
  DEBUG_PRINT("TicTacToeSkill:dtor");
}

////////////////////////////////////////////////////////////////////////////////

void TicTacToeSkill::create_subscribers_and_publishers_playzone() {
  DEBUG_PRINT("TicTacToeSkill:create_subscribers_and_publishers_playzone()");

  /*
  * configure an eventual drawer
  */
  // supply a nice background for the drawer
  cv::Size pzs(300, 300);
  _playzone_modified_for_drawer.create(pzs);
  _playzone_modified_for_drawer.setTo(cv::Vec3b(200, 200, 200));
  for (unsigned int col = 0 ; col < TicTacToeSolver::NB_COLS ; ++col)
    for (unsigned int row = 0 ; row < TicTacToeSolver::NB_ROWS ; ++row)
      cv::rectangle(_playzone_modified_for_drawer,
                    cv::Rect(coord_left_cell(pzs, col), coord_up_cell(pzs, row),
                             cell_width(pzs), cell_height(pzs)),
                    ((col +  row * TicTacToeSolver::NB_ROWS) % 2 == 0 ?
                       CV_RGB(220, 220, 255) : CV_RGB(220, 255, 220)),
                    -1);
  //  cv::imshow("playzone_out", playzone_out);
  //  cv::waitKey(5000);

  /* instructions */
  say_text("|es:Vamos a jugar al tres en raya. "
           "Si quieres empezar el juego pon la primera ficha. "
           "Despues toca mi hombro. "
           "Si quieres que empiece yo, toca mi hombro ahora. "
           "Durante el juego, toca mi hombro cada vez que sea mi turno. "

           "|en:Let us play tic tac toe. "
           "If you want to start the game, put the first checker. "
           "Then touch my shoulder. "
           "If you want that I start, touch my shoulder now. "
           "During the game, touch my shoulder whenever it is my turn.");

  // make maggie say what pions she has give the crosses to Maggie
  say_text(_solver.get_Maggie_pions_sentence());
  say_text(_solver.get_Maggie_level_sentence());
  // set eyes as tranquility

  /* move the head up */
  //neck.move_theta( 50 );
  //sleep(3);
  //neck.move_phi( -20 );

  say_text("|en:Start whenever you want, I'm waiting for you."
           "|es:Empeza cuando quieres, espero tu decision.");
}

////////////////////////////////////////////////////////////////////////////////

void TicTacToeSkill::shutdown_subscribers_and_publishers_playzone() {
  DEBUG_PRINT("TicTacToeSkill:shutdown_subscribers_and_publishers_playzone()");
  shut_up();
  say_text("|es:Basta con el tres en raya, hacemos otra cosa? "
           "|en:Let us stop playing tic tac toe, what do we do now? ");
}

////////////////////////////////////////////////////////////////////////////////

void TicTacToeSkill::do_stuff_before_get_playzone() {
  DEBUG_PRINT("TicTacToeSkill:do_stuff_before_get_playzone()");
  PlayzoneSequentialUser::do_stuff_before_get_playzone();
}

////////////////////////////////////////////////////////////////////////////////

void TicTacToeSkill::do_stuff_after_get_playzone(bool was_find_and_process_success) {
  DEBUG_PRINT("TicTacToeSkill:do_stuff_after_get_playzone(success:%i)",
             was_find_and_process_success);
  PlayzoneSequentialUser::do_stuff_after_get_playzone(was_find_and_process_success);
}

////////////////////////////////////////////////////////////////////////////////

double TicTacToeSkill::left_margin(const cv::Size & pzs) {
  return 1.f * MARGIN_RATE_HOR * pzs.width;
}

double TicTacToeSkill::up_margin(const cv::Size & pzs) {
  return 1.f * MARGIN_RATE_VER * pzs.height;
}

int TicTacToeSkill::coord_left_cell(const cv::Size & pzs, const TicTacToeSolver::Coord col) {
  return (int) (left_margin(pzs) + col * cell_width(pzs));
}

int TicTacToeSkill::coord_up_cell(const cv::Size & pzs, const TicTacToeSolver::Coord row) {
  return (int) (up_margin(pzs) + row * cell_height(pzs));
}

double TicTacToeSkill::cell_width(const cv::Size & pzs) {
  return (1.f * pzs.width - 2.f * left_margin(pzs)) / TicTacToeSolver::NB_COLS;
}

double TicTacToeSkill::cell_height(const cv::Size & pzs) {
  return (1.f * pzs.height - 2.f * up_margin(pzs)) / TicTacToeSolver::NB_ROWS;
}

#ifdef PZ_FIND_IMAGES
int save_counter = 0;
#endif

/* recognize the grid - returns false if no success */
bool TicTacToeSkill::process_pz(const cv::Mat3b & pz) {
  DEBUG_PRINT("TicTacToeSkill:process_pz()");
  //cv::imshow("pz", pz); cv::waitKey( 50000 );

  //  if (consecutive_failures == 0) {
  //  int choice = rand() % 4;
  //  if (choice == 0)
  //  tts.say_text("|es:Me toca jugar!");
  //  if (choice == 1)
  //  tts.say_text("|es:Es mi turno!");
  //  if (choice == 2)
  //  tts.say_text("|es:Ahora yo, me toca.");
  //  if (choice == 3)
  //  tts.say_text("|es:Si, voy a encontrar una buena jugada !");
  //  }
  vision_utils::Timer timer;

  cv::Size pzs = pz.size();
  cv::cvtColor(pz, _playzone_monochrome, CV_RGB2GRAY);
  cv::threshold(_playzone_monochrome, _playzone_monochrome, 128, 255, CV_THRESH_BINARY_INV);

  /* display */
#ifdef SAVE_IMAGES
  cv::imwrite("20-playzone_monochrome.png", playzone_monochrome);
#endif // SAVE_IMAGES

  // cv::imshow(window1Name, pz);
  //  cv::imshow(window2Name, playzone_monochrome);
  //  cv::waitKey( 50000 );

  /* scan each case with the comparer */
  cv::Mat1b cell_monochrome_continuous;
  for (unsigned int cell_index = 0;
       cell_index < TicTacToeSolver::NB_COLS * TicTacToeSolver::NB_ROWS;
       ++cell_index) {
    int col = cell_index / TicTacToeSolver::NB_ROWS;
    int row = cell_index % TicTacToeSolver::NB_ROWS;
    TicTacToeSolver::CellContent currentSymbol = TicTacToeSolver::UNKNOWN;

    /*
  * reduce frame monochrome to the current cell
  */
    int xL = coord_left_cell(pzs, col);
    int yL = coord_up_cell(pzs, row);
    _playzone_monochrome(cv::Rect(xL, yL, cell_width(pzs), cell_height(pzs)) )
        .copyTo(cell_monochrome_continuous);
    DEBUG_PRINT("TicTacToeSkill:(i:%i, j:%i)-starting analysis, playzone_monochrome:'%s'",
               col, row, vision_utils::infosImage( cell_monochrome_continuous ).c_str());
    //cv::imshow(window2Name, cell_monochrome);
    //cv::waitKey( 50000 );
#ifdef PZ_FIND_IMAGES
    cv::rectangle( playzone_out,
                   cv::Rect(xL, yL, cell_width(pzs), cell_height(pzs)),
                   vision_utils::color_scalar<cv::Scalar>(3* j + i, 9), 2);
#endif

    /*
  * determine if the case is empty
  */
    double emptyRate = 1.f - 1.f * cv::countNonZero(cell_monochrome_continuous)
        / (cell_width(pzs) * cell_height(pzs));
    DEBUG_PRINT("TicTacToeSkill:(i:%i, j:%i)-cv::countNonZero(cell_monochrome):%i - Empty rate:%f %%",
               col, row, cv::countNonZero(cell_monochrome_continuous), 100.f * emptyRate);
    if (emptyRate > EMPTYNESS_THRESHOLD) {
      DEBUG_PRINT("TicTacToeSkill:comparer.getBestResult():%f", comparer.getBestResult());
      currentSymbol = TicTacToeSolver::EMPTY;
#ifdef PZ_FIND_IMAGES
      // empty
      cv::putText(playzone_out, "empty",
                  cv::Point(xL + 3, yL + (int) (cell_height(pzs)/2)),
                  cv::FONT_HERSHEY_TRIPLEX, .5,
                  vision_utils::color_scalar<cv::Scalar>(3* j + i, 9) );
#endif
    }


    /*
  * if the case is not empty, determine cross or circle
  */
    if (currentSymbol == TicTacToeSolver::UNKNOWN) {
      comparer.compareFile(cell_monochrome_continuous);
      //std::cout << ", d=" << comparer.getBestResult();
      //  DEBUG_PRINT("TicTacToeSkill:(i:%i, j:%i)-Best filename:'%s', best result:%f",
      //  i, j, comparer.getBestFilename().c_str(), comparer.getBestResult());

      if (comparer.getBestResult() < DISTANCE_MAX_BEFORE_D2) {
        TicTacToeSolver::CellContent cell = (TicTacToeSolver::CellContent) comparer.get_best_index();
        // std::string filename = comparer.getBestFilename();
        if (cell == TicTacToeSolver::CROSS)
          currentSymbol = TicTacToeSolver::CROSS;
        else if (cell == TicTacToeSolver::ROUND)
          currentSymbol = TicTacToeSolver::ROUND;
#ifdef PZ_FIND_IMAGES // paint the content of the cell
        for(int x = 0 ; x < cell_monochrome.cols ; ++x) {
          for(int y = 0 ; y < cell_monochrome.rows ; ++y) {
            if (cell_monochrome(x,y) > 0)
              cv::line(playzone_out,
                       cv::Point(x + xL, y + yL),
                       cv::Point(x + xL, y + yL),
                       vision_utils::color_scalar<cv::Scalar>(3* j + i, 9));
          }
        }
#endif
      } else {
        DEBUG_PRINT("TicTacToeSkill:(i:%i, j:%i)-Best distance returned by the comparer:%f"
                   " > DISTANCE_MAX_BEFORE_D2:%f"
                   ", we still don't know what this cell is.",
                   col, row, comparer.getBestResult(), DISTANCE_MAX_BEFORE_D2);
      }
    } // end of if (currentSymbol == TicTacToeSolver::UNKNOWN)


    /*
  * if we reach here, the case is not empty, but the content of the whole cell is not obvious
  */
    if (currentSymbol == TicTacToeSolver::UNKNOWN) {
      // first try with selecting the biggest component
      std::vector<cv::Point> biggestComponent;
      vision_utils::biggestComponent_vector2(cell_monochrome_continuous,
                                            biggestComponent);
      comparer.compareVector(biggestComponent);
      //  DEBUG_PRINT("TicTacToeSkill:(i:%i, j:%i)-biggestComponent.size():%i, Best filename:'%s', best result:%f",
      //  i, j, (int) biggestComponent.size(),
      //  comparer.getBestFilename().c_str(), comparer.getBestResult());

      if (comparer.getBestResult() < DISTANCE_MAX_AFTER_D2) {
        // std::string filename = comparer.getBestFilename();
        TicTacToeSolver::CellContent cell = (TicTacToeSolver::CellContent) comparer.get_best_index();
        if (cell == TicTacToeSolver::CROSS)
          currentSymbol = TicTacToeSolver::CROSS;
        else if (cell == TicTacToeSolver::ROUND)
          currentSymbol = TicTacToeSolver::ROUND;
#ifdef PZ_FIND_IMAGES // paint the biggest component
        for (std::vector<cv::Point>::iterator it = biggestComponent.begin();
             it < biggestComponent.end(); it++) {
          cv::line(playzone_out,
                   cv::Point2i(it->x +xL, it->y + yL),
                   cv::Point2i(it->x +xL, it->y + yL),
                   vision_utils::color_scalar<cv::Scalar>(3* j + i, 9));
        }
#endif
      } else {
        DEBUG_PRINT("TicTacToeSkill:(i:%i, j:%i)-The best distance returned by the comparer:%f"
                   " > DISTANCE_MAX_AFTER_D2:%f"
                   ", we still don't know what this cell is.",
                   col, row, comparer.getBestResult(), DISTANCE_MAX_AFTER_D2);
      }
    } // end of if (currentSymbol == TicTacToeSolver::UNKNOWN)

    /*
  * if we still don't know what is this cell -> fail
  */
    if (currentSymbol == TicTacToeSolver::UNKNOWN) {
      ROS_WARN("TicTacToeSkill:(col %i, row %i)-We still haven't determined what is "
             "this cell - restarting analysis", col, row);
      // notify of failure
      return false;
    } // end of if (currentSymbol == TicTacToeSolver::UNKNOWN)

    board[row * TicTacToeSolver::NB_COLS + col] = currentSymbol;
  } // end loop cell_index

  DEBUG_PRINT("TicTacToeSkill: time after cell loop:%g ms, board:'%s'",
         timer.time(), get_board_as_string().c_str());
  //tts.say_text("|es:Bien.");

#ifdef PZ_FIND_IMAGES
  DEBUG_PRINT("TicTacToeSkill:Saving images");
  std::ostringstream name;
  save_counter++;
  name << "200-playzone_monochrome_" << save_counter << ".png";
  cv::imwrite(name.str(), playzone_monochrome );
  name.str("");
  name << "210-playzone_out" << save_counter << ".png";
  cv::imwrite( name.str(), playzone_out );
#endif

  analyze_next_move();
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void TicTacToeSkill::first_move_hook(bool has_player_started) {
  if (has_player_started)
    say_text("|es:Asi que, tu has hecho el primer movimiento !"
             "|en:So, you've made the first move!");
  else
    say_text("|es:Me gusta hacer el primer movimiento del juego !"
             "|en:I like to make the first move of the game!");
}

////////////////////////////////////////////////////////////////////////////////

void TicTacToeSkill::analyze_next_move() {
  _solver.set_board(board);
  _solver.display_grid();
  shut_up();

  /* say special stuff if the game has started */
  int a, b;
  _solver.get_nb_of_pions(a, b);
  if (a == 0 && b == 0) {
    first_move_hook(false);
  } else if (a == 0 && b == 1) {
    first_move_hook(true);
  }

  _solver.find_next_move();

  TicTacToeSolver::AnswerType ans_type;
  TicTacToeSolver::CoordPair ans_point;
  _solver.get_answer(ans_type, ans_point);
  TicTacToeSolver::Coord ans_col = ans_point.first, ans_row = ans_point.second;

  if (ans_type == TicTacToeSolver::ANSWER_RUNNING) {
    std::ostringstream txt_es, txt_en;
    txt_es << "|es:Voy a jugar ";
    txt_en << "|en:I will play ";
    txt_es << (_solver.maggie_pions == TicTacToeSolver::CROSS ? "una cruz" : "un circulo");
    txt_en << (_solver.maggie_pions == TicTacToeSolver::CROSS ? "a cross" : "a round");
    txt_es << " en la fila  " << ans_col <<
              " y la columna  " << ans_row << " desde tu punto de vista !";
    txt_en << " in the row  " << ans_col <<
              " and the column  " << ans_row << " from your point of view !";
    std::ostringstream txt;
    txt << txt_es.str() << txt_en.str();
    say_text(txt.str());

    /* check if Maggie wons the next time */
    if (_solver.reason_of_answer == TicTacToeSolver::REASON_TO_WIN) {
      say_text("|es:Creo que este va a ser un buen movimiento."
               "|en:I think this will be a good move.");
    }
    /* look for a possibility to win from the player */
    else if (_solver.reason_of_answer == TicTacToeSolver::REASON_TO_BLOCK) {
      if (rand() % 2 == 1)
        say_text("|es:No creas que vas a ganar tan facilmente!"
                 "|en:Don't think you can win so easily !");
      else
        say_text("|es:No vas a ganar con solo este movimiento ! Cacho carne !"
                 "|en:You won't win with just that move, pal !");
      board[ans_row * 3 + ans_col] = _solver.maggie_pions;
      _solver.set_board(board);
      _solver.has_played = 0;
      _solver.block_losing_move();
      if (_solver.has_played)
        say_text("|es:Pero... creo que no va a ser suficiente para bloquearte."
                 "|en:But... I am afraid it won't be enough to block you.");
    }
    /* announce a complex strategy */
    else if (_solver.reason_of_answer == TicTacToeSolver::REASON_EXPERT) {
      say_text("|es:Jajaja, tengo una idea !"
               "|en:Haha, I have an idea.");
    }

    /* illustrate for the drawer */
    illustrate_next_move(_playzone_modified_for_drawer, ans_col, ans_row, _solver.maggie_pions);
    _playzone_modified_for_drawer.copyTo(_playzone_illus);

    // set eyes as HAPPY
    set_eyes("NORMAL");
  }

  else if (ans_type == TicTacToeSolver::ANSWER_MAGGIE_WON) {
    say_text("|es:bien!!  Yo he ganado el juego! "
             "No es tan facil ganar contra un robot, eh! "
             "Para jugar otra vez, quita las fichas, y despues tocame."
             "|en:Good ! I have won the game. "
             "It is not that easy to win against a robot, is it? "
             "To start over, just remove the pawns from the game, then touch me. "
             );
    // set eyes as HAPPY
    set_eyes("NORMAL");
    // play gesture "flori_happy"
    std_msgs::String gesture_name; gesture_name.data = "flori_happy";
    _gesture_pub.publish(gesture_name);
    sleep(5);
  }

  else if (ans_type == TicTacToeSolver::ANSWER_PLAYER_WON) {
    say_text("|es:Tu has ganado el juego. Enahorabuena! "
             "Seguro que la proxima vez, gano. "
             "Para jugar otra vez, quita las fichas, y despues tocame. "
             "|en:You have won the game. Congratulations! "
             "Next time, I will for sure. "
             "To start over, just remove the pawns from the game, then touch me. "
             );
    // set eyes as ANGRY
    set_eyes("ANGRY");
    // play gesture "sad"
    std_msgs::String gesture_name; gesture_name.data = "flori_sad";
    _gesture_pub.publish(gesture_name);
    sleep(5);
  }

  else if (ans_type == TicTacToeSolver::ANSWER_DRAW) {
    say_text("|es:El juego esta acabado! "
             "Nadie ha ganado. "
             "Si quieres jugar otra vez, quita las fichas, y despues tocame. "
             "|en:The game is over. "
             "It is a draw. "
             "To start over, just clean the game, then touch me. "
             );
    // set eyes as ANGRY
    set_eyes("ANGRY");
  }

  else if (ans_type == TicTacToeSolver::ANSWER_TOO_MANY_CROSSES) {
    say_text("|es:Hay muchas mas cruces que circulos!"
             "|en:There are many more crosses than rounds!");
    // play gesture "no_mute"
    //  std_msgs::String gesture_name; gesture_name.data = "flori_no_mute";
    //  _gesture_pub.publish(gesture_name);
    // set eyes as ANGRY
    set_eyes("ANGRY");
  }

  if (ans_type == TicTacToeSolver::ANSWER_TOO_MANY_ROUNDS) {
    say_text("|es:Hay muchos mas circulos que cruces!"
             "|en:There are many more rounds than crosses!");
    // play gesture "no_mute"
    //  std_msgs::String gesture_name; gesture_name.data = "flori_no_mute";
    //  _gesture_pub.publish(gesture_name);
    // set eyes as ANGRY
    set_eyes("ANGRY");
  }

  else if (ans_type == TicTacToeSolver::ANSWER_IT_IS_PLAYER_TURN) {
    say_text("|es:Es tu turno de jugar. "
             "Tu solo necessitas tocarme cuando es mi turno! "
             "|en:It's your turn to play. "
             "You just need to touch me when it's my turn!");
    // play gesture "no_mute"
    //  std_msgs::String gesture_name; gesture_name.data = "flori_no_mute";
    //  _gesture_pub.publish(gesture_name);
  }
} // end analyze_next_move();

////////////////////////////////////////////////////////////////////////////////

void TicTacToeSkill::illustrate_next_move(cv::Mat & pz_illus,
                                          const TicTacToeSolver::Coord col,
                                          const TicTacToeSolver::Coord row,
                                          const TicTacToeSolver::CellContent new_move) {
  DEBUG_PRINT("TicTacToeSkill:illustrate_next_move( (%i, %i), new_move:%i", col, row, new_move);
  cv::Size pzs = pz_illus.size();
  int cell_center_x = coord_left_cell(pzs, col) + cell_width(pzs) / 2;
  int cell_center_y = coord_up_cell(pzs, row) + cell_height(pzs) / 2;
  int radius = std::min(cell_width(pzs), cell_height(pzs)) / 3;
  int width = 9;
  cv::Scalar color = CV_RGB(0, 0, 0);

  if (new_move == TicTacToeSolver::CROSS) {
    cv::line(pz_illus,
             cv::Point(cell_center_x - radius, cell_center_y + radius),
             cv::Point(cell_center_x + radius, cell_center_y - radius),
             color, width);
    cv::line(pz_illus,
             cv::Point(cell_center_x - radius, cell_center_y - radius),
             cv::Point(cell_center_x + radius, cell_center_y + radius),
             color, width);
  } else if (new_move == TicTacToeSolver::ROUND) {
    cv::circle(pz_illus, cv::Point(cell_center_x, cell_center_y), radius,
               color, width);
  }
}
