#include "games_vision/CHangMan.h"

/***************************************************************************//**
 * \file test_CHangMan.cpp
 *
 * \brief Some tests for the hangman solver
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 05/06/2009
 *******************************************************************************/

////////////////////////////////////////////////////////////////////////////////

void hangman_test() {
  HangmanSolver hs;
  hs.init(HangmanSolver::ROLE_FINDER, 5, PATH_ALPHABET_ES, PATH_DICTIONNARY_ES);
  //hs.display_all_possible_words();
  std::cout << std::endl << "hs.display_word_letters_status()" << std::endl;
  hs.display_word_letters_status();

  std::cout << std::endl << "hs.finder_compute_next_guess()" << std::endl;
  hs.finder_compute_next_guess();

  std::cout << std::endl << "hs.set_letter_tried('i')" << std::endl;
  hs.set_letter_tried('i');

  std::cout << std::endl << "hs.set_letter_at_position(0, 'c')" << std::endl;
  hs.set_letter_at_position(0, 'c');
  //cout << std::endl << "test:" << hs.check_word_conform_to_syntax( "cerca" ) << std::endl;

  std::cout << std::endl << "hs.set_letter_at_position(3, 'c')" << std::endl;
  hs.set_letter_at_position(3, 'c');
  //cout << std::endl << "test:" << hs.check_word_conform_to_syntax( "cerca" ) << std::endl;

  std::cout << std::endl << "hs.display_word_letters_status()" << std::endl;
  hs.display_word_letters_status();

  std::cout << std::endl << "hs.finder_compute_next_guess()" << std::endl;
  hs.finder_compute_next_guess();

  std::cout << std::endl << "hs.display_all_possible_words()" << std::endl;
  hs.display_all_possible_words();

  HangmanSolver::FinderAnswer type;
  std::string answer;
  hs.finder_get_answer( type, answer );
  std::cout << "The answer is '" << answer << "'" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void purge_dict() {
  //HangmanSolver::purge_dictionnary( "data/spanish_no_purged.dict",  "data/spanish.dict" );
  //HangmanSolver::purge_dictionnary( "data/english_no_purged.dict",  "data/english.dict" );
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   an exchange between the finder and the solver
 */
void exchange() {
#if 0 // TODO
  HangmanSolver::FinderAnswer finder_type;
  HangmanSolver::HangerAnswer hanger_type;
  std::string answerFinder;
  std::vector<int> posHanger;

  HangmanSolver hs;
  hs.init(HangmanSolver::ROLE_FINDER, 8, PATH_ALPHABET_ES, PATH_DICTIONNARY_ES);
  HangmanSolver hs2;
  hs2.init(HangmanSolver::ROLE_HANGER, 8, PATH_ALPHABET_ES, PATH_DICTIONNARY_ES);

  int nbSteps = 0;

  while ( hs2.get_game_status() == HangmanSolver::GAME_RUNNING ) {
    std::cout << "*******************************************" << std::endl;
    if (nbSteps > 0)
      hs.finder_receive_data_from_hanger( hanger_type, posHanger );
    hs.finder_compute_next_guess();
    hs.display_word_letters_status();
    std::cout << "hs.get_number_of_possible_words():" << hs.get_number_of_possible_words() << std::endl;
    if (hs.get_number_of_possible_words() < 100) hs.display_all_possible_words();
    hs.finder_get_answer( finder_type, answerFinder );

    //cout << std::endl;
    hs2.hanger_receive_data_from_finder( finder_type, answerFinder);
    hs2.hanger_get_answer( hanger_type, posHanger);
    nbSteps++;
  }

  std::cout << "status:" << hs2.game_status_in_string << " - steps:" << nbSteps << " - failures:" << hs.number_of_failures << std::endl;
#endif
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  //purge_dict();
  //CHangMan::generate_alphabet(32, 32);

  hangman_test();
  //exchange();
}

