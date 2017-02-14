// Bring in gtest
#include <gtest/gtest.h>
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

TEST(TestSuite, solver_test) {
  HangmanSolver hs;
  HangmanSolver::FinderAnswer type;
  std::string answer;
  hs.init(HangmanSolver::ROLE_FINDER, 5, PATH_ALPHABET_ES, PATH_DICTIONNARY_ES);
  ASSERT_TRUE(hs.get_game_status() == HangmanSolver::GAME_RUNNING);

  // check finder is initialized correctly
  //hs.display_all_possible_words();
  std::cout << std::endl << "hs.display_word_letters_status()" << std::endl;
  hs.display_word_letters_status();
  std::cout << std::endl << "hs.finder_compute_next_guess()" << std::endl;
  hs.finder_compute_next_guess();
  unsigned int nwords0 = hs.get_number_of_possible_words();
  ASSERT_TRUE(nwords0 > 0) << "nwords0:" << nwords0;
  hs.finder_get_answer(type, answer);
  ASSERT_TRUE(type == HangmanSolver::FINDER_ANSWER_LETTER) << "type:" << type;

  // try a first letter
  std::cout << std::endl << "hs.set_letter_tried('i')" << std::endl;
  hs.set_letter_tried('i');
  hs.finder_compute_next_guess();
  unsigned int nwords1 = hs.get_number_of_possible_words();
  ASSERT_TRUE(nwords0 > nwords1) << "nwords0:" << nwords0 << ", nwords1:" << nwords1;

  std::cout << std::endl << "hs.set_letter_at_position(0, 'c')" << std::endl;
  hs.set_letter_at_position(0, 'c');
  hs.finder_compute_next_guess();
  unsigned int nwords2 = hs.get_number_of_possible_words();
  ASSERT_TRUE(nwords1 > nwords2) << "nwords1:" << nwords1 << ", nwords2:" << nwords2;
  //cout << std::endl << "test:" << hs.check_word_conform_to_syntax( "cerca" ) << std::endl;

  std::cout << std::endl << "hs.set_letter_at_position(3, 'c')" << std::endl;
  hs.set_letter_at_position(3, 'c');
  hs.finder_compute_next_guess();
  //cout << std::endl << "test:" << hs.check_word_conform_to_syntax( "cerca" ) << std::endl;

  std::cout << std::endl << "hs.display_word_letters_status()" << std::endl;
  hs.display_word_letters_status();

  std::cout << std::endl << "hs.finder_compute_next_guess()" << std::endl;
  hs.finder_compute_next_guess();

  std::cout << std::endl << "hs.display_all_possible_words()" << std::endl;
  hs.display_all_possible_words();
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
TEST(TestSuite, exchange) {
  HangmanSolver::FinderAnswer finder_type;
  HangmanSolver::HangerAnswer hanger_type;
  std::string answerFinder;
  std::vector<unsigned int> posHanger;

  HangmanSolver finder, hanger;
  finder.init(HangmanSolver::ROLE_FINDER, 8, PATH_ALPHABET_ES, PATH_DICTIONNARY_ES);
  hanger.init(HangmanSolver::ROLE_HANGER, 8, PATH_ALPHABET_ES, PATH_DICTIONNARY_ES);

  int nbSteps = 0;

  while ( hanger.get_game_status() == HangmanSolver::GAME_RUNNING ) {
    std::cout << "*******************************************" << std::endl;
    if (nbSteps > 0)
      finder.finder_receive_data_from_hanger( hanger_type, posHanger );
    finder.finder_compute_next_guess();
    finder.display_word_letters_status();
    std::cout << "hs.get_number_of_possible_words():" << finder.get_number_of_possible_words() << std::endl;
    if (finder.get_number_of_possible_words() < 100) finder.display_all_possible_words();
    finder.finder_get_answer( finder_type, answerFinder );

    //cout << std::endl;
    hanger.hanger_receive_data_from_finder( finder_type, answerFinder);
    hanger.hanger_get_answer( hanger_type, posHanger);
    nbSteps++;
  }

  std::cout << "status:" << HangmanSolver::game_status2string(hanger.get_game_status())
            << " - steps:" << nbSteps
            << " - failures:" << finder.get_number_of_failures() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

