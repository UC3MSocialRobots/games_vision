
/***************************************************************************//**
 * \file HangmanSolver.cpp
 *
 * \brief The implementation of HangmanSolver.h
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 04/06/2009
 *******************************************************************************/

#include "games_vision/HangmanSolver.h"

/*!
 * \brief   constructor
 *
 * \param   role
 *            the type of hangman solver : ROLE_FINDER or ROLE_HANGER
 * \param   nb_characters
 *            the number of characters in the unknown word
 */
HangmanSolver::HangmanSolver() {
  NB_LETTERS_IN_WORD = 0;
  NB_LETTERS_IN_ALPHABET = 0;
  ROLE = ROLE_FINDER;
}

void HangmanSolver::init(Role role,
                         int nb_characters,
                         const std::string &alphabet_path,
                         const std::string &dictionary_path) {
  ROS_INFO("init(role:%i, nb_characters:%i, alphabet_path:'%s', dictionary_path:%s)",
               role, nb_characters, alphabet_path.c_str(), dictionary_path.c_str());

  if (role != ROLE_HANGER && role != ROLE_FINDER) {
    ROS_INFO("HangmanSolver:Hey, the role you gave (%i)  can only be ROLE_FINDER (%i) or ROLE_HANGER (%i) ! Ending.", role, ROLE_FINDER, ROLE_HANGER);
    exit(-1);
  }
  ROS_INFO("CONSTRUCTOR");

  ROLE = role;
  game_status = GAME_RUNNING;
  game_status_in_string = "GAME_RUNNING";

  /* init the unknwn word */
  NB_LETTERS_IN_WORD = nb_characters;
  number_of_failures = 0;
  number_of_false_word_attempts = 0;
  word_letters_status.resize(NB_LETTERS_IN_WORD, WORD_LETTER_UNKNOWN);

  /* load the language */
  //alphabet_path
  load_alphabet(alphabet_path);
  load_dictionnary(dictionary_path);
  alphabet_letter_is_tried.resize(NB_LETTERS_IN_ALPHABET, false);

  /* initialize random seed: */
  srand(time(NULL));

  if (ROLE == ROLE_FINDER)
    finder_init();
  else
    hanger_init();
}

/*!
 * \brief   destructor
 */
HangmanSolver::~HangmanSolver() {
  ROS_INFO("DESTRUCTOR");
}

/*!
 * \brief   check if the role is really the hanger, send an error otherwise
 *
 * \param   info the name of the function which test this
 *
 * \return  an error if the role is different
 */
void HangmanSolver::assert_role_is_hanger(const std::string & info) {
  if (ROLE != ROLE_HANGER) {
    std::cout << "HangmanSolver::the function '" << info
              << "' can only be called if ROLE==ROLE_HANGER ! Ending";
    exit(-1);
  }
}

/*!
 * \brief   check if the role is really the finder, send an error otherwise
 *
 * \param   info the name of the function which test this
 *
 * \return  an error if the role is different
 */
void HangmanSolver::assert_role_is_finder(const std::string & info) {
  if (ROLE != ROLE_FINDER) {
    std::cout << "HangmanSolver::the function '" << info
              << "' can only be called if ROLE==ROLE_FINDER ! Ending";
    exit(-1);
  }
}

/*!
 * \brief   load the alphabet
 *
 * \param   path the path of the .alphabet file
 */
void HangmanSolver::load_alphabet(const std::string &path) {

  /* parse each line */
  std::ifstream input_file(path.c_str());
  std::string line;

  std::vector<char> found_letters;
  char current_letter;
  std::vector<double> found_probas;
  double current_proba;

  while (input_file.good()) {
    std::getline(input_file, line);
    if (line.length() == 0)
      continue; // empty line
    if (line.at(0) == '/')
      continue; // commentar

    //parse the line
    std::istringstream iss(line);
    iss >> current_letter >> current_proba;
    found_letters.push_back(current_letter);
    found_probas.push_back(current_proba);
    //std::cout << "current_letter:" << current_letter << " - current_proba:" << current_proba << std::endl;
  }

  /* found a different number of letters and probas => error */
  if (found_letters.size() != found_probas.size()) {
    std::cout << "While parsing the alphabet (" << path
              << "), Found a different number of letters ("
              << found_letters.size() << ") and probas ("
              << found_probas.size() << ")" << std::endl;
    exit(-1);
  }

  /* copy the results in the global arrays */
  NB_LETTERS_IN_ALPHABET = found_letters.size();
  // init the available letters
  LETTERS.resize(NB_LETTERS_IN_ALPHABET);
  // init the associated probas
  PROBAS.resize(NB_LETTERS_IN_ALPHABET);
  for (int i = 0; i < NB_LETTERS_IN_ALPHABET; ++i) {
    LETTERS[i] = found_letters.at(i);
    PROBAS[i] = found_probas.at(i);
  }

  ROS_INFO("Alphabet loaded, number of letters:%i", NB_LETTERS_IN_ALPHABET);
}

/*!
 * \brief   load all the possible words in the chosen language
 *
 * Caution : to be faster, no check of double words.
 * To clean a dictionnary from double words
 * and special characters, use the static function 'purge_dictionnary'
 *
 * \param   path the path to the txt file containing all the words
 */
void HangmanSolver::load_dictionnary(const std::string &path) {
  std::ifstream input_file(path.c_str());
  std::string line;

  while (input_file.good()) {
    std::getline(input_file, line);
    // dismiss the words with a bad length
    if (line.length() != NB_LETTERS_IN_WORD)
      continue;
    //std::cout << line.length() << std::endl;
    // keep the survivors
    possible_words.push_back(line);
  }

  ROS_INFO("Words loaded, number of words:%i", (int) possible_words.size() );
}

/*!
 * \brief   declare the game as won by the finder
 */
void HangmanSolver::set_game_won_by_finder() {
  ROS_INFO("Game won by finder !!!");
  game_status = GAME_WON_BY_FINDER;
  game_status_in_string = "GAME_WON_BY_FINDER";
  finder_answer_type = FINDER_ANSWER_GAME_WON_BY_FINDER;
  hanger_answer_type = HANGER_ANSWER_GAME_WON_BY_FINDER;
}

/*!
 * \brief   declare the game as won by the finder
 */
void HangmanSolver::set_game_lost_by_finder() {
  ROS_INFO("Game lost by finder !!!");
  game_status = GAME_LOST_BY_FINDER;
  game_status_in_string = "GAME_LOST_BY_FINDER";
  finder_answer_type = FINDER_ANSWER_GAME_LOST_BY_FINDER;
  hanger_answer_type = HANGER_ANSWER_GAME_LOST_BY_FINDER;
}

/*!
 * \brief   check if the hanger or the finder has won
 */
void HangmanSolver::check_if_is_winner() {
  ROS_INFO("check_if_is_winner()");

  /* check if the finder has found */
  if (get_number_of_unknown_letters() == 0)
    return set_game_won_by_finder();

  /* check if the finder has lost (two many failures ) */
  update_number_of_failures();
  if (number_of_failures > MAX_FAILURES)
    set_game_lost_by_finder();
}

/*!
 * \brief   get the index of a letter
 *
 * \param   c the searched letter
 * \return  the index, -1 if not found
 */
int HangmanSolver::char2idx(char c) {
  std::vector<char>::const_iterator rep = LETTERS.begin();
  for (int i = 0; i < NB_LETTERS_IN_ALPHABET; ++i)
    if (*rep++ == c)
      return i;
  std::cout << "The following char : '" << c << "' is not in the alphabet."
            << " You should call the static procedure"
            << " HangmanSolver::purge_dictionnary() to clean your dict file"
            << " from parasite characters." << " Ending." << std::endl;
  exit(-1);
}

/*!
 * \brief   get a letter according to his letter
 *
 * \param   i the index of the letter
 * \return  the corresponding letter
 */
char HangmanSolver::idx2char(int i) {
  return LETTERS[i];
}

/*!
 * \brief   know if a letter was previously forbidden
 *
 * \param   c the searched letter
 * \return  1 if the letter was previously asked and is not in the word
 */
int HangmanSolver::get_letter_status(char c) {
  return get_letter_status_by_index(char2idx(c));
}

/*!
 * \brief   know if a letter was previously forbidden
 *
 * \param   c_idx the index of the searched letter
 * \return  1 if the letter was previously asked and is not in the word
 */
int HangmanSolver::get_letter_status_by_index(int c_idx) {
  // never tried letter
  if (alphabet_letter_is_tried[c_idx] == 0)
    return ALPHABET_LETTER_NOT_TRIED;

  // look if is in the word
  std::vector<WordLetterStatus>::const_iterator status = word_letters_status.begin();
  for (unsigned int i = 0; i < NB_LETTERS_IN_WORD; ++i) {
    if (*status++ == c_idx)
      return ALPHABET_LETTER_PRESENT;
  }

  // is not present
  return ALPHABET_LETTER_NOT_PRESENT;
}

/*!
 * \brief   know if a letter was previously forbidden
 * \param   c the searched letter
 * \return  1 if the letter was previously asked and is not in the word
 */
bool HangmanSolver::was_letter_forbidden(char c) {
  return (get_letter_status(c) == ALPHABET_LETTER_NOT_PRESENT);
}

/*!
 * \brief   recompute the number of failures
 */
void HangmanSolver::update_number_of_failures() {
  ROS_INFO("update_number_of_failures()");
  number_of_failures = number_of_false_word_attempts;
  for (int i = 0; i < NB_LETTERS_IN_ALPHABET; ++i) {
    if (get_letter_status_by_index(i) == ALPHABET_LETTER_NOT_PRESENT)
      number_of_failures++;
  }
  ROS_INFO("Number of failures:%i", number_of_failures);
}

int HangmanSolver::get_number_of_unknown_letters() {
  ROS_INFO("get_number_of_unknown_letters()");
  int rep = 0;
  std::vector<WordLetterStatus>::const_iterator status = word_letters_status.begin();
  for (unsigned int i = 0; i < NB_LETTERS_IN_WORD; ++i)
    if (*status++ == WORD_LETTER_UNKNOWN)
      rep++;
  ROS_INFO("Number of unknown letters:%i", rep);
  return rep;
}

/*!
 * \brief   know if a letter was previously asked
 * \param   c the searched letter
 * \return  1 if the letter was previously asked
 */
bool HangmanSolver::was_letter_tried(char c) {
  return (get_letter_status(c) != ALPHABET_LETTER_NOT_TRIED);
}

/*!
 * \brief   define a letter as tried by the solver
 *
 * \param   c the tried letter
 */
void HangmanSolver::set_letter_tried(char c) {
  ROS_INFO("setLetterTried(%c)", c);
  alphabet_letter_is_tried[char2idx(c)] = 1;
}

/*!
 * \brief   check if a word is conform to the syntax
 *
 * \param   word the word to check - CAUTION : it has to be of length NB_LETTERS_IN_WORD
 * \return  true if the test is OK
 */
bool HangmanSolver::check_word_conform_to_syntax(const std::string & word) {
  //    std::cout << "***" << word << "***" << std::endl;

  std::string::const_iterator letter = word.begin();
  std::vector<WordLetterStatus>::const_iterator status = word_letters_status.begin();
  int curr_idx;

  for (unsigned int i = 0; i < NB_LETTERS_IN_WORD; ++i) {
    curr_idx = char2idx(*letter);
    //        std::cout << "i:" << i
    //        << " - letter:" << word.at(i)
    //        << " - curr_idx:" << curr_idx << " - word_letters_status[ i ]:"
    //        << word_letters_status[ i ] << std::endl;

    if (*status != curr_idx) {
      /* check for fidelity with the model */
      /* if we know there is a different letter at this place => return false */
      if (*status != WORD_LETTER_UNKNOWN) {
        //std::cout << "case 1" << std::endl;
        return 0;
      }

      /* check for forbidden letters */
      /* if we know the letter was tried and
             * the letter at this position is different => return false */
      if (alphabet_letter_is_tried[curr_idx]) {
        //std::cout << "case 2" << std::endl;
        return 0;
      }
    } // end of test

    ++letter;
    ++status;
  }

  return 1;
}

/*!
 * \brief   check all the possible words
 */
void HangmanSolver::check_possible_words() {
  ROS_INFO("check_possible_words()");

  for (std::vector<std::string>::iterator it = possible_words.begin(); it
       < possible_words.end(); it++) {
    if (!check_word_conform_to_syntax(*it)) {
      possible_words.erase(it);
      it--; // we "rewind" the pointer
      //ROS_INFO("rejected.");
    }
    //        else ROS_INFO("kept.");
  }

  ROS_INFO("After purging, number of words possible:%i",
               get_number_of_possible_words() );
}

/*!
 * \brief   define a letter at a certain position in the unknown word
 *
 * \param   pos the position in the word of the letter (starting at 0)
 * \param   c the letter
 */
void HangmanSolver::set_letter_at_position(unsigned int pos, char c) {
  ROS_INFO("set_letter_at_position(%i, %c)", pos, c);

  if (pos >= NB_LETTERS_IN_WORD) {
    std::cout << "Impossible to set the char '" << c << "' at position " << pos
              << " : the word is only of length " << NB_LETTERS_IN_WORD
              << ".";
    exit(-1);
  }
  word_letters_status[pos] = char2idx(c);
  // declare the char as tried
  // but don't reload the list of possible words
  // (to allow several positions of the char c in the word)
  set_letter_tried(c);
}

/*!
 * \brief   call set_letter_at_position(int pos, char c) for each one of the
 *          positions in the std::vector
 *
 * \param   positions the int giving the positionn of the char
 * \param   c the char in these positions
 */
void HangmanSolver::set_letter_at_positions(const std::vector<unsigned int> & positions, char c) {
  for (std::vector<unsigned int>::const_iterator it = positions.begin();
       it < positions.end(); ++it)
    set_letter_at_position(*it, c);
}

/*!
 * \brief   display all the words still possible
 * Warning, it does not check there has been an update of the list before.
 * Call check_possible_words() if you need it before.
 */
void HangmanSolver::display_all_possible_words() {
  ROS_INFO("display_all_possible_words()");
  ROS_INFO("** Total number of words:%i", get_number_of_possible_words() );
  std::cout << "->";
  for (std::vector<std::string>::iterator it = possible_words.begin(); it
       < possible_words.end(); it++) {
    std::cout << *it << "\t";
  }
  std::cout << std::endl;
}

/*!
 * \brief   get the number of the words still possible
 */
unsigned int HangmanSolver::get_number_of_possible_words() {
  return possible_words.size();
}

/*!
 * \brief   remove a given word (for instance, a wrong word) from the possibles
 *
 * \param   word the word to remove
 */
void HangmanSolver::remove_from_possible_words(const std::string &word) {
  ROS_INFO("remove_from_possible_words('%s')", word.c_str() );

  for (std::vector<std::string>::iterator it = possible_words.begin(); it
       < possible_words.end(); it++) {
    if (*it == word) {
      possible_words.erase(it);
      ROS_INFO("Word '%s' succesfully erased", word.c_str() );
      break;
    }
  }
}

/*!
 * \brief   display the status of the word
 */
void HangmanSolver::display_word_letters_status() {
  std::ostringstream m;
  m << "\t\t";
  /* display word letters status */
  for (unsigned int i = 0; i < NB_LETTERS_IN_WORD; i++) {
    m << " ";
    if (word_letters_status[i] == WORD_LETTER_UNKNOWN)
      m << "_";
    else
      m << idx2char(word_letters_status[i]);
  }

  /* display forbidden letters */
  std::vector<char>::const_iterator c = LETTERS.begin();
  int nb = 0;
  m << "\t\tForbidden:[";
  for (int i = 0; i < NB_LETTERS_IN_ALPHABET; ++i) {
    if (was_letter_forbidden(*c)) {
      if (nb > 0)
        m << ", ";
      m << *c;
      nb++;
    }
    c++;
  }
  m << "]";
  ROS_INFO("m:'%s'", m.str().c_str() );

  /* extensive display */
  //  for (int i = 0; i < 26; ++i) {
  //      char c = (char) ('a' + i);
  //      std::cout << c << " : ";
  //      std::cout << (get_letter_status(c) == ALPHABET_LETTER_PRESENT ? "PRESENT" : "");
  //      std::cout << (get_letter_status(c) == ALPHABET_LETTER_NOT_PRESENT ? "NOT PRESENT" : "");
  //      std::cout << (get_letter_status(c) == ALPHABET_LETTER_NOT_TRIED ? "NOT TRIED" : "");
  //      std::cout << std::endl;
  //  }
}

/*!
 * \brief   init the variables of the finder
 */
void HangmanSolver::finder_init() {
  finder_letter_idx2occurences.resize(NB_LETTERS_IN_ALPHABET);
}

/*!
 * \brief   find the lext guess to do
 */
void HangmanSolver::finder_compute_next_guess() {
  assert_role_is_finder("find_next_guess()");
  ROS_INFO("finder_compute_next_guess()");
  // look for a winner
  check_if_is_winner();
  if (game_status == GAME_WON_BY_FINDER)
    return;
  if (game_status == GAME_LOST_BY_FINDER)
    return;

  //reload the words
  check_possible_words();

  /* nothing possible => finder_answer nothing */
  if (get_number_of_possible_words() == 0)
    return finder_answer_cannot_find();

  /* one possible word => finder_answer it */
  if (get_number_of_possible_words() == 1)
    return finder_answer_word(possible_words.at(0));

  /* if there is only one unknwon letter, take the most probable */
  if (get_number_of_unknown_letters() == 1) {
    std::ostringstream m;
    for (unsigned int i = 0; i < NB_LETTERS_IN_WORD; ++i) {
      if (word_letters_status[i] == WORD_LETTER_UNKNOWN)
        m << finder_compute_next_letter();
      else
        m << idx2char(word_letters_status[i]);
    }
    return finder_answer_word(m.str());
  }

  /* general case : find the most frequent letter in the words left */
  char c = finder_compute_next_letter();
  finder_answer_letter(c);
}

/*!
 * \brief   copy the computed answers into the given args
 *
 * \param   type the type of answer :
 *          FINDER_ANSWER_WORD, FINDER_ANSWER_LETTER or FINDER_ANSWER_CANNOT_FIND
 * \param   answer the answer itself, respectively :
 *          the answered word, ""+the answered char, ""
 */
void HangmanSolver::finder_get_answer(FinderAnswer & type,
                                      std::string & answer) {
  assert_role_is_finder("finder_get_answer(int* type, std::string* answer)");
  type = finder_answer_type;
  answer = finder_answer;
}

/*!
 * \brief   find the letter which would be the most popular
 * in the possible words
 */
char HangmanSolver::finder_compute_next_letter() {
  assert_role_is_finder("finder_compute_next_letter()");
  ROS_INFO("finder_compute_next_letter()");

  /* compute the finder_occurences */
  for (int i = 0; i < NB_LETTERS_IN_ALPHABET; ++i)
    finder_letter_idx2occurences[i] = 0;

  for (std::vector<std::string>::iterator it = possible_words.begin(); it
       < possible_words.end(); it++) {
    std::vector<WordLetterStatus>::const_iterator curr_letter = word_letters_status.begin();
    for (std::string::iterator letter = it->begin(); letter < it->end(); ++letter) {
      if (*curr_letter == WORD_LETTER_UNKNOWN) {
        //std::cout << "letter:" << *letter << " - " << get_letter_number_from_char(*letter) << std::endl;
        /* add 1 to the finder_occurences of the letter index */
        ++finder_letter_idx2occurences[char2idx(*letter)];
      }
      ++curr_letter;
    }
  }

  /* fint the one with the max occurence */
  int max = -1, index = 0;
  std::ostringstream display;
  display << "Occurences: ";
  for (int i = 0; i < NB_LETTERS_IN_ALPHABET; ++i) {
    display << idx2char(i) << ":"
            << finder_letter_idx2occurences[i] << "\t";
    if ((finder_letter_idx2occurences[i] > max) || (finder_letter_idx2occurences[i] == max
                                                    && PROBAS[i] > PROBAS[index])) {
      max = finder_letter_idx2occurences[i];
      index = i;
    }
  }
  ROS_INFO("display:'%s'", display.str().c_str() );
  //ROS_INFO( "index of the answered letter:", index );

  /* and return it */
  return idx2char(index);
}

/*!
 * \brief   finder_answer a letter to the human player
 *
 * \param   c the letter answered
 */
void HangmanSolver::finder_answer_letter(char c) {
  assert_role_is_finder("answer_letter()");

  ROS_INFO("Answering a letter:'%c'", c );
  finder_answer_type = FINDER_ANSWER_LETTER;
  finder_answer = c;
}

/*!
 * \brief   finder_answer a word to the human player
 *
 * \param   word the answered word
 */
void HangmanSolver::finder_answer_word(const std::string &word) {
  assert_role_is_finder("answer_word()");

  ROS_INFO("Answering a word:'%s'", word.c_str() );
  finder_answer_type = FINDER_ANSWER_WORD;
  finder_answer = word;
}

/*!
 * \brief   finder_answer to the human player that
 *          the algorithm cant find the finder_answer
 */
void HangmanSolver::finder_answer_cannot_find() {
  assert_role_is_finder("answer_cannot_find()");

  ROS_INFO("Answering that the algorithm cannot find");
  finder_answer_type = FINDER_ANSWER_CANNOT_FIND;
  finder_answer = "";

  game_status_in_string = "FINDER_ANSWER_CANNOT_FIND";
  hanger_answer_type = HANGER_ANSWER_GAME_LOST_BY_FINDER;
  game_status = GAME_LOST_BY_FINDER;

  // lose the game
  //set_game_lost_by_finder();
}

/*!
 * \brief   init the different variables
 * and computes the unknown word
 */
void HangmanSolver::hanger_init() {
  assert_role_is_hanger("hanger_init()");
  int word_index = rand() % get_number_of_possible_words();
  hanger_word = possible_words.at(word_index);
  hanger_answer_position.clear();

  //hanger_word = "converge";

  ROS_INFO("The unknown word to guess is :'%s'", hanger_word.c_str() );
}

/*!
 * \brief   manage the data coming from the hanger
 *
 * \param  type a value among HANGER_ANSWER_LETTER_FOUND ,
 * HANGER_ANSWER_LETTER_NOT_FOUND , HANGER_ANSWER_WORD_WRONG
 * GAME_WON_BY_FINDER , GAME_LOST_BY_FINDER
 * \param answer respectively, the std::vector of the positions of the suggested letter,
 * an empty std::vector and an empty std::vector
 */
void HangmanSolver::finder_receive_data_from_hanger(HangerAnswer & type,
                                                    std::vector<unsigned int> & answer) {
  if (type == HANGER_ANSWER_NEW_GAME) {
    // nothing to do
  }
  else if (type== HANGER_ANSWER_GAME_WON_BY_FINDER) {
    set_game_won_by_finder();
  } else if (type== HANGER_ANSWER_GAME_LOST_BY_FINDER) {
    set_game_lost_by_finder();
  } else if (type== HANGER_ANSWER_LETTER_FOUND) {
    set_letter_at_positions(answer, finder_answer.at(0));
  } else if (type== HANGER_ANSWER_LETTER_NOT_FOUND) {
    number_of_failures++;
    set_letter_tried(finder_answer.at(0));
  } else if (type== HANGER_ANSWER_WORD_WRONG) {
    number_of_failures++;
    remove_from_possible_words(finder_answer);
  }
}

/*!
 * \brief   receive a suggestion of word from the finder
 * \param   word the suggested word
 */
void HangmanSolver::hanger_get_word_suggestion(const std::string & word) {
  ROS_INFO("hanger_get_word_suggestion('%s')", word.c_str() );
  assert_role_is_hanger("hanger_get_word_suggestion()");
  hanger_answer_position.clear();

  // word found => won !
  if (word == hanger_word)
    set_game_won_by_finder();
  else {// failure++
    number_of_failures++;
    hanger_answer_type = HANGER_ANSWER_WORD_WRONG;
    check_if_is_winner();
  }
  }

/*!
 * \brief   receive a suggestion of character from the finder
 * \param   c the suggested character
 */
void HangmanSolver::hanger_get_letter_suggestion(char c) {
  ROS_INFO("hanger_get_letter_suggestion('%c')", c );
  assert_role_is_hanger("hanger_get_letter_suggestion()");
  hanger_answer_position.clear();

  for (unsigned int i = 0; i < NB_LETTERS_IN_WORD; ++i) {
    // letter in position i known => skip
    if (word_letters_status[i] != WORD_LETTER_UNKNOWN)
      continue;

    // letter in position i == c => add to the results
    if (hanger_word.at(i) == c) {
      set_letter_at_position(i, c);
      hanger_answer_position.push_back(i);
    }
  }

  // the letter was not present :
  if (hanger_answer_position.size() == 0) {
    hanger_answer_letter_not_present();
    set_letter_tried(c);
  }
  // the letter was present at least once :
  else
    hanger_answer_letter_found();

  check_if_is_winner();
}

/*!
 * \brief   answer that the suggested letter has been found
 */
void HangmanSolver::hanger_answer_letter_found() {
  assert_role_is_hanger("hanger_answer_letter_found()");
  hanger_answer_type = HANGER_ANSWER_LETTER_FOUND;
  std::ostringstream m;
  m << "Answering that the letter has been found in positions :";
  for (unsigned int i = 0; i < hanger_answer_position.size(); ++i)
    m << hanger_answer_position.at(i) << ", ";
  ROS_INFO("m:'%s'", m.str().c_str() );
}

/*!
 * \brief   answer that the suggested letter has NOT been found
 */
void HangmanSolver::hanger_answer_letter_not_present() {
  assert_role_is_hanger("hanger_answer_letter_not_present()");
  hanger_answer_type = HANGER_ANSWER_LETTER_NOT_FOUND;
  ROS_INFO("Answering that the letter has not been found.");
}

/*!
 * \brief   manage the data coming from the hanger
 *
 * \param  type a value among HANGER_ANSWER_LETTER_FOUND ,
 * HANGER_ANSWER_LETTER_NOT_FOUND , HANGER_ANSWER_WORD_WRONG
 * GAME_WON_BY_FINDER , GAME_LOST_BY_FINDER
 * \param answer respectively, the std::vector of the positions of the suggested letter,
 * an empty std::vector and an empty std::vector
 */
void HangmanSolver::hanger_get_answer(HangerAnswer & type,
                                      std::vector<int> & answer) {
  assert_role_is_hanger("hanger_answer_letter_found()");
  type= hanger_answer_type;
  answer = hanger_answer_position;
}

/*!
 * \brief   manage the data coming from the finder
 *
 * \param  type a value among FINDER_ANSWER_WORD, FINDER_ANSWER_LETTER ,FINDER_ANSWER_CANNOT_FIND
 * GAME_WON_BY_FINDER , GAME_LOST_BY_FINDER
 * \param answer respectively, the std::vector of the positions of the suggested letter,
 * an empty std::vector and an empty std::vector
 */
void HangmanSolver::hanger_receive_data_from_finder(FinderAnswer & type,
                                                    const std::string & answer) {
  if (type== FINDER_ANSWER_GAME_WON_BY_FINDER) {
    set_game_won_by_finder();
  } else if (type== FINDER_ANSWER_GAME_LOST_BY_FINDER) {
    set_game_lost_by_finder();
  } else if (type== FINDER_ANSWER_WORD) {
    hanger_get_word_suggestion(answer);
  } else if (type== FINDER_ANSWER_LETTER) {
    hanger_get_letter_suggestion(answer.at(0));
  } else if (type== FINDER_ANSWER_CANNOT_FIND) {
    set_game_lost_by_finder();
  }
}

/*!
 * \brief   check that each letter in a dictionnary is between 'a' and 'z'.
 * Convert the letters between 'A' and 'Z' to the lower case.
 * Remove the other things.
 * Nothing is modified in the input file, these modifs are made in the output file.
 *
 * \param   path the path to the dictionnary.
 * \param   path_out the path to the output dictionnary.
 */
void HangmanSolver::purge_dictionnary(const std::string &path, const std::string &path_out) {
  std::cout << "*** Purging the dictionnary '" << path << "' ***" << std::endl;
  std::cout << "*** Output file :'" << path_out << "' *** " << std::endl;

  std::ifstream input_file(path.c_str());
  std::ofstream output_file(path_out.c_str());
  std::string line, line_out, last_line_out = "";
  std::ostringstream line_out_stream;

  while (input_file.good()) {
    std::getline(input_file, line);
    line_out_stream.str("");

    for (std::string::iterator letter = line.begin(); letter < line.end(); ++letter) {
      // lower case
      if (*letter >= 'a' && *letter <= 'z')
        line_out_stream << *letter;

      // upper case => case conversion
      else if (*letter >= 'A' && *letter <= 'Z')
        line_out_stream << (char) (*letter - 'A' + 'a');

      // other chars
      else
        continue;
    }

    line_out = line_out_stream.str();
    // line not different from the last one written => skip
    if (line_out == last_line_out) {
      std::cout << "Word in double:" << last_line_out << std::endl;
      continue;
    }
    if (line != line_out)
      std::cout << line << " \t->\t " << line_out << std::endl;
    output_file << line_out << std::endl;
    last_line_out = line_out;
  }
}
