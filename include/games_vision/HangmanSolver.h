#ifndef HANGMANSOLVER_H_
#define HANGMANSOLVER_H_

/***************************************************************************//**
 * \class HangmanSolver
 *
 * \brief this class allows to solve a hangman ( ahorcado ) problem
 *
 * It has no links with the vision or Maggie, it is just
 * called from the skill Hangman.
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 04/06/2009
 *******************************************************************************/

////// maggie includes

///// STL imports
#include <stdio.h>
#include <stdlib.h>         // for exit
#include <iostream>         // for cin, cout
#include <string>           // for strings
#include <sstream>          // for std::sstreams
#include <fstream>          // for file reading
#include <vector>           // for vectors

#include <vision_utils/img_path.h>
#define HANGMAN_DIR          vision_utils::IMG_DIR() + "hangman/"
#define PATH_ALPHABET_EN     HANGMAN_DIR "english.alphabet"
#define PATH_DICTIONNARY_EN  HANGMAN_DIR "english.dict"
#define PATH_ALPHABET_ES     HANGMAN_DIR "spanish.alphabet"
#define PATH_DICTIONNARY_ES  HANGMAN_DIR "spanish.dict"

class HangmanSolver {
public:
  typedef unsigned int LetterPosition;
  static const unsigned int MAX_FAILURES  = 5;

  enum Role { ROLE_FINDER = 100,
              ROLE_HANGER = 200 };
  typedef char WordLetterStatus;
  static const WordLetterStatus WORD_LETTER_UNKNOWN = 'c';
  enum AlphabetLetterStatus { ALPHABET_LETTER_NOT_TRIED,
                              ALPHABET_LETTER_PRESENT,
                              ALPHABET_LETTER_NOT_PRESENT };
  enum GameStatus { GAME_RUNNING,
                    GAME_WON_BY_FINDER,
                    GAME_LOST_BY_FINDER };
  enum FinderAnswer { FINDER_ANSWER_GAME_WON_BY_FINDER,
                      FINDER_ANSWER_GAME_LOST_BY_FINDER,
                      FINDER_ANSWER_NEW_GAME,
                      FINDER_ANSWER_WORD,
                      FINDER_ANSWER_LETTER,
                      FINDER_ANSWER_CANNOT_FIND };
  enum HangerAnswer { HANGER_ANSWER_GAME_WON_BY_FINDER,
                      HANGER_ANSWER_GAME_LOST_BY_FINDER,
                      HANGER_ANSWER_NEW_GAME,
                      HANGER_ANSWER_WORD_WRONG,
                      HANGER_ANSWER_LETTER_FOUND,
                      HANGER_ANSWER_LETTER_NOT_FOUND };

  HangmanSolver();
  ~HangmanSolver();

  void init(Role role,
            int nb_characters,
            const std::string & alphabet_path,
            const std::string & dictionary_path);

  /* loading functions */
  void load_alphabet(const std::string & path);
  void load_dictionnary(const std::string & path);

  /* get some info about the solver */
  inline GameStatus get_game_status() const { return game_status; }
  unsigned int get_number_of_possible_words();
  void display_all_possible_words();
  void display_word_letters_status();
  WordLetterStatus get_word_letter_status(unsigned int pos_in_word) const
  { return word_letters_status[pos_in_word]; }
  unsigned int get_number_of_letters_in_word() const { return NB_LETTERS_IN_WORD; }
  unsigned int get_number_of_failures() const { return number_of_failures; }

  /* set some information about the game */
  void set_letter_at_position(unsigned int pos, char c);
  void set_letter_at_positions(const std::vector<unsigned int> &positions, char c);
  void set_letter_tried(char c);

  /* interact when the robot is a finder */
  void finder_receive_data_from_hanger(HangerAnswer &type, std::vector<unsigned int> &answer);
  void finder_compute_next_guess();
  void finder_get_answer(FinderAnswer &type, std::string &answer);

protected:
  /* check the fidelity between the available words and the model */
  void check_if_is_winner();
  void set_game_won_by_finder();
  void set_game_lost_by_finder();
  void update_number_of_failures();
  bool check_word_conform_to_syntax(const std::string & word);
  void check_possible_words();

  int char2idx(char c);
  char idx2char(int i);
  int get_letter_status(char c);
  int get_letter_status_by_index(int c_idx);
  int get_number_of_unknown_letters();
  bool was_letter_tried(char c);
  bool was_letter_forbidden(char c);
  void remove_from_possible_words(const std::string & word);

  /* check the good role */
  void assert_role_is_hanger(const std::string & info);
  void assert_role_is_finder(const std::string & info);

  /* communication of the results of the hanger */
  void finder_init();
  char finder_compute_next_letter();
  void finder_answer_letter(char c);
  void finder_answer_word(const std::string & word);
  void finder_answer_cannot_find();

  /* get the suggestions of the finder */
  void hanger_init();
  void hanger_get_word_suggestion(const std::string &word);
  void hanger_get_letter_suggestion(char c);
  void hanger_answer_letter_found();
  void hanger_answer_letter_not_present();
  void hanger_get_answer(HangerAnswer &type, std::vector<int> &answer);
  void hanger_receive_data_from_finder(FinderAnswer &type, const std::string &answer);

  //////
  ////// static functions
  //////
  static void purge_dictionnary(const std::string & path,
                                const std::string & path_out);

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  Role ROLE;

  /* info on the alphabet */
  //! the number of letters in the alphabet (normally, 26)
  int NB_LETTERS_IN_ALPHABET;
  //! the list of letters in the alphabet (normtype filter textally, a, b, c...)
  std::vector<char> LETTERS;
  //!< the average finder_occurences of each letter in the language
  std::vector<double> PROBAS;
  //!< the list of all the possible words left
  std::vector<std::string> possible_words;

  /* infos on the word */
  //! the number of characters in the unknown word
  unsigned int NB_LETTERS_IN_WORD;
  //! indicates the status of each letter of the unknown word
  std::vector<WordLetterStatus> word_letters_status;
  //! the status of each letter of the alphabet (1 = tried)
  std::vector<bool> alphabet_letter_is_tried;

  /* status of the game */
  //! among : GAME_RUNNING, GAME_WON_BY_FINDER, GAME_LOST_BY_FINDER
  GameStatus game_status;
  //! among : "GAME_RUNNING", "GAME_WON_BY_FINDER", "GAME_LOST_BY_FINDER"
  std::string game_status_in_string;
  //! the number of errors made by the finder
  unsigned int number_of_failures;
  unsigned int number_of_false_word_attempts;

  /////
  ///// FINDER
  /////
  //! the occurences of each letter in the remaining words
  std::vector<int> finder_letter_idx2occurences;
  /*! the type of answer : FINDER_ANSWER_WORD, FINDER_ANSWER_LETTER ,FINDER_ANSWER_CANNOT_FIND
     GAME_WON_BY_FINDER or GAME_LOST_BY_FINDER */
  FinderAnswer finder_answer_type;
  //! the answer itself, respectively : the answered word, ""+the answered char, "", "", ""
  std::string finder_answer;

  /////
  ///// HANGER
  /////
  //! the word to guess
  std::string hanger_word;
  /*! a value among HANGER_ANSWER_LETTER_FOUND,
     HANGER_ANSWER_LETTER_NOT_FOUND or HANGER_ANSWER_WORD_WRONG */
  HangerAnswer hanger_answer_type;
  /*! respectively, the vector of the positions of the suggested letter,
     an empty vector and an empty vector */
  std::vector<int> hanger_answer_position;
};

#endif /*HANGMANSOLVER_H_*/

