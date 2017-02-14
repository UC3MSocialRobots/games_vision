#ifndef HANGMAN_SKILL_H
#define HANGMAN_SKILL_H

/***************************************************************************//**
 * \class CHangMan
 *
 * \brief A skill to play to the game Hangman ( ahorcado ) with the camera
 *
 * It uses the skill PlayzoneFindSkill and the HangmanSolver (AI of the gane)
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 08/06/2009
 *******************************************************************************/

///// my imports
#include "games_vision/image_comparer.h"
#include "games_vision/playzone_sequential_user.h"
#include "games_vision/HangmanSolver.h"

class CHangMan: public PlayzoneSequentialUser {
public:
  typedef std::string LanguagePrefix;

  static const double LETTER_RECOG_THRES = 2; // 1.5;

  static const int CASE_UNKNOWN = -1;
  static const int CASE_EMPTY = 1;
  static const int CASE_CROSS = 2;
  static const int CASE_ROUND = 3;
  static const int LEVEL_NOVICE = 1;
  static const int LEVEL_INTERMEDIATE = 2;
  static const int LEVEL_ADVANCED = 3;
  static const int LEVEL_EXPERT = 4;

  /** constructor */
  CHangMan();
  /** destructor */
  ~CHangMan();

  void display();

private:
  /////
  ///// general functions
  /////
  /** main loop of the skill */
  bool process_pz(const cv::Mat3b & pz);
  /*! the function that will be called before trying to get the playzone */
  void do_stuff_before_get_playzone();
  /*! the function that will be called after trying to get the playzone */
  void do_stuff_after_get_playzone(bool was_find_and_process_success);

  /** lauch the skill */
  void create_subscribers_and_publishers_playzone();

  /** terminate the skill */
  void shutdown_subscribers_and_publishers_playzone();

  /////
  ///// general fields
  /////
  //! the monochrome version of the acquired frame
  cv::Mat1b playzone_monochrome;

  /////
  ///// parameters
  /////
  //! \brief   return a string including the char and its international alphabet
  std::string say_letter(const char c,
                         const LanguagePrefix language_idx) const;

  /** the results */
  void sayResult(int method, double t);

  /////
  ///// play part
  /////
  /*! recognize the grid - returns false if no success */
  bool analyse_play_zone(const cv::Mat3b &pz);

  void get_components();

  //! the points of the connected components
  std::vector<std::vector<cv::Point> > components_points;

  //! the bounding boxes of the connected components
  std::vector<cv::Rect> bboxes;

  /*!
     * \brief   find only the dashes among the components
     * Remove the outliers dashes (the ones far from the median y)
     */
  bool find_dashes();

  /*! an array of size the number of components.
      Is true when the corresponding component is with no letter on top */
  std::vector<bool> is_comp_free;

  /*!
     * \brief   now we will look for the affectations dash <-> letter
     */
  void find_letter_components_above_dashes();

  std::vector<std::pair<int, int> > pairs_dash_letter;

  void set_letter_is_forbidden(char c, bool is_forbidden);


  /*!
     * \brief   analyse the drawing of each letter and
     * find the letter it represents
     */
  bool recognize_letters();

  //! the comparer between the monochrome letter points and the real letters
  ImageComparer comparer;

  //! the model_mask of the forbidden letters
  std::vector<bool> letters_forbidden;

  std::vector<char> found_symbols;

  ///// game
  HangmanSolver hang_solver;

  /*!
     * \brief   submit the data to the hangman solver
     */
  void send_data_to_solver();

  //! interpret the answer of the solver
  void analyze_solver_answer();
  HangmanSolver::FinderAnswer last_robot_answer_type;
  std::string last_robot_answer_string;

  //////
  ////// static functions
  //////
  /*!
     * \brief   generates all the pairs_dash_letter of the alphabet in images
     *
     * \param   width the width of the output images
     * \param   height the height of the output images
     */
  static void generate_alphabet(int width, int height);

  /*!
     * \brief   compute the score that a component is the letter
     * associated with an dash
     *
     * \param   dash the pointer to the bbox of the dash
     * \param   letter the pointer to the bbox of the letter
     * \return  the score
     */
  static double match_bboxes(cv::Rect* dash, cv::Rect* letter);

};

static std::string international_alphabet[26] =
{
  "Alpha", "Bravo", "Charlie", "Delta",  "Eko",  "Foxtrot", "Golf", //
  "Hotel", "India", "Juliet", "Kilo", "Lima", "Mike", "November", //
  "Oscar", "Papa", "Quebec", "Romeo", "Sierra", "Tango", "Uniform", //
  "Victor", "Whiskey", "Sylophono", //"X-ray",
  "Yankee", "Zulu"
};

static std::string letters_pronunciation[26] =
{
  "aa", "be", "ce", "de", "ee",
  "heffe", "ge", "hache", "ii", "jota", "ka", "hele", "heme", "hene",
  "oo", "pe", "ku", "erre", "ese", "te", "uu", "uve", "doble uve",
  "ekis", "i griega", "ceta"
};

#endif

