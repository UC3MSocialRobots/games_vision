/***************************************************************************//**
 * \file CHangMan.cpp
 *
 * \brief The implementation of CHangMan.h
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 26/05/2009
 *******************************************************************************/

#include "games_vision/CHangMan.h"
#include "vision_utils/connectedcomponents2.h"
#include "vision_utils/bboxes_included.h"
#include "vision_utils/redimContent.h"
#include "vision_utils/color_utils.h"

////////////////////////////////////////////////////////////////////////////////

CHangMan::CHangMan() :
  PlayzoneSequentialUser("HANGMAN_START", "HANGMAN_STOP") {
  ROS_INFO("constructor");
  _win1_name = "CHangMan - acq";
  _win2_name = "CHangMan - mod";
}

////////////////////////////////////////////////////////////////////////////////

CHangMan::~CHangMan() {
  ROS_INFO("destructor");
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::create_subscribers_and_publishers_playzone() {
  ROS_INFO("create_subscribers_and_publishers_playzone()");

  say_text("|es:Estoy lista para jugar al ahorcado. "
           "Piensa en la palabra que quieres. "
           "Haz un dibujo con un guion para cada letra. "

           "|en:Let us play hangman. "
           "Think about the word you want. "
           "Then draw a figure with a dash for each letter.");

  // configure an eventual drawer - make a nice background
  _playzone_modified_for_drawer.create(300, 300);
  _playzone_modified_for_drawer = cv::Vec3b(230, 230, 230);

  /* creating the playzones */
  playzone_monochrome.create(1, 1);

  /* init the other things */
  /* reset the letter model_mask */
  letters_forbidden.resize(26, false);
  last_robot_answer_type = HangmanSolver::FINDER_ANSWER_NEW_GAME;
  last_robot_answer_string = "";
  srand(time(NULL)); // init random numbers
  comparer.set_models(HANGMAN_DIR "symbols/index.txt", cv::Size(32, 32));
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::shutdown_subscribers_and_publishers_playzone() {
  ROS_INFO("shutdown_subscribers_and_publishers_playzone()");
  shut_up();
  say_text("|es:Basta con el ahorcado, hacemos otra cosa? "
           "|en:Let us stop playing hangman, what do we do now? ");

}

////////////////////////////////////////////////////////////////////////////////

bool CHangMan::process_pz(const cv::Mat3b & pz) {
  ROS_INFO("process_pz(const cv::Mat3b & pz)");
  bool analysis_OK = analyse_play_zone(pz);
  if (analysis_OK == false)
    return false;
  send_data_to_solver();
    analyze_solver_answer();
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::do_stuff_before_get_playzone() {
  ROS_INFO("do_stuff_before_get_playzone()");
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::do_stuff_after_get_playzone(bool was_find_and_process_success) {
  ROS_INFO("do_stuff_after_get_playzone(success:%i)", was_find_and_process_success);
}

////////////////////////////////////////////////////////////////////////////////

std::string CHangMan::say_letter(const char c,
                                 const LanguagePrefix language_idx) const {
  int idx = (int) c - (int) 'a';
  std::ostringstream rep;
  if (language_idx == "en")
    rep << "\"" << letters_pronunciation[idx] << "\""
        << ", like in \"" << international_alphabet[idx] << "\"";
  else if (language_idx == "es")
    rep << "\"" << letters_pronunciation[idx] << "\""
        << ", como \"" << international_alphabet[idx] << "\"";
  return rep.str();
}

////////////////////////////////////////////////////////////////////////////////

bool CHangMan::analyse_play_zone(const cv::Mat3b & pz) {
  ROS_INFO("analyse_play_zone()");

  //Timer t;
  cv::cvtColor(pz, playzone_monochrome, CV_RGB2GRAY);
  cv::threshold(playzone_monochrome, playzone_monochrome, 128, 255, CV_THRESH_BINARY_INV);

  /* start analysis */
  //t.printTime("at the beginning of analysis");
  bool recognitionOK = true;

  /* find every connected component */
  get_components();

  /* find only the components which are dashes of the game */
  recognitionOK = find_dashes();
  if (!recognitionOK)
    return false;

  /* init the solver if needed */
  unsigned int nb_letters = pairs_dash_letter.size();
  if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_NEW_GAME) {
    // init the HangmanSolver
    std::string path_alphabet = "", path_dict = "";
    std::string domain = "en";
    _nh_public.param("language", domain, domain);
    //Subscribe to tts topic
    if (domain == "en") {
      path_alphabet = PATH_ALPHABET_EN;
      path_dict = PATH_DICTIONNARY_EN;
    } else if (domain == "es") {
      path_alphabet = PATH_ALPHABET_ES;
      path_dict = PATH_DICTIONNARY_ES;
    }
    else {
      printf("\n/!\\ CHangMan: no dictionnary for language '%s', loading english!",
             domain.c_str());
      path_alphabet = PATH_ALPHABET_EN;
      path_dict = PATH_DICTIONNARY_EN;
    }

    hang_solver .init(HangmanSolver::ROLE_FINDER,
                      nb_letters,
                      path_alphabet, path_dict);
    std::ostringstream sentence;
    sentence << "|en:The word has " << nb_letters << " letters.";
    sentence << "|es:La palabra tiene " << nb_letters << " lettras.";
    say_text(sentence.str());
  }
  /* solver initiated  && nb of letters not corresponding => restart */
  else if (hang_solver.get_number_of_letters_in_word()  != nb_letters) {
    ROS_WARN("solver initiated with %i letters && here %i letters => restart",
                 hang_solver.get_number_of_letters_in_word(), nb_letters);
    recognitionOK = 0;
  }

  if (!recognitionOK)
    return false;

  /* find the component above dashes, which correspond to letters */
  find_letter_components_above_dashes();
  if (!recognitionOK)
    return false;

  /* recognize the letters */
  recognitionOK = recognize_letters();
  if (!recognitionOK)
    return false;

  //t.printTime("at the end of analysis");
  //tts.sayText("Good", "Bien.");
  //cv::imshow(window2Name, playzone_monochrome);
  //cv::waitKey( 50000 );
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::get_components() {
  ROS_INFO("get_components()");

  //cvCopy(playzoneFinder->get_playzone_monochrome(), playzone_monochrome);
#ifdef SAVE_IMAGES
  cv::imwrite("300-playzone_monochrome.png", playzone_monochrome);
#endif

  /* delete the margins */
  double LEFT_MARGIN = 3 / 100.f;
  double UP_MARGIN = 3 / 100.f;

  int xMin = (int) (LEFT_MARGIN * playzone_monochrome.cols);
  int xMax = (int) ((1.f - LEFT_MARGIN) * playzone_monochrome.cols);
  int yMin = (int) (UP_MARGIN * playzone_monochrome.rows);
  int yMax = (int) ((1.f - UP_MARGIN) * playzone_monochrome.rows);
  /* reset the ROI */
  cv::Mat1b playzone_monochrome_no_border;
  playzone_monochrome(cv::Rect(xMin, yMin, xMax - xMin, yMax - yMin)).
      copyTo(playzone_monochrome_no_border);

  /* get the components_points */
  components_points.clear();
  bboxes.clear();
  vision_utils::connectedComponents2(playzone_monochrome_no_border,
                                    components_points, bboxes);
  ROS_INFO("Number of components:%i", (int) components_points.size() );

  // add the shift induced by the border
  for (unsigned int comp_idx = 0; comp_idx < components_points.size(); ++comp_idx) {
    // add to the component points
    std::vector<cv::Point> comp = components_points.at(comp_idx);
    for ( std::vector<cv::Point>::iterator pt = comp.begin();
          pt != comp.end() ; ++pt) {
      pt->x += xMin;
      pt->y += yMin;
    }
    // add to the bounding box
    bboxes.at(comp_idx).x += xMin;
    bboxes.at(comp_idx).y += yMin;
  } // end loop  comp_idx

  /* reset the ROI */
  //cvResetImageROI(playzone_monochrome);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * \class Comparer_pairs
 * \brief a comparison object between pairs of int,
 * each one representing the indices of bounding boxes of
 * two associated connected components.
 *
 * To be clear :
 * - s is pointer toward a  std::vector of rectangles
 * - let p1 and p2 be two objects we want to compare
 * p1 contains two integers, p1.first and p1.second.
 * It means p1 represent the association of rectangles ( s(p1.first), s(p1.second))
 * - The comparer will compare s(p1.first) and s(p2.first) .
 * - It will return true if the first one is more on the left than the second one.
 *
 * The  std::vector of bounding boxes is passed by value.
 */
class Comparer_pairs {
public:
  std::vector<cv::Rect>* s;
  /* constructor */
  Comparer_pairs(std::vector<cv::Rect>* s_) :
    s(s_) {
  }
  ;
  bool operator()(const std::pair<int, int> p1, const std::pair<int, int> p2) const {
    return s->at(p1.first).x < s->at(p2.first).x;
  } // returns x>y if s>0, else x<y
};

////////////////////////////////////////////////////////////////////////////////

bool CHangMan::find_dashes() {
  ROS_INFO("find_dashes()");

  std::vector< std::vector<cv::Point> >::iterator current_component;
  std::vector<cv::Rect>::iterator current_bbox;

  // the ordonnates of the components, sorted
  std::vector<int> ordonnate;

  /* find the dashes */
  pairs_dash_letter.clear();
  is_comp_free.resize(components_points.size());
  current_bbox = bboxes.begin();
  current_component = components_points.begin();

  for (unsigned int idx = 0; idx < components_points.size(); ++idx) {
    bool is_a_comp = 1;

    /* too little => skip */
    if (current_component->size() < 30)
      is_a_comp = 0;

    /* too narrow => skip */
    if (current_bbox->width < 15)
      is_a_comp = 0;

    /* don't have the shape of an dash => skip */
    if (current_bbox->width < 2. * current_bbox->height)
      is_a_comp = 0;

    if (is_a_comp) {
      pairs_dash_letter.push_back(std::pair<int, int>(idx, -1));
      is_comp_free.at(idx) = 0;
      ordonnate.push_back(current_bbox->y + current_bbox->height / 2);
    } else
      is_comp_free.at(idx) = 1;

    ++current_bbox;
    ++current_component;
  }

  ROS_INFO("Number of dashes with outliers:%i",
               (int) pairs_dash_letter.size() );
  // zero dashes found => return false
  if (pairs_dash_letter.size() == 0) {
    ROS_WARN("Before removing outliers, "
                 "pairs_dash_letter == 0, find_dashes() fail!" );
    return false;
  }

#ifdef PZ_FIND_IMAGES
  /* draw all the dashes */
  cv::Mat3b dashes_illus (playzone.size());
  cv::cvtColor(playzone_monochrome, dashes_illus, CV_GRAY2RGB);

  int i = 0;
  for ( std::vector < std::pair<int, int> >::iterator it =
        pairs_dash_letter.begin(); it < pairs_dash_letter.end(); it++) {
    cv::Rect bb = bboxes.at((*it).first);
    vision_utils::drawRectangle(dashes_illus, bb,
                               vision_utils::color_scalar<cv::Scalar>(i, pairs_dash_letter.size()), 2);
    ++i;
  }

  cv::imwrite("301-dashes_illus.png", dashes_illus);
#endif

  /* eliminate the outliers */
  sort(ordonnate.begin(), ordonnate.end());
  int average = ordonnate.at(ordonnate.size() / 2);
  cv::Rect* bb;

  for (std::vector<std::pair<int, int> >::iterator it =
       pairs_dash_letter.begin(); it < pairs_dash_letter.end(); it++) {
    bb = &bboxes.at((*it).first);
    if (abs(bb->y + bb->height / 2 - average) > 30) {
      pairs_dash_letter.erase(it);
      it--; // we "rewind" the pointer
    }
  }

  ROS_INFO("Number of dashes without outliers:%i",
               (int) pairs_dash_letter.size() );
  // zero dashes found => return false
  if (pairs_dash_letter.size() == 0) {
    ROS_WARN("After removing outliers, "
                 "pairs_dash_letter == 0, find_dashes() fail!" );
    return false;
  }

  /* sort the associatons according to y */
  sort(pairs_dash_letter.begin(),
       pairs_dash_letter.end(),
       Comparer_pairs(&bboxes));

  return true;
}

////////////////////////////////////////////////////////////////////////////////

double CHangMan::match_bboxes(cv::Rect* dash, cv::Rect* letter) {
  ROS_INFO("match_bboxes()");
  double score = 0;
  // if the letter is under the dash, return INF
  if (letter->y + letter->height > dash->y + dash->height)
    return INFINITY;

  // the yMax of the letter must be close from the yMin of the component
  score += dash->y - (letter->y + letter->height);

  // the centers must be alignated
  score += dash->x + dash->width / 2 - (letter->x + letter->width
                                        / 2);
  return score;
}

////////////////////////////////////////////////////////////////////////////////

//! ratio of the width of a dash that will be used to widen it
#define DASH_WIDTH_BBOX_RATIO  .5f

void CHangMan::find_letter_components_above_dashes() {
  ROS_INFO("find_letter_components_above_dashes()");

  std::vector<cv::Rect>::iterator current_bbox_dash;
  std::vector<cv::Rect>::iterator current_bbox_letter;
  std::vector< std::vector<cv::Point> >::iterator current_points_letter;

  cv::Rect current_box_above_dash;

  double xMin, xMax, yMin, yMax, curr_score, best_score;
  int best_index;

  for (unsigned int unders_idx = 0; unders_idx
       < pairs_dash_letter.size(); ++unders_idx) {
    current_bbox_dash = bboxes.begin() + pairs_dash_letter.at(
          unders_idx).first;

    // compute a bbox above the dash
    xMin = current_bbox_dash->x - DASH_WIDTH_BBOX_RATIO * current_bbox_dash->width;
    xMax = current_bbox_dash->x + (1 + DASH_WIDTH_BBOX_RATIO) * current_bbox_dash->width;
    yMin = current_bbox_dash->y - 2.25f * current_bbox_dash->width;
    yMax = current_bbox_dash->y + 0.25f * current_bbox_dash->width;
    current_box_above_dash = cv::Rect
        ((int) xMin, (int) yMin, (int) (xMax - xMin + 1), (int) (yMax - yMin + 1));

    //      cout << "#" << unders_idx << ":(" << xMin << "," << yMin << ")->("
    //      << xMax << "," << yMax << ")" << endl;

    current_bbox_letter = bboxes.begin();
    current_points_letter = components_points.begin();
    best_score = INFINITY;
    best_index = -1;

    /* find the component which is the most over the dash */
    for (unsigned int idx = 0; idx < components_points.size(); ++idx) {
      // keep only the not dash components
      if (is_comp_free.at(idx) == 0) {
        ++current_bbox_letter;
        continue;
      }
      // dismiss the little ones
      //if (current_points_letter->size() < 15)
      if (current_bbox_letter->width < 10) {
        ++current_bbox_letter;
        continue;
      }

      // if the bbox of the letter is
      // included in the bbox above the dash
      if (vision_utils::bboxes_included(current_box_above_dash,
                                          *current_bbox_letter)) {
        // compute its score
        curr_score = match_bboxes(&current_box_above_dash,
                                  &(*current_bbox_letter));
        //cout << "include ! curr_score:" << curr_score << endl;

        // and if it is the best, keep the index
        if (curr_score < best_score) {
          if (best_index != -1) // free the old one
            is_comp_free.at(best_index) = 1;
          best_score = curr_score;
          best_index = idx;
          is_comp_free.at(best_index) = 0;
        }
      } // end of bbox included
      ++current_bbox_letter;
      ++current_points_letter;
    } // end of loop idx

    //cout << "Making pair:" << pairs_dash_letter.at( unders_idx ).first << "<->" << best_index << endl;
    pairs_dash_letter.at(unders_idx).second = best_index;
  } // end of loop`dash_index

  //delete[] is_comp_free;

#ifdef PZ_FIND_IMAGES
  /* the searching boxes */
  cv::Mat3b illus (playzone.size());
  cv::cvtColor(playzone_monochrome, illus, CV_GRAY2RGB);

  for (unsigned int unders_idx = 0;
       unders_idx < pairs_dash_letter.size(); ++unders_idx) {
    current_bbox_dash = bboxes.begin() + pairs_dash_letter.at(unders_idx).first;

    // compute a bbox above the dash
    xMin = current_bbox_dash->x - DASH_WIDTH_BBOX_RATIO * current_bbox_dash->width;
    xMax = current_bbox_dash->x + (1 + DASH_WIDTH_BBOX_RATIO) * current_bbox_dash->width;
    yMin = current_bbox_dash->y - 2.25f * current_bbox_dash->width;
    yMax = current_bbox_dash->y + 0.25f * current_bbox_dash->width;
    current_box_above_dash = cv::Rect( (int) xMin, (int) yMin,
                                       (int) (xMax -xMin+1), (int) (yMax-yMin+1));

    vision_utils::drawRectangle(illus, (*current_bbox_dash),
                               vision_utils::color_scalar<cv::Scalar>(unders_idx,
                                                                     pairs_dash_letter.size()), 2);
    vision_utils::drawRectangle(illus, current_box_above_dash,
                               vision_utils::color_scalar<cv::Scalar>(unders_idx,
                                                                     pairs_dash_letter.size()), 2);
  }

  cv::imwrite("305-illus.png", illus);
#endif
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::set_letter_is_forbidden(char c, bool is_forbidden) {
  ROS_INFO("set_letter_is_forbidden(%c, %i)", c, is_forbidden );
  //cout << (int) c - (int) 'a' << endl;
  int idx = (int) c - (int) 'a';
  if (idx >= 0 && idx < 26)
    letters_forbidden[idx] = is_forbidden;
  else        ROS_INFO("Impossible to set this letter available, out of bounds !");
}

////////////////////////////////////////////////////////////////////////////////

bool CHangMan::recognize_letters() {
  ROS_INFO("recognize_letters()");

  /* display letters findable */
  bool is_a_letter_findable = 0;
  std::ostringstream letters;
  letters << "Letters findable: [";
  for (int i = 0; i < 26; ++i) {
    if (!letters_forbidden[i]) {
      letters << (char) (i + 'a') << " ";
      is_a_letter_findable = 1;
    }
  }
  letters << "]";
  ROS_INFO("letters:'%s'", letters.str().c_str() );

  std::vector<cv::Point>* curr_comp;
  char curr_symbol;
  std::vector<std::pair<int, int> >::iterator it = pairs_dash_letter.begin();
  std::string bestFilename;
  double bestDistance = 0;
  found_symbols.clear();
  comparer.setMask(letters_forbidden);

  assert(pairs_dash_letter.size() == hang_solver.get_number_of_letters_in_word());
  for (unsigned int i = 0; i < pairs_dash_letter.size(); ++i) {
    std::ostringstream m;
    m << "Letter #" << i << " : ";
    curr_symbol = '?';

    /* no letter allowed to be found => skip */
    if (is_a_letter_findable == 0) {
      m << "No letter findable => curr_symbol = ' '";
      curr_symbol = ' ';
    }

    /* empty dash => skip the analyze */
    if (curr_symbol == '?' && (*it).second == -1) {
      m << "dash associated with nobody => curr_symbol = ' '";
      curr_symbol = ' ';
    }

    /* letter already analyzed => skip the analyze */
    if (hang_solver.get_word_letter_status(i) != HangmanSolver::WORD_LETTER_UNKNOWN) {
      m << "dash aready analyzed => curr_symbol = '/'";
      curr_symbol = '/';
    }

    /* letter never analyzed => analyze the component */
    if (curr_symbol == '?') {
      curr_comp = &components_points.at((*it).second);
      comparer.compareVector(*curr_comp);

      curr_symbol = bestFilename.at(bestFilename.size() - 5);
      m << "bestFilename:" << bestFilename << " - bestDistance:"
        << bestDistance;
    }

    /* add to the results */
    found_symbols.push_back(curr_symbol);

    m << " | symbol='" << curr_symbol << "'";
    ROS_INFO("m:%s", m.str().c_str() );
    // bad recognition
    if (bestDistance > LETTER_RECOG_THRES)
      return 0;

    ++it;
  }

  /* if we miss some symbols, restart */
  if (found_symbols.size() != hang_solver.get_number_of_letters_in_word() ) {
    ROS_WARN("Problem with the recognition, restarting");
    ROS_INFO("found_symbols.size():%i" , (int) found_symbols.size() );
    ROS_INFO("hang_solver.NB_LETTERS_IN_WORD:%i", hang_solver.get_number_of_letters_in_word() );
    return 0;
  }

  return 1;
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::send_data_to_solver() {
  ROS_INFO("send_data_to_solver()");

  // if new letter, add it to the recognizables
  for (unsigned int i = 0; i < found_symbols.size(); ++i) {
    char c = found_symbols.at(i);
    if (c != ' ' && c != '/')
      hang_solver.set_letter_at_position(i, c);
  }



  HangmanSolver::HangerAnswer hanger_ans;
  /* if Maggie asked a word, check if it good */
  if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_WORD) {
    // check if Maggie has found with the last answer
    bool has_found = 1;
    for (unsigned int i = 0; i < found_symbols.size(); ++i) {
      char c = found_symbols.at(i);
      if (c != '/' && c != last_robot_answer_string.at(i)) {
        has_found = 0;
        break;
      } // end if
    } // end for i

    std::ostringstream sentence;
    if (has_found) {
      sentence << "|en:So, at last, it was the word \""
               << last_robot_answer_string << "\"";
      sentence << "|es:Asi que, es la palabra \""
               << last_robot_answer_string << "\"";
      say_text(sentence.str());
      hanger_ans = HangmanSolver::HANGER_ANSWER_GAME_WON_BY_FINDER;
    } else {
      sentence << "|en:What ? It was not the word \""
               << last_robot_answer_string << "\" ?";
      sentence << "|es:Que ? No es la palabra \""
               << last_robot_answer_string << "\" ?";
      say_text(sentence.str());
      hanger_ans = HangmanSolver::HANGER_ANSWER_WORD_WRONG;
    }
  }

  /* if Maggie asked a letter, check if it good */
  else if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_LETTER) {
    // check if letter found
    char c = last_robot_answer_string.at(0);
    bool has_found = 0;
    for (unsigned int i = 0; i < found_symbols.size(); ++i) {
      if (c == found_symbols.at(i)) {
        has_found = 1;
        break;
      } // end if
    } // end for i

    if (!has_found) {
      set_letter_is_forbidden(c, 1);
      std::ostringstream sentence_en, sentence_es, sentence;
      sentence_en << "|en:Ah ? The letter " << say_letter(c, "en")
                  << " is not in your word?";
      sentence_es << "|es:Ah ? La letra " << say_letter(c, "es")
                  << " no esta en la palabra?";
      sentence << sentence_en.str() << sentence_es.str();
      say_text(sentence.str());
      hanger_ans = HangmanSolver::HANGER_ANSWER_LETTER_NOT_FOUND;
    } else {
      say_text("|en:Awesome|es:Bueno !");
      hanger_ans = HangmanSolver::HANGER_ANSWER_LETTER_FOUND;
    }
  }

  /* if maggie has won => do nothing */
  else if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_GAME_WON_BY_FINDER) {
    hanger_ans = HangmanSolver::HANGER_ANSWER_GAME_WON_BY_FINDER;
  }

  /* if maggie has lost => do nothing */
  else if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_GAME_LOST_BY_FINDER) {
    hanger_ans = HangmanSolver::HANGER_ANSWER_GAME_LOST_BY_FINDER;
  }

  ROS_INFO("Type of answer to the solver:%i", hanger_ans );

  std::vector<unsigned int> nill;
  hang_solver.finder_receive_data_from_hanger(hanger_ans, nill);
  hang_solver.display_word_letters_status();
  hang_solver.finder_compute_next_guess();
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::analyze_solver_answer() {
  ROS_INFO("analyze_solver_answer()");
  hang_solver.finder_get_answer(last_robot_answer_type, last_robot_answer_string);
  ROS_INFO("Number of possible words:%i",
               hang_solver.get_number_of_possible_words() );
  if (hang_solver.get_number_of_possible_words() < 100)
    hang_solver.display_all_possible_words();

  //cout << "last_answer_type:" << last_answer_type << endl;

  if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_GAME_WON_BY_FINDER) {
    std::ostringstream sentence_en, sentence_es, sentence;
    sentence_en << "|en:Yuhu, I have won! I have only made "
                << hang_solver.get_number_of_failures() << " errors. ";
    sentence_es << "|es:Te he ganado, jaja! Solo he hecho "
                << hang_solver.get_number_of_failures() << " errores. ";
    sentence << sentence_en.str() << sentence_es.str();
    say_text(sentence.str());
    // play gesture "flori_happy"
    std_msgs::String gesture_name; gesture_name.data = "flori_happy";
    _gesture_pub.publish(gesture_name);
    // set eyes as HAPPY
    set_eyes("NORMAL");
    sleep(5);
  }

  else if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_GAME_LOST_BY_FINDER) {
    std::ostringstream sentence_en, sentence_es, sentence;
    sentence_en << "|en:Oo, I have made "
                << hang_solver.get_number_of_failures() << " errors. "
                << "I have lost. Well played!";
    sentence_es << "|es:Jo, he hecho "
                << hang_solver.get_number_of_failures() << " errores. "
                << "Me has ganado, enahorabuena!";
    sentence << sentence_en.str() << sentence_es.str();
    say_text(sentence.str());
    // play gesture "sad"
    std_msgs::String gesture_name; gesture_name.data = "flori_sad";
    _gesture_pub.publish(gesture_name);
    // set eyes as ANGRY
    set_eyes("ANGRY");
    sleep(5);
  }

  else if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_WORD) {
    std::ostringstream sentence_en, sentence_es, sentence;
    if (rand() % 2 == 0) {
      sentence_en << "|en:I am gonna try the word \"";
      sentence_es << "|es:Voy a probar la palabra \"";
    }
    else {
      sentence_en << "|en:I think this is the word \"";
      sentence_es << "|es:Creo que es la palabra \"";
    }
    // add the spelling
    sentence_en << last_robot_answer_string << "\", written as follows: ";
    sentence_es << last_robot_answer_string << "\", escrita de la manera siguiente: ";
    for (unsigned int letter_idx = 0; letter_idx < last_robot_answer_string.size();
         ++letter_idx) {
      sentence_en << say_letter(last_robot_answer_string[letter_idx],
                                "en")
                  << ", ";
      sentence_es << say_letter(last_robot_answer_string[letter_idx],
                                "es")
                  << ", ";
    } // end loop letter_idx

    // concatenate sentences in english and spanish
    sentence << sentence_en.str() << sentence_es.str();
    say_text(sentence.str());

    // allow all the letters to be recognized
    for (std::string::iterator it = last_robot_answer_string.begin(); it
         < last_robot_answer_string.end(); it++)
      set_letter_is_forbidden(*it, 0);
  }

  else if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_LETTER) {
    char c = last_robot_answer_string.at(0);
    std::ostringstream sentence_en, sentence_es, sentence;
    if (rand() % 2 == 0) {
      sentence_en << "|en:I am gonna try the letter ";
      sentence_es << "|es:Voy a probar la letra ";
    } else {
      sentence_en << "|en:In this word, there is the letter ";
      sentence_es << "|es:In esta palabra, esta la lettra ";
    }
    // make a little pause
    sentence_en << ". " << say_letter(c, "en") << "?";
    sentence_es << ". " << say_letter(c, "es") << "?";

    // concatenate sentences in english and spanish
    sentence << sentence_en.str() << sentence_es.str();
    say_text(sentence.str());
    // allow the letter to be recognized
    set_letter_is_forbidden(c, 0);
  }

  else if (last_robot_answer_type == HangmanSolver::FINDER_ANSWER_CANNOT_FIND) {
    say_text(//
             "|en:I don't know such a word. The game is over." //
             "|es:No conoceo una palabra como esta. El juego es acabado.");
  }

  // share the image for an eventual drawer
  //    std::string playzone_str;
  //    vision_utils::to_string(playzone, playzone_str);
  //    MCP.replace_item("FIND_PLAYZONE_MCP_IMG", playzone_str);
  //    eventManager.emit_("drawer_new_background", 0);

}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::generate_alphabet(int width, int height) {
  ROS_INFO("generate_alphabet()");
  int bigW = 5 * width;
  cv::Mat1b big (bigW, bigW);
  cv::Mat1b resized (width, height);

  for (char c = 'A'; c <= 'Z'; c++) {
    big = 0;
    std::ostringstream txt;
    txt << c;
    cv::putText(big, txt.str(), cv::Point(bigW / 2, bigW / 2),
                CV_FONT_HERSHEY_DUPLEX, 1.0f,
                cvScalarAll(255));
    cv::threshold(big, big, 10, 255, CV_THRESH_BINARY);

    vision_utils::redimContent(big, resized);
    std::ostringstream filename;
    filename << "data/symbols/" << (char) (c - 'A' + 'a') << ".png";
    cv::imwrite(filename.str().c_str(), resized);
  }
}

////////////////////////////////////////////////////////////////////////////////

void CHangMan::display() {
  /* display */
  //playzone.copyTo(playzone_illus);
  cv::cvtColor(playzone_monochrome, _playzone_illus, CV_GRAY2RGB);
  for (unsigned int i = 0; i < pairs_dash_letter.size(); ++i) {
    cv::Rect r1 = bboxes.at(pairs_dash_letter.at(i).first);
    cv::rectangle(_playzone_illus, r1,
                  vision_utils::color_scalar<cv::Scalar>(i, pairs_dash_letter.size()), 2);

    if (pairs_dash_letter.at(i).second >= 0) {
      cv::Rect r2 = bboxes.at(pairs_dash_letter.at(i).second);
      cv::rectangle(_playzone_illus, r2,
                    vision_utils::color_scalar<cv::Scalar>(i, pairs_dash_letter.size()), 2);
    }
  } // end loop i

#ifdef SAVE_IMAGES
  cv::imwrite("310-playzoneOut.png", playzone_out);
#endif
} // end display()

