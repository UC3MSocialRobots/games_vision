#include "games_vision/playzone_find_benchmark.h"

////////////////////////////////////////////////////////////////////////////////

PlayzoneFindBenchmark::PlayzoneFindBenchmark(const std::string & path_to_xml_file,
                                             const std::string & xml_filename) :
  _playzone_finder(PlayzoneFindBenchmark::PLAYZONE_OUT_WIDTH,
                   PlayzoneFindBenchmark::PLAYZONE_OUT_HEIGHT) {
  from_xml_file(path_to_xml_file, xml_filename);
  ROS_INFO("ctor");
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFindBenchmark::start() {
  ROS_INFO("start()");

  // process all the files
  _nb_files_successful = 0;
  _nb_files_processed = 0;
  _global_times.clear_times();

  while (_nb_files_processed <= get_nb_files()) {
    process_current_file();
    ++_nb_files_processed;

    ROS_WARN("_current_status:'%s'", get_current_status_as_string().c_str());
    ROS_WARN("Success: %i/%i, _global_times:'%s'",
                _nb_files_successful, _nb_files_processed,
                _global_times.to_string().c_str());

    //cv::imshow("input", *get_current_cv_img()); cv::imshow("get_playzone", *_playzone_finder.get_playzone()); cv::waitKey(5000);
    //ROS_WARN("waiting for keyboard return..."); std::cin.ignore();

    go_to_next_file();
  }
}

////////////////////////////////////////////////////////////////////////////////

PlayzoneFindBenchmark::Status PlayzoneFindBenchmark::get_current_status() const {
  return _current_status;
}

////////////////////////////////////////////////////////////////////////////////

std::string PlayzoneFindBenchmark::get_current_status_as_string() const {
  switch (_current_status) {
    case PROCESSING:
      return "PROCESSING.";
      break;
    case DETECTION_POSITIVE:
      return "DETECTION_POSITIVE.";
      break;
    case DETECTION_POSITIVE_WRONG_LOCATION:
      return "DETECTION_POSITIVE_WRONG_LOCATION.";
      break;
    case DETECTION_FALSE_NEGATIVE:
      return "DETECTION_FALSE_NEGATIVE.";
      break;
  }
  return "unknwon.";
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFindBenchmark::preprocessing_function(cv::Mat3b &) {
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFindBenchmark::process_current_file() {
  ROS_DEBUG("process_current_file()");

  preprocessing_function(*get_current_cv_img());

  // make the PlayzoneFind work
  _playzone_finder.set_input(get_current_cv_img());

  // get answer of PlayzoneFind
  bool detection_success = _playzone_finder.find_playzone();
  ROS_INFO("'%s' : pz.status:'%s' (%i), times:'%s'",
               get_current_filename().c_str(),
               _playzone_finder.get_current_status_as_string().c_str(),
               detection_success,
               _playzone_finder.get_current_times().to_string().c_str());
  if (detection_success == false) {
    _current_status = DETECTION_FALSE_NEGATIVE;
    return;
  }

  // compare the corners with the manual annotations
  if (corners.size() != 4) {
    ROS_WARN("Fail while loading the corners !");
    return;
  }
  //CornerList correct_corners = corners;
  CornerList computed_corners;
  _playzone_finder.get_corner_list(computed_corners);
  double avg_distance_per_corner = sqrt(distance_to_answer(computed_corners)) / 4;
  ROS_INFO("correct_corners:'%s', computed_corners:'%s', avg_distance_per_corner:%f",
               vision_utils::accessible_to_string(corners).c_str(),
               vision_utils::accessible_to_string(computed_corners).c_str(),
               avg_distance_per_corner);

  // test if the average distance is over threshold
  if (avg_distance_per_corner > MAX_DIST_PER_CORNER_FOR_SUCCESFUL_MATCHING) {
    _current_status = DETECTION_POSITIVE_WRONG_LOCATION;
    return;
  }

  // update the times
  PlayzoneFind::Times current_times = _playzone_finder.get_current_times();
  update_timer_ntimes(_global_times.t01_after_thresh_image, _nb_files_successful,
                      current_times.t01_after_thresh_image);
  update_timer_ntimes(_global_times.t02_after_numeroteComponents, _nb_files_successful,
                      current_times.t02_after_numeroteComponents);
  update_timer_ntimes(_global_times.t03_after_compareComponents, _nb_files_successful,
                      current_times.t03_after_compareComponents);
  update_timer_ntimes(_global_times.t04_after_getCorners, _nb_files_successful,
                      current_times.t04_after_getCorners);
  update_timer_ntimes(_global_times.t05_after_saving_images, _nb_files_successful,
                      current_times.t05_after_saving_images);
  update_timer_ntimes(_global_times.t06_after_removeBorder2, _nb_files_successful,
                      current_times.t06_after_removeBorder2);
  update_timer_ntimes(_global_times.t07_after_calc_warp_matrix, _nb_files_successful,
                      current_times.t07_after_calc_warp_matrix);
  update_timer_ntimes(_global_times.t08_after_rectif_image, _nb_files_successful,
                      current_times.t08_after_rectif_image);
  update_timer_ntimes(_global_times.t_after_everything, _nb_files_successful,
                      current_times.t_after_everything);

  // if we reach here still processing, it is a success !
  _current_status = DETECTION_POSITIVE;
  ++_nb_files_successful;

}

////////////////////////////////////////////////////////////////////////////////

