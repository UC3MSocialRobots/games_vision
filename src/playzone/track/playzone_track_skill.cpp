
/***************************************************************************//**
 * \file PlayzoneTrackSkill.cpp
 *
 * \brief The implementation of playzone_track.h
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 25/05/2009
 *******************************************************************************/

#include <vision_utils/img_path.h>

#include "games_vision/playzone_track_skill.h"

// opencv
#include <opencv2/imgproc/imgproc.hpp>

std::string PlayzoneTrackSkill::window1Name = "PlayzoneTrackSkill-original";
std::string PlayzoneTrackSkill::window2Name = "PlayzoneTrackSkill-modified";

////////////////////////////////////////////////////////////////////////////////

PlayzoneTrackSkill::PlayzoneTrackSkill(int BOARD_OUT_WIDTH_,
                                       int BOARD_OUT_HEIGHT_)
  : PlayzoneTrack(BOARD_OUT_WIDTH_, BOARD_OUT_HEIGHT_),
    RgbSkill("PLAYZONE_TRACK_START", "PLAYZONE_TRACK_STOP") {
  ROS_INFO("ctor");
}

////////////////////////////////////////////////////////////////////////////////

PlayzoneTrackSkill::~PlayzoneTrackSkill() {
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneTrackSkill::create_subscribers_and_publishers() {
  ROS_INFO("create_subscribers_and_publishers()");

  /* creating the windows */
  //  if (DISPLAY) {
  //      cv::namedWindow(window1Name);
  //      cv::namedWindow(window2Name);
  //      cvMoveWindow(window1Name.c_str(), 0, 0);
  //      cvMoveWindow(window2Name.c_str(), 650, 0);
  //  }
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneTrackSkill::shutdown_subscribers_and_publishers() {
  ROS_INFO("shutdown_subscribers_and_publishers();");
  // killing the windows
  //  if (DISPLAY)
  //      cvDestroyAllWindows();

  // killing the frames
  // ...useless
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneTrackSkill::process_rgb(const cv::Mat3b & rgb) {
  //ROS_INFO("---");
  ROS_INFO("process_rgb()");

  /* acquire images */
  rgb.copyTo(PlayzoneFind::frame_buffer);

  find_playzone();
}

