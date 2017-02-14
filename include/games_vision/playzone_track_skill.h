#ifndef TRACK_PLAY_ZONE_SKILL_H
#define TRACK_PLAY_ZONE_SKILL_H

/***************************************************************************//**
 * \class PlayzoneTrackSkill
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 02/07/2009
 *******************************************************************************/


///// my imports
#include "games_vision/playzone_track.h"
#include "vision_utils/rgb_skill.h"

class PlayzoneTrackSkill : public PlayzoneTrack, public vision_utils::RgbSkill  {
public:
  /** constructor */
  PlayzoneTrackSkill(int BOARD_OUT_WIDTH_,
                     int BOARD_OUT_HEIGHT_);
  /** destructor */
  ~PlayzoneTrackSkill();

  /////
  ///// general functions
  /////
  /** main loop of the PlayzoneTrackSkill mode */
  void process_rgb(const cv::Mat3b & rgb);
  /** lauch the PlayzoneTrackSkill mode */
  void create_subscribers_and_publishers();
  /** terminate the PlayzoneTrackSkill mode */
  void shutdown_subscribers_and_publishers();

  virtual void display(const cv::Mat3b & /*rgb*/) {
    if (reproject_mode == MODE_REPROJECT_IMAGE || reproject_mode == MODE_REPROJECT_VIDEO)
      cv::imshow(window1Name, frame_with_model);
    else
      cv::imshow(window1Name, frame_buffer);

#ifdef PZ_FIND_IMAGES
    cv::imshow(window2Name, frameOut);
#endif // PZ_FIND_IMAGES

    /* key listener */
    char c = cv::waitKey(5);
    if (c == ' ') {
      find_from_playzone();
    }
    if (c == 27)
      exit(0); // exit key pressed - DON'T DELETE THIS LINE OR THERE WILL BE NO WINDOW DISPLAYED
  } // end display();


private:
  //! the name of the first window
  static std::string window1Name;
  //! the name of the second window
  static std::string window2Name;

};

#endif

