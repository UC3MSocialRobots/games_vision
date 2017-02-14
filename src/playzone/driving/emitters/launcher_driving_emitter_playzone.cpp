/*!
  \file        launcher_driving_emitter_playzone.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/6/27

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file
 */
//! generate videos from the display
#define PZ_DRIVING_VIDEOS 0

// games_vision
#include "games_vision/WheelEmitter.h"
#include "games_vision/playzone_track_skill.h"

class DrivingEmitterPlayzone : public PlayzoneTrackSkill, public WheelEmitter {
public:
  //! the speed of the car when the sheel is detected for the first time
  static const int SPEED_DEFAULT = 10;
  //! the ratio between the variations of size of the wheel and the acceleration of the car
  static const int SPEED_RATIO = 60;

  /** constructor */
  DrivingEmitterPlayzone() :
    PlayzoneTrackSkill(50, 50) {
    ROS_INFO("DrivingEmitterPlayzone ctor");
    // set_new_events("DRIVING_EMITTER_PLAYZONE_START", "DRIVING_EMITTER_PLAYZONE_STOP");
    set_mode(PlayzoneTrack::MODE_ONLY_TRACK, "");
  }
  /** destructor */
  ~DrivingEmitterPlayzone() {
    ROS_INFO("DrivingEmitterPlayzone dtor");
  }

  /////
  ///// general functions
  /////
  /** main loop of the DrivingEmitterPlayzone mode */
  void process_rgb(const cv::Mat3b & rgb) {
    ROS_INFO("process_rgb()");

    double final_speed = WheelEmitter::ERROR_SPEED;
    double final_angle = WheelEmitter::ERROR_ANGLE;

  #if PZ_DRIVING_VIDEOS
    writer_track_frame.write(PlayzoneTrackSkill::frame );
    writer_track_frameOut.write(PlayzoneTrackSkill::frameOut );
    writer_game_screen.write(game.screen );

    cv::Mat3b wheel_buffer ( PlayzoneTrackSkill::frame.size() );
    wheel_buffer.setTo(0);
    cvSetImageROI( wheel_buffer, cvGetImageROI(game.car->wheel_and_speedmeter_image) );
    cvCopy(game.car->wheel_and_speedmeter_image, wheel_buffer);
    cvResetImageROI( wheel_buffer );
    writer_game_wheel.write(wheel_buffer );
  #endif

    PlayzoneTrackSkill::process_rgb(rgb);
    if (!PlayzoneTrackSkill::is_tracking_OK) {
      ROS_INFO("PlayzoneTrackSkill::is_tracking_OK = FALSE, return");
      square_normal_dimension = -1;
      //cv::cvtColor(game.screen, screenBW, CV_RGB2GRAY);
      //cv::imshow(game.window1Name, screenBW);
      return;
    }

    /* finding rotation */
    cv::Point center = vision_utils::barycenter(board_corners);
    //cout << "center" << center.x << "," << center.y << endl;
    double angle1 = atan2(board_corners.at(2).y - center.y, board_corners.at(2).x
                          - center.x);
    double angle2 = atan2(board_corners.at(3).y - center.y, board_corners.at(3).x
                          - center.x);
    if (angle2 < angle1)// && angle1 > CV_PI/2)
      angle1 -= 2 * CV_PI;
    double angle_avg = (angle1 + angle2 + CV_PI) * .5f;
    final_angle = angle_avg * RAD2DEG;
    //      cout << "angle1:" << angle1 << ", angle2:" << angle2 << ", angle_avg:"
    //                      << angle_avg << endl;

    /* finding speed */
    double square_dimension = 0;
    for (int i = 0; i < 4; ++i) {
      cv::Point* A = &board_corners.at(i);
      cv::Point* B = &board_corners.at((i + 1) % 4);
      square_dimension = fmax(square_dimension, hypot(A->x - B->x, A->y
                                                      - B->y));
    }

    if (square_normal_dimension == -1) { // new normal dimension
      square_normal_dimension = square_dimension;
    } else {
      double percent_var = (square_dimension - square_normal_dimension)
          / square_normal_dimension;
      final_speed = percent_var * (WHEEL_SPEED_MODULE_MAX - SPEED_DEFAULT)
          + SPEED_DEFAULT;
    }
    //cout << "square_dimension:" << square_dimension << endl;

    ROS_INFO("final_angle:%f, final_speed:%f", final_angle, final_speed);

    /* emit that stuff */
    emit_angle(final_angle);
    emit_speed(final_speed);
  } // end process_rgb();

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    ROS_INFO("create_subscribers_and_publishers()");

    square_normal_dimension = -1;

  #if PZ_DRIVING_VIDEOS
    int codec = //-1
        //              CV_FOURCC('M','J','P','G')    //= motion-jpeg codec (does not work well)
        CV_FOURCC('D', 'I', 'V', 'X') //= MPEG-4 codec
        ;
    int fps = 10;
    //writer_game_wheel  ("030-writer_game_wheel.avi", codec, fps, cvGetSize(game.car->wheel_and_speedmeter_image));
    writer_game_wheel  ("030-writer_game_wheel.avi", codec, fps, cvGetSize(PlayzoneTrackSkill::frame));

    writer_game_screen  ("040-writer_game_screen.avi", codec, fps, cvGetSize(game.screen));
    writer_track_frame  ("010-writer_track_frame.avi",codec, fps, cvGetSize(PlayzoneTrackSkill::frame));
    writer_track_frameOut  ("020-writer_track_frameOut.avi",codec, fps, cvGetSize(PlayzoneTrackSkill::frameOut));
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    ROS_INFO("shutdown_subscribers_and_publishers()");
    PlayzoneTrackSkill::shutdown_subscribers_and_publishers();

  #if PZ_DRIVING_VIDEOS
    cvReleaseVideoWriter( &writer_track_frame );
    cvReleaseVideoWriter( &writer_track_frameOut );
    cvReleaseVideoWriter( &writer_game_wheel );
    cvReleaseVideoWriter( &writer_game_screen );
  #endif
  }

  //////////////////////////////////////////////////////////////////////////////

  /////
  ///// gemeral parameters
  /////
  //! the size of the side of the square corresponding to the default speed
  double square_normal_dimension;

  //private:

#if PZ_DRIVING_VIDEOS
  cv::VideoWriter writer_track_frame;
  cv::VideoWriter writer_track_frameOut;
  cv::VideoWriter writer_game_wheel;
  cv::VideoWriter writer_game_screen;
#endif
}; // end class DrivingEmitterPlayzone

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "launcher_driving_emitter_playzone");
  DrivingEmitterPlayzone skill;
  skill.check_autostart();
  ros::spin();
  return 0;
}

