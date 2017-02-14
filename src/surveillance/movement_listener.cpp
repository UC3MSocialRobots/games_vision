/*!
  \file        movement_listener.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        18/06/2009

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

\class MovementListener
\brief a watcher of image evolution
It triggers an event when the image is modified more than a certain
threshold

\section Parameters
  - \b "~threshold_pixel_diff"
        [double, pixel value] (default: "20")
        The difference between two pixels (in greyscale values)
        to estimate they are equal.

  - \b "~threshold_image_rate_unstable"
        [double, percentage] (default: "0.2")
        The percentage of pixels equal in two consecutive frames to
        estimate these two images equal.

  - \b "~threshold_stable_time"
        [double, seconds] (default: "3")
        The image is defined as stable if
        all consecutive acquired frames during this time
        are similar enough.

\section Subscriptions
  - Image topic : \see RgbSkill
    No depth needed for this skill.

\section Publications
  - \b "alert_messages"
        [games_vision::AlertMessage]
        The alert messages we publish when someone enters a forbidden zone.
*/

// OpenCV
#include "opencv2/imgproc/imgproc.hpp"
// STL imports
#include <deque>            // for double ended queues
// AD
#include "games_vision/AlertMessage.h"
#include "vision_utils/matrix_testing.h"
#include "vision_utils/nano_etts_api.h"
#include "vision_utils/rectangle_intersection.h"
#include "vision_utils/rectangle_to_string.h"
#include "vision_utils/rgb_skill.h"
#include "vision_utils/sensor_cv_encodings_bridge.h"
#include "vision_utils/timer.h"

class MovementListener: public vision_utils::RgbSkill {
public:
  //! constructor
  MovementListener() :
    RgbSkill("MOVEMENTLISTENER_START", "MOVEMENTLISTENER_STOP")
  // , neck(robot_config)
  {
    ROS_INFO("MovementListener constructor");
    _window1_name = "MovementListener1";
    _window2_name = "MovementListener2";

    ros::NodeHandle nh_public, nh_private("~");
    // parameters
    nh_private.param("threshold_pixel_diff", threshold_pixel_diff, 20.);
    nh_private.param("threshold_image_rate_unstable", threshold_image_rate_unstable, 0.2);
    nh_private.param("threshold_stable_time", threshold_stable_time, 3.);

    // publishers
    _alert_message_pub = nh_public.advertise
        <games_vision::AlertMessage>("alert_messages", 1);
    ROS_WARN("MovementListener: "
             "Publishing alert messages on: '%s'. ",
             _alert_message_pub.getTopic().c_str());

    /* init the other things */
    set_threshold_stable_time(threshold_stable_time);
    set_threshold_image_rate_unstable(threshold_image_rate_unstable);
  }

  /////////////////////////////////////////////////////////////////////////////

  //! destructor
  ~MovementListener() {
    // unsuscribe to the signals
    // ...
  }

  /////////////////////////////////////////////////////////////////////////////

  /** lauch the MovementListener mode */
  void create_subscribers_and_publishers() {
    ROS_INFO("create_subscribers_and_publishers()");

    is_watched_zone_set = false;
    //cv::namedWindow(_window1_name);
    //cv::namedWindow(_window2_name);

    /* move the head up */
    // neck.move_theta( -20);
  } // end create_subscribers_and_publishers();

  /////////////////////////////////////////////////////////////////////////////

  /** terminate the MovementListener mode */
  void shutdown_subscribers_and_publishers() {
    ROS_INFO("shutdown_subscribers_and_publishers()");
    //cv::destroyWindow(_window1_name);
    //cv::destroyWindow(_window2_name);
  }

  /////////////////////////////////////////////////////////////////////////////

  /** main loop of the MovementListener mode */
  void process_rgb(const cv::Mat3b & rgb) {
    // ROS_INFO_THROTTLE(1, "process_rgb()");

    if (!is_watched_zone_set)
      set_random_watched_zone(rgb);
    exam(rgb);
  } // end process_rgb()

  //////////////////////////////////////////////////////////////////////////////

  virtual void display(const cv::Mat3b & rgb) {
    //cv::imshow("rgb", rgb);
    rgb.copyTo(frame_out);
    cv::Scalar color = (is_stable ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0));
    //      ROS_WARN("watched_zone:'%s'",
    //               vision_utils::print_rect(watched_zone).c_str());
    cv::rectangle(frame_out, watched_zone, color, 3);

    cv::imshow(_window1_name, frame_out);
    if (!frame_diff.empty())
      cv::imshow(_window2_name, frame_diff);
    /* key listener */
    char c = cv::waitKey(50);
    if (c == ' ')
      set_random_watched_zone(rgb);
    else if ((int) c == 27)
      exit(0);
  }


  //////////////////////////////////////////////////////////////////////////////

  /*! check when the last alert was given.
   *  If long enough, send an alert message.
   */
  void alert_if_no_timeout(const cv::Mat3b & rgb) {
    ros::Duration time_since_last_alert = _images_header.stamp - _last_alert_stamp;
    if (time_since_last_alert.toSec() < 5) {
      ROS_WARN("MovementListener:there is motion, "
               "but last alert was given %g seconds ago. Skipping.",
               time_since_last_alert.toSec());
      return;
    }
    _last_alert_stamp = _images_header.stamp;
    ROS_WARN("MovementListener:there is motion. Alerting.");

    games_vision::AlertMessage alert_msg;
    alert_msg.header = _images_header;
    alert_msg.message = "Motion in front of the camera!";
    alert_msg.priority = games_vision::AlertMessage::PRIORITY_WARN;
    alert_msg.etts_sentence = "en:Motion!|es:Movimiento!";
    // load and convert imaghe
    cv_bridge::CvImage cv_image;
    rgb.copyTo(cv_image.image);
    cv_image.encoding = vision_utils::sensor_encoding_from_cv_encoding(cv_image.image.type());
    cv_image.toImageMsg(alert_msg.image);
    _alert_message_pub.publish(alert_msg);
  } // end alert_if_no_timeout();

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * \brief   change the maximum difference rate for declaring an image stable
   * \param   rate the max rate (for instance 1E-2 = 1%)
   */
  void set_threshold_image_rate_unstable(double rate) {
    ROS_INFO("set_threshold_image_rate_unstable(rate:%f)", rate);
    threshold_image_rate_unstable = rate;
  }

  /////////////////////////////////////////////////////////////////////////////

  /*!
   * \brief   change the minimum time for declaring the image stable
   * \param   time_sec the minimum time in seconds
   */
  void set_threshold_stable_time(int time_sec) {
    ROS_INFO("set_threshold_stable_time(%i)", time_sec);
    threshold_stable_time = time_sec;
  }

  /////////////////////////////////////////////////////////////////////////////

  //! define the total image as the new watched zone
  void set_random_watched_zone(const cv::Mat3b & rgb) {
    // set_watched_zone(cv::Rect(0, 0, rgb.cols, rgb.rows));
    cv::Rect new_roi(rgb.cols / 2 - (100 + rand() % 30),
                     rgb.rows / 2 - (100 + rand() % 30),
                     200 + rand() % 60,
                     200 + rand() % 60);
    new_roi = vision_utils::rectangle_intersection
        (new_roi, cv::Rect(0, 0, rgb.cols, rgb.rows));
    set_watched_zone(new_roi);
  }

  /////////////////////////////////////////////////////////////////////////////

  /*!
   * \brief   define a new watched zone in the image
   * \param   zone the new zone
   */
  void set_watched_zone(const cv::Rect & zone) {
    ROS_INFO("set_watched_zone(%s)", vision_utils::rectangle_to_string(zone).c_str());
    watched_zone = zone;
    is_watched_zone_set = true;
    is_stable = true; // set new zones as stable
  }

  /////////////////////////////////////////////////////////////////////////////

  //! a routine to watch if there is a change of status
  void exam(const cv::Mat3b & rgb) {
    // ROS_INFO_THROTTLE(1, "exam()");
    /* reset ROI and set the new ROI */
    rgb(watched_zone).copyTo(frame_roi);
    /* get the diff with the previous image and threshold it */
    // grayscale conversion
    cv::cvtColor(frame_roi, frameBW_roi, CV_BGR2GRAY);
    cv::blur(frameBW_roi, frameBW_roi, cv::Size(5, 5));
    // equalize hist
    cv::equalizeHist(frameBW_roi, frameBW_roi);
    double difference_rate =
        (frameBW_old_roi.empty() || frameBW_roi.empty()
         ? 0 // for the zone to be stable at the beginning
         : vision_utils::rate_of_changes_between_two_images(frameBW_old_roi, frameBW_roi,
                                                              frame_diff,
                                                              (uchar) threshold_pixel_diff));
    frameBW_roi.copyTo(frameBW_old_roi); // process_rgb lastFrameBW

    // watch for a change in the last frame bigger than threshold_image_rate_unstable
    if (difference_rate > threshold_image_rate_unstable) {
      ROS_INFO_THROTTLE(1, "difference_rate:%g > %g: last frame not stable!",
                        difference_rate, threshold_image_rate_unstable);
      is_stable = false;
      last_frame_stable_time.reset();
      // send an alert message!
      alert_if_no_timeout(rgb);
    }
    else { /* last frame is stable ->
      watch if there has not been any change bigger
      than threshold_image_rate_unstable since more than threshold_stable_time */
      // ROS_INFO("time_stable:%f", last_frame_stable_time.getTimeSeconds());
      if (last_frame_stable_time.getTimeSeconds() > threshold_stable_time) {
        ROS_INFO_THROTTLE(1, "difference_rate:%g, time_stable:%g > %g -> stable",
                          difference_rate,
                          last_frame_stable_time.getTimeSeconds(), threshold_stable_time);
        is_stable = true;
      }
      else
        ROS_INFO_THROTTLE(1, "difference_rate:%g, time_stable:%g",
                          difference_rate, last_frame_stable_time.getTimeSeconds());
    } // end if !is_stable
  } //end exam()

  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  double threshold_stable_time;
  double threshold_image_rate_unstable;
  double threshold_pixel_diff;

  //! the acquired frame
  cv::Mat3b frame_roi;
  //! the output
  cv::Mat3b frame_out;
  //! the BW version of the acquired frame
  cv::Mat1b frameBW;
  cv::Mat1b frameBW_roi;
  //! the BW version of the acquired frame before
  cv::Mat1b frameBW_old_roi;
  //! the differences between the last frame and the one before
  cv::Mat1b frame_diff;
  //NeckApi neck;
  cv::Rect watched_zone;
  bool is_watched_zone_set;

  bool is_stable;
  vision_utils::Timer last_frame_stable_time;

  std::string _window1_name;
  std::string _window2_name;

  ros::Publisher _alert_message_pub;
  ros::Time _last_alert_stamp;
}; // end class MovementListener

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "MovementListener");
  MovementListener skill;
  skill.check_autostart();
  ros::spin();
}

