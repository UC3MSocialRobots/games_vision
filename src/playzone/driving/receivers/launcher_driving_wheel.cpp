/*!
  \file        launcher_driving_wheel.cpp
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

#include "vision_utils/nano_skill.h"
#include <vision_utils/img_path.h>
#include "games_vision/WheelReceiver.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//! the driving wheel
class DrivingWheel : public NanoSkill, WheelReceiver {
public:
  DrivingWheel()
    : NanoSkill( "DRIVINGWHEEL_START", "DRIVINGWHEEL_STOP") {
    ROS_INFO("ctor");

    // prepair the images
    std::string wheelFilename = IMG_DIR  "driving/wheel_game.png";
    ROS_INFO("wheelFilename:'%s'", wheelFilename.c_str());
    wheel_image = cv::imread(wheelFilename.c_str(), CV_LOAD_IMAGE_COLOR);
    wheel_image_rotated_and_tachy.create(cv::Size(wheel_image.cols + 150, wheel_image.rows));
    wheel_rot_mat.create(2, 3, CV_32FC1);

    /* create window */
    window2Name = "DrivingWheel";
    //cvMoveWindow(window2Name, game_size.width + 10, 0);
  }

  //////////////////////////////////////////////////////////////////////////////

  ~DrivingWheel() {
    ROS_INFO("dtor");
  }

  //////////////////////////////////////////////////////////////////////////////

  void proceso() {
    //ROS_INFO("proceso()");

    /* draw wheel */
    compute_wheel_image();

    cv::imshow(window2Name, wheel_image_rotated_and_tachy);

    short c = (short) ((char) cv::waitKey(15));
    //cout << c << endl;
    if (c == 27)
      exit(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  void compute_wheel_image() {
    wheel_image_rotated_and_tachy.setTo(0);

    /* rotate the wheel */
    cv::Point2f wheel_center(wheel_image.cols / 2, 0.45f * wheel_image.rows);
    wheel_rot_mat = cv::getRotationMatrix2D(
          wheel_center,
          -get_wheel_angle(),
          1.0);
    cv::Mat3b wheel_rot_roi = wheel_image_rotated_and_tachy(cv::Rect(
                                                              0, 0, wheel_image.cols, wheel_image.rows));
    cv::warpAffine(wheel_image, wheel_rot_roi,
                   wheel_rot_mat,
                   wheel_image.size());

    /* draw the speed-o-meter */
    double a = 1.f * (wheel_image.rows - 2 * y_margin) /
        (WheelReceiver::WHEEL_SPEED_MODULE_MIN - WheelReceiver::WHEEL_SPEED_MODULE_MAX);
    double b = 1.f * y_margin - a * WheelReceiver::WHEEL_SPEED_MODULE_MAX;
    // draw the frame of the tachy
    cv::rectangle(wheel_image_rotated_and_tachy,
                  cv::Point (wheel_image.rows + 50,
                             b + a * WheelReceiver::WHEEL_SPEED_MODULE_MIN),
                  cv::Point (wheel_image.rows + 80,
                             b + a * WheelReceiver::WHEEL_SPEED_MODULE_MAX),
                  CV_RGB(150, 150, 150), 3);
    // draw the real speed
    cv::Scalar color = (get_speed_module() >= 0 ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0));
    cv::rectangle(wheel_image_rotated_and_tachy,
                  cv::Point (wheel_image.rows + 50, b),
                  cv::Point (wheel_image.rows + 80, b + a * get_speed_module()),
                  color, -1);
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    ROS_INFO("create_subscribers_and_publishers()");
    cvStartWindowThread();
    cv::namedWindow(window2Name, CV_WINDOW_AUTOSIZE);
  }

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    ROS_INFO("shutdown_subscribers_and_publishers()");
    cv::destroyWindow(window2Name);
  }

  //////////////////////////////////////////////////////////////////////////////

  const char* window2Name;
  cv::Mat3b wheel_image;
  cv::Mat3b wheel_image_rotated_and_tachy;
  cv::Mat wheel_rot_mat;
  static const int y_margin = 15;
}; // end class DrivingWheel

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "launcher_driving_wheel");
  DrivingWheel skill;
  skill.check_autostart();
  ros::spin();
  return 0;
}
