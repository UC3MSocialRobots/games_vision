/*!
  \file        launcher_driving_game.cpp
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

///// my imports
#include "games_vision/WheelReceiver.h"
#include "vision_utils/nano_skill.h"
#include <vision_utils/img_path.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEG2RAD 0.01745329251994329577

class DrivingGame : public NanoSkill {
public:

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  //! the Track class
  class Track {
  public:
    Track(std::string filename) {
      track_image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);

      /* move at initial position */
      //  pos_screen_x = -30;
      //  pos_screen_y = -30;
    }

    //////////////////////////////////////////////////////////////////////////////

    void draw_om_image(cv::Mat & img) {
      ROS_DEBUG("Track::draw_om_image()");

      track_image(cv::Rect(-(int) pos_screen_x, -(int) pos_screen_y,
                           img.cols, img.rows)).copyTo(img);
    }

    //////////////////////////////////////////////////////////////////////////////

    double pos_screen_x;
    double pos_screen_y;
    cv::Mat track_image;
  };

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  //! the Car class
  class Car : public WheelReceiver {
  public:
    const static int WHEEL_FACTOR = 2;

    Car(std::string carFilename, Track* t) {
      /* move at initial position */
      track = t;
      pos_real_x = 200;
      pos_real_y = 200;
      pos_angle_deg = 0;

      /* init car_image */
      car_image = cv::imread(carFilename);
      //    cv::imshow("foo",car_image);
      //    cv::waitKey(5000);
    }

    ////////////////////////////////////////////////////////////////////////////

    void update_position() {
      /* test if we are in the grass */

      /* change the orientation of the car */
      double dt = last_update_time.getTimeSeconds();
      pos_angle_deg += get_wheel_angle() * dt / WHEEL_FACTOR;
      /* move the car */
      double variaX = get_speed_module() * dt * cos(pos_angle_deg * DEG2RAD);
      double variaY = get_speed_module() * dt * sin(pos_angle_deg * DEG2RAD);
      pos_real_x += variaX;
      pos_real_y += variaY;
      last_update_time.reset();
    }

    ////////////////////////////////////////////////////////////////////////////

    void draw_om_image(cv::Mat & img) {
      ROS_DEBUG("Car::draw_om_image()");
      //cout << vision_utils::infosImage( car_image ) << endl;

      /* rotate the car */
      //  cv2DRotationMatrix(cv::Point2f(car_image.cols/2, car_image.rows/2),//cv::Point2f center,
      //          0 * pos_angle * 180 / CV_PI, //double       pos_angle,
      //          1.0, //double       scale,
      //          rot_mat);
      //  cvGetQuadrangleSubPix(car_image, car_image_rotated, rot_mat);
      //  cv::cvtColor(car_image_rotated, car_image_rotated_BW, CV_RGB2GRAY);
      //  cout << cv::countNonZero( car_image_rotated_BW );
      //
      //  cout << vision_utils::infosImage( car_image_rotated_BW ) << endl;

      /* copy on screen */
      //vision_utils::drawCross( track->car_image, (int)pos_real_x, (int)pos_real_y, 100, CV_RGB(0,255,0) );
      for (short y = 0; y < car_image.rows; ++y) {
        for (short x = 0; x < car_image.cols; ++x) {
          short b = car_image(y, x)[0];
          short g = car_image(y, x)[1];
          short r = car_image(y, x)[2];
          if (r == 0 && g == 0 && b == 0)
            continue;

          short x2 = x - car_image.cols / 2;
          short y2 = y - car_image.rows / 2;
          double xR = cos(-pos_angle_deg* DEG2RAD) * x2 +
              sin(-pos_angle_deg * DEG2RAD) * y2
              + track->pos_screen_x + pos_real_x;
          double yR = -sin(-pos_angle_deg* DEG2RAD) * x2 +
              cos(-pos_angle_deg* DEG2RAD) * y2
              + track->pos_screen_y + pos_real_y;
          cv::circle(img, cv::Point((int) xR, (int) yR), 2, CV_RGB(r, g, b), -1);
        } // end loop x
      } // end loop y
    } // end draw_om_image();

    ////////////////////////////////////////////////////////////////////////////

    /* x position with the origin of the track */
    double pos_real_x;
    /* y position with the origin of the track */
    double pos_real_y;
    /* angle of the car */
    double pos_angle_deg;
    Track* track;
    cv::Mat3b car_image;
    Timer last_update_time;
  };

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  DrivingGame() : NanoSkill( "DRIVINGGAME_START", "DRIVINGGAME_STOP"){
    ROS_INFO( "DrivingGame CONSTRUCTOR" );
    /* load the track */
    track = new Track(IMG_DIR "driving/track_1.png");
    car = new Car(IMG_DIR "driving/car.png", track);
    GAME_SIZE = cv::Size(600, 600);
  }

  //////////////////////////////////////////////////////////////////////////////

  ~DrivingGame() {
    ROS_INFO( "DrivingGame DESTRUCTOR" );
    delete car;
    delete track;
  }

  //////////////////////////////////////////////////////////////////////////////

  void proceso() {
    //ROS_INFO("proceso()");

    /* compute the new positions */
    car->update_position();
    move_track();
    /* redisplay */
    track->draw_om_image(screen);
    car->draw_om_image(screen);

    //  cout << "track->pos_screen_x:" << track->pos_screen_x << endl;
    //  cout << "track->pos_screen_y:" << track->pos_screen_y << endl;
    //  cout << "car->pos_real_x:" << car->pos_real_x << endl;
    //  cout << "car->pos_real_y:" << car->pos_real_y << endl;
    //  cout << "car->speed_module:" << car->speed_module << endl;

    cv::imshow(window1Name, screen);

    char c = cv::waitKey(5);
    //cout << c << endl;
    if (c == 27)
      exit(0);
    /*if (c == 81)
          car->set_wheel_angle(car->get_wheel_angle() - 5);
      if (c == 83)
          car->set_wheel_angle(car->get_wheel_angle() + 5);
      if (c == 82)
          car->set_speed_module(car->get_speed_module() + 5);
      if (c == 84)
          car->set_speed_module(car->get_speed_module() - 5);*/
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    /* init params */
    screen.create(GAME_SIZE);

    /* create window */
    cvStartWindowThread();
    window1Name = "DrivingGame";
    cv::namedWindow(window1Name, CV_WINDOW_AUTOSIZE);
    cvMoveWindow(window1Name, 0, 0);

  }

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    cv::destroyWindow(window1Name);
  }

  //////////////////////////////////////////////////////////////////////////////

  void move_track() {
    //ROS_INFO("move_track()");

    /* move the screen */
    track->pos_screen_x = -car->pos_real_x + GAME_SIZE.width / 2;
    track->pos_screen_x = fmin(fmax(track->pos_screen_x,
                                    -track->track_image.cols + DrivingGame::GAME_SIZE.width), 0);

    //track->pos_screen_y -= variaY;
    track->pos_screen_y = -car->pos_real_y + GAME_SIZE.height / 2;
    track->pos_screen_y = fmin(fmax(track->pos_screen_y,
                                    -track->track_image.rows + DrivingGame::GAME_SIZE.height), 0);
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  cv::Size GAME_SIZE;
  cv::Mat3b screen;
  Track* track;
  Car* car;
  const char* window1Name;
}; // end DrivingGame

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "launcher_driving_game");
  DrivingGame skill;
  skill.check_autostart();
  ros::spin();
  return 0;
}

