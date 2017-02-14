/*!
  \file        drawer.h
  \author      Arnaud Ramey
                -- Robotics Lab, University Carlos III of Madrid
  \date        Dec 21, 2011
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

Implementation of the kid's game "red light, green light" for a
social robot with a kinect camera and making use of the interface People

\section Parameters
  None

\section Subscriptions
  None

\section Publications
  - \b "DRAWER_DRAWING_FINISHED"
        [std_msgs/Empty]
        Send this message to say a simple text with etts

  - \b "capacitive_touch"
        [std_msgs/String]
        When the drawer is finished, sends a fake touch message,
        containing "drawer".

 */
#ifndef DRAWER_H_
#define DRAWER_H_

#include "vision_utils/nano_skill.h"
#include <image_transport/image_transport.h>
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
// C++
#include <vector>

class Drawer: public vision_utils::NanoSkill {
public:
  // some definitions and aliases
  typedef cv::Scalar PenColor;
  typedef cv::Point2i PenPosition;
  typedef cv::Vec3b PixelColor; // BGR
  typedef cv::Mat3b Image;

  // default parameters
  const static int default_width;
  const static int default_height;
  const static int interface_width;
  const static int interface_height;
  const static PenColor default_pen_color;
  const static int default_pen_width;

  std::string WINDOW_NAME;

  //! Constructor
  Drawer();

  //! Destructor
  ~Drawer();

  /*! Launch the skill
  \param aux a pointer to this skill
  \param p ignored
  */
  void create_subscribers_and_publishers();

  /*! Stop the skill
  \param aux a pointer to this skill
  \param p ignored
  */
  void shutdown_subscribers_and_publishers();

private:

  /*!
  * Set a new background for the drawing
  \param new_background the background to replace
  It will be resized if needed
  */
  void set_new_background(const Image & new_background);
  void in_cb(const sensor_msgs::ImageConstPtr &msg);

  /*! erase the current drawing and restore it */
  void flush_current_drawing();

  void check_final_window_content_size();

  /*! the image shown to the user, where you have the pen painted.
  * it is a subimage of final_window_content */
  Image current_display();

  /*! the user interface
  * it is a subimage of final_window_content */
  Image interface();

  /*! redraw the buttons on the interface */
  void draw_interface();

  /* ****
  Communication parameters
  ****/
  /*!
  * transmit the image to the shared memory
  */
  void publish_drawing(bool remove_background);

  /*!
  Move the pen to a new position
  \param new_position
  */
  void move_pen(const PenPosition & new_position);

  /*! illustrate the pen position on an image */
  void draw_pen_position(Image & img);

  /*! sets the pen writing on the paper */
  void set_pen_pressed();

  /*! sets the pen not writing (floating in the air) */
  void set_pen_released();

  /*! sets the new pen width */
  void set_pen_width(const int & new_width);

  /*! sets the new pen color */
  void set_pen_color(const PenColor & new_color);

  /*! the OpenCV mouse callback */
  static void mouse_callback(int event, int x, int y, int flags, void* drawer_ptr);

  /* ****
  Pen parameters
  ****/
  /*! the background image */
  Image background;
  /*! the combination of the current display and the interface */
  Image final_window_content;

  /*! the pen position */
  PenPosition pen_position;
  /*! the pen position in the frame before */
  PenPosition last_pen_position;
  //! true if the pen is touching the paper
  bool is_pen_pressed;
  bool need_imshow;

  /*! the pen stroke width in pixels */
  int pen_width;

  /*! the pen color (RGB) */
  PenColor pen_color;

  image_transport::ImageTransport _it;
  image_transport::Subscriber _bg_sub;
  image_transport::Publisher _drawing_pub;
  cv_bridge::CvImage _drawing_msg;
  std::vector<ros::Publisher> _event_pubs;
  ros::Publisher _drawer_drawing_finished_pub;
};

#endif /* DRAWER_H_ */
