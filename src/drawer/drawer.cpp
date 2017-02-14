/*!
 * \file Drawer.cpp
 *
 * The implementation of drawer.h
 *
 * \date Dec 21, 2011
 * \author Arnaud Ramey
 */

#include "games_vision/drawer.h"
//
// ROS
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
// opencv
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

//#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
#define DEBUG_PRINT(...)   ROS_INFO(__VA_ARGS__)

const int Drawer::default_width = 500;
const int Drawer::default_height = 500;
const int Drawer::interface_width = 100;
const int Drawer::interface_height = 500;
const Drawer::PenColor Drawer::default_pen_color = CV_RGB(255, 0, 0); // red
const int Drawer::default_pen_width = 7;

Drawer::Drawer() :
  NanoSkill("DRAWER_START", "DRAWER_STOP"),
  _it(_nh_private){
  DEBUG_PRINT("Drawer ctor");

  // get events to send
  _drawer_drawing_finished_pub = _nh_public.advertise<std_msgs::Empty>("DRAWER_DRAWING_FINISHED", 1);

  // reset the background
  Image default_background(default_width, default_height, PixelColor(230, 230, 230));
  set_new_background( default_background );

  set_pen_released();
  set_pen_width(default_pen_width);
  set_pen_color(default_pen_color);  // redraw

  WINDOW_NAME = "Drawer";
}

////////////////////////////////////////////////////////////////////////////////

Drawer::~Drawer() {
  DEBUG_PRINT("dtor");
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::create_subscribers_and_publishers() {
  DEBUG_PRINT("create_subscribers_and_publishers()");
  // subscribe to the background topic
  std::string in_topic = "in", out_topic = "out";
  _nh_private.param("in_topic", in_topic, in_topic);
  _nh_private.param("out_topic", out_topic, out_topic);
  _bg_sub = _it.subscribe(in_topic, 1, &Drawer::in_cb, this);
  _drawing_pub = _it.advertise(out_topic, 1);

  // create the window
  cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

  // set the mouse callback - passing the class as parameter
  cvSetMouseCallback(WINDOW_NAME.c_str(), mouse_callback, this);

  // redraw the buttons
  draw_interface();
  flush_current_drawing();

  printf("Drawer: getting backgrounds on '%s', publishing drawings on '%s'\n",
         _bg_sub.getTopic().c_str(), _drawing_pub.getTopic().c_str());

  while(ros::ok()) {
      ros::spinOnce();
      if (need_imshow) {
          cv::imshow(WINDOW_NAME, final_window_content);
          need_imshow = false;
        }
      // wait more if the pen is not pressed
      int time_sleep = (is_pen_pressed ? 20 : 100);
      char c = cv::waitKey(time_sleep);
      if (c  == 27)
        exit(0);
    } // end while (ros::ok())
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::shutdown_subscribers_and_publishers() {
  DEBUG_PRINT("shutdown_subscribers_and_publishers()");
  cv::destroyWindow(WINDOW_NAME);
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::check_final_window_content_size() {
  DEBUG_PRINT("check_final_window_content_size()");
  //    DEBUG_PRINT("background:'%s', interface_width:%i, interface_height:%i",
  //                 vision_utils::infosImage(background).c_str(),
  //                 interface_width, interface_height);
  int new_ncols = background.cols + interface_width;
  int new_nrows = (background.rows > interface_height ? background.rows : interface_height);
  bool is_size_changed = (final_window_content.cols != new_ncols) ||
      (final_window_content.rows != new_nrows);
  final_window_content.create(new_nrows, new_ncols);
  if (is_size_changed)
    draw_interface();
  //    DEBUG_PRINT("new_width:%i, new_height:%i", new_width, new_height);
  //    DEBUG_PRINT("final_window_content:'%s'",
  //                 vision_utils::infosImage(final_window_content).c_str());
}

////////////////////////////////////////////////////////////////////////////////

Drawer::Image Drawer::current_display() {
  //DEBUG_PRINT("current_display()");
  return Image(final_window_content,
               cv::Rect(0, 0, background.cols, background.rows));
}

////////////////////////////////////////////////////////////////////////////////

Drawer::Image Drawer::interface() {
  //DEBUG_PRINT("interface()");
  return Image(final_window_content,
               cv::Rect(background.cols, 0, interface_width, interface_height));
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::draw_interface() {
  DEBUG_PRINT("draw_interface()");
  Image inter = interface();
  // red zone on the upper half
  cv::rectangle(inter,
                cv::Point(0, 0), cv::Point(inter.cols, inter.rows / 2),
                CV_RGB(255, 100, 100), -1);
  // green zone on the lower half
  cv::rectangle(inter,
                cv::Point(0, inter.rows / 2), cv::Point(inter.cols, inter.rows),
                CV_RGB(100, 255, 100), -1);
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::set_pen_color(const PenColor & new_color) {
  DEBUG_PRINT("set_pen_color(%f, %f, %f)",
              new_color.val[0], new_color.val[1], new_color.val[2]);
  pen_color = new_color;
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::set_new_background(const Image & new_background) {
//  DEBUG_PRINT("set_new_background(%s)",
//              vision_utils::infosImage(new_background).c_str());
  //    cv::imshow("new_background", new_background);
  //    cv::waitKey(5000);

  new_background.copyTo( background );
  check_final_window_content_size();
  // redraw with the new background
  flush_current_drawing();
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::in_cb(const sensor_msgs::ImageConstPtr & msg) {
  DEBUG_PRINT("new_background_cb()");
  cv_bridge::CvImageConstPtr img = cv_bridge::toCvShare(msg);
  Image new_bg;
  img->image.copyTo(new_bg);
  set_new_background(new_bg);
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::set_pen_released() {
  DEBUG_PRINT("set_pen_released()");
  is_pen_pressed = false;
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::publish_drawing(bool remove_background) {
  DEBUG_PRINT("publish_drawing(remove_background:%i) - MCP_address:'%s'",
              remove_background, _drawing_pub.getTopic().c_str());

  // remove the background if wanted
  // rows, cols

  if (remove_background) {
      Image current_disp = current_display();
      Image current_background_without_bg (background.rows, background.cols,
                                           PixelColor(0,0,0));
      for (int col = 0; col < background.cols; ++col) {
          for (int row = 0; row < background.rows; ++row) {
              // copy the background if the drawing is a black pixel
              if (current_disp(col, row) == background(col, row))
                // black pixel
                current_background_without_bg(col, row) = PixelColor(0, 0, 0);
              else // otherwise copy the drawing
                current_background_without_bg(col, row) = current_disp(col, row);
            } // end loop row
        } // end loop col
      current_background_without_bg.copyTo(_drawing_msg.image);
    } // end if remove_background
  else { // remove_background = false
      // we make a copy of the data to have something continuous
      current_display().copyTo(_drawing_msg.image);
      //_drawing_msg.image = current_display();
    }

  //cv::imshow("drawerpz", _drawing_msg.image); cv::waitKey(5000);

  // publish the image
  _drawing_msg.encoding = sensor_msgs::image_encodings::BGR8;
  _drawing_msg.header.stamp = ros::Time::now();
  _drawing_pub.publish(_drawing_msg.toImageMsg());

  // send the events
  std_msgs::Empty empty;
  _drawer_drawing_finished_pub.publish(empty);
} // end publish_drawing

////////////////////////////////////////////////////////////////////////////////

void Drawer::set_pen_pressed() {
  DEBUG_PRINT("set_pen_pressed()");
  is_pen_pressed = true;
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::move_pen(const PenPosition & new_position) {
  //DEBUG_PRINT("move_pen(x:%i, y:%i)", new_position.x, new_position.y);
  last_pen_position = pen_position;
  pen_position = new_position;
  if (is_pen_pressed) {
      Image current = current_display();
      cv::line(current, last_pen_position, pen_position, pen_color, pen_width);
      need_imshow = true;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::draw_pen_position(Image & img) {
//  DEBUG_PRINT("draw_pen_position(%s)",
//              vision_utils::infosImage(img).c_str());
  // draw a cross to show the pen
  cv::Scalar color (0, 255, 0);
  // horizontal line
  cv::line( img,
            cv::Point(0, pen_position.y),
            cv::Point(background.cols, pen_position.y),
            color, 2);
  // vertical line
  cv::line( img,
            cv::Point(pen_position.x, 0),
            cv::Point(pen_position.x, background.rows),
            color, 2);
  need_imshow = true;
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::set_pen_width(const int & new_width) {
  DEBUG_PRINT("set_pen_width(%i)", new_width);
  pen_width = new_width;
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::flush_current_drawing() {
  DEBUG_PRINT("flush_current_drawing()");
  //current_drawing = Image(background.cols, background.rows, PixelColor(0,0,0));
  Image current = current_display();
  background.copyTo( current );
  need_imshow = true;
}

////////////////////////////////////////////////////////////////////////////////

void Drawer::mouse_callback(int event, int x, int y, int , void* drawer_ptr) {
  Drawer* drawer = (Drawer*) drawer_ptr;
  // left button down -> pen pressed
  if (event == CV_EVENT_MOUSEMOVE)
    drawer->move_pen(PenPosition(x, y));

  // left button down -> pen pressed
  else if (event == CV_EVENT_LBUTTONDOWN) {
      if ( x < drawer->background.cols ) {
          drawer->set_pen_pressed();
        }
      else {
          // red zone
          if ( y <= drawer->final_window_content.rows / 2)
            drawer->flush_current_drawing();

          // green zone
          else
            drawer->publish_drawing(false);
        } // end if x < background width
    } // end if left button

  // left button down -> pen pressed
  else if (event == CV_EVENT_LBUTTONUP)
    drawer->set_pen_released();
}
