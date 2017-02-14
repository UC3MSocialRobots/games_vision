/*!
  \file        motion_video_recorder.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/31

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
 */

// C
#include <stdlib.h> // for atoi
// OpenCV
#include <opencv2/highgui/highgui.hpp>
// ROS msgs
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
// from http://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/
#include <image_transport/image_transport.h>
// AD
#include "vision_utils/timer.h"
#include "vision_utils/timestamp.h"
#include "vision_utils/depth_image_to_vizualisation_color_image.h"

// params
std::string _rgb_topic, _depth_topic, _video_name;
// 20 cm, rate of motion in image, max static time before stopping grabbing
double depth_diff_thres, motion_rate_thres, static_time_thres;
// decompression
int _frames_saved = 0, _queue_size = 1;
image_transport::ImageTransport* _it;
image_transport::Subscriber depth_sub, rgb_sub;
bool rgb_sub_active = false;
cv_bridge::CvImageConstPtr _rgb_ptr, _depth_ptr;
// depth comparing
cv::Mat1f _ref_depth, _depth_diff;
vision_utils::Timer _static_timer, _last_frame;
double _motion_rate;
// video writing
cv::Mat3b _collage, _collage_rgb, _collage_depth;
int fps, codec =
    // 0;
    //CV_FOURCC('I', '4', '2', '0'); // uncompressed
    CV_FOURCC('M', 'P', '4', '2');
//CV_FOURCC('M', 'J', 'P', 'G');
cv::VideoWriter _video_writer;

////////////////////////////////////////////////////////////////////////////////

inline bool need_save_depth(const cv::Mat1f & depth) {
  if (_ref_depth.empty()) {
    ROS_INFO("New ref depth");
    depth.copyTo(_ref_depth);
    return true;
  }

  // determine if image different from base
  int npixels_total = depth.cols * depth.rows, npixels = 0, npixels_motion = 0;
  assert(_ref_depth.isContinuous() && depth.isContinuous());
  const float *curr_data = depth.ptr<float>(), *ref_data = _ref_depth.ptr<float>();
  //cv::Mat1b depth_viz(depth.size(), (uchar) 0);
  for (int pixel = 0; pixel < npixels_total; ++pixel) {
    //if (!isnan(*curr_data) && !isnan(*ref_data)) {
    if (*curr_data && *ref_data) {
      ++npixels;
      if (fabs(*ref_data - *curr_data) > depth_diff_thres) {
        ++npixels_motion;
        // depth_viz.at<uchar>(pixel / depth.cols, pixel % depth.cols) = 255;
      }
    }
    ++curr_data;
    ++ref_data;
  } // end loop row
  _motion_rate = 1. * npixels_motion / npixels;
  //  double _motion_rate = vision_utils::rate_of_changes_between_two_images
  //      (depth, _ref_depth, _depth_diff, depth_diff_thres);
  //cv::imshow("depth_viz", depth_viz); cv::waitKey(10);

  if (_motion_rate > motion_rate_thres || _static_timer.getTimeSeconds() < static_time_thres) {
    ROS_INFO_THROTTLE(1, "fps:%i, motion rate:%f > thres:%g, "
                      "saving images (%i images saved, _static_timer:%g ms)",
                      (int) (1. / _last_frame.getTimeSeconds()),
                      _motion_rate, motion_rate_thres,
                      _frames_saved, _static_timer.getTimeSeconds());
    if (_motion_rate > motion_rate_thres)
      _static_timer.reset();
    return true;
  }
  ROS_INFO_THROTTLE(1, "fps:%i, motion rate:%f < thres:%g, "
                    "frame static for %g secs > thres=%g secs, skipping frames",
                    (int) (1. / _last_frame.getTimeSeconds()),
                    _motion_rate, motion_rate_thres,
                    _static_timer.getTimeSeconds(), static_time_thres);
  return false;
}

////////////////////////////////////////////////////////////////////////////////

inline void rgb_depth_process(const cv::Mat3b & rgb, const cv::Mat1f & depth) {
  // create collage if needed
  if (_collage.empty()) {
    cv::Size _video_size(std::max(rgb.cols, depth.cols), rgb.rows + depth.rows);
    _video_writer.open(_video_name, codec, fps, _video_size, true);
    assert(_video_writer.isOpened());
    _collage.create(_video_size);
    _collage.setTo(0);
    _collage_rgb = _collage(cv::Rect(0, 0, rgb.cols, rgb.rows));
    _collage_depth = _collage(cv::Rect(0, rgb.rows, depth.cols, depth.rows));
  }

  // sizes check
  if (_collage_depth.size() != depth.size()) {
    ROS_WARN("Sizes mismatch: _collage_depth(%i, %i), depth(%i, %i), skipping",
             _collage_depth.cols, _collage_depth.rows, depth.cols, depth.rows);
    return;
  }
  if (_collage_rgb.size() != rgb.size()) {
    ROS_WARN("Sizes mismatch: _collage_rgb(%i, %i), rgb(%i, %i), skipping",
             _collage_rgb.cols, _collage_rgb.rows, rgb.cols, rgb.rows);
    return;
  }

  // convert to visible
  vision_utils::depth_image_to_vizualisation_color_image
      (depth, _collage_depth, vision_utils::FULL_RGB_SCALED);
  rgb.copyTo(_collage_rgb);
  cv::putText(_collage, vision_utils::timestamp(), cv::Point(10, 20),
              CV_FONT_HERSHEY_PLAIN, 2, CV_RGB(0, 255, 0), 2);
  // cv::imshow("_collage", _collage); cv::waitKey(10);
  // save image
  //cv::imwrite(vision_utils::timestamp() + ".jpg", _collage);
  _video_writer << _collage;
  ++_frames_saved;
}

////////////////////////////////////////////////////////////////////////////////

void rgb_cb(const sensor_msgs::ImageConstPtr& rgb_msg) {
  try { // decompress RGB
    _rgb_ptr = cv_bridge::toCvShare(rgb_msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  rgb_depth_process(_rgb_ptr->image, _depth_ptr->image);
} // end rgb_depth_cb();

////////////////////////////////////////////////////////////////////////////////

void depth_cb(const sensor_msgs::ImageConstPtr& depth_msg) {
  try { // decompress depth
    _depth_ptr = cv_bridge::toCvShare(depth_msg);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  bool need_save = need_save_depth(_depth_ptr->image);
  if (need_save && !rgb_sub_active) {
    ROS_INFO("enable RGB subscriber");
    rgb_sub = _it->subscribe(_rgb_topic, _queue_size, rgb_cb);
    rgb_sub_active = true;
  }
  else if (!need_save && rgb_sub_active) {
    ROS_INFO("disable RGB subscriber");
    rgb_sub.shutdown();
    rgb_sub_active = false;
  }
  _last_frame.reset();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_video_recorder");
  ros::NodeHandle nh_public, nh_private("~");
  nh_private.param("rgb_topic", _rgb_topic, std::string("rgb"));
  nh_private.param("depth_topic", _depth_topic, std::string("depth"));
  _video_name = std::string("motion_video_recorder_") + vision_utils::timestamp() + ".avi";
  nh_private.param("video_name", _video_name, _video_name);
  nh_private.param("fps", fps, 6);
  nh_private.param("depth_diff_thres", depth_diff_thres, .2);
  nh_private.param("motion_rate_thres", motion_rate_thres, 1E-2);
  nh_private.param("static_time_thres", static_time_thres, 5.);
  ROS_INFO("motion_video_recorder:saving video:'%s', "
           "depth_diff_thres:%g, motion_rate_thres:%g, static_time_thres:%g",
           _video_name.c_str(), depth_diff_thres, motion_rate_thres, static_time_thres);

  // create subscribers
  _it = new image_transport::ImageTransport(nh_public);
  depth_sub = _it->subscribe(_depth_topic, _queue_size, depth_cb);
  ROS_INFO("Spinning...");
  ros::spin();
  delete _it;
}
