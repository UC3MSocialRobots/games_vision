/*!
  \file        costmap_watcher.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/3/10

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

\class CostmapWatcher
\brief A vision skill that checks if a user enters a forbidden zone.
In that case, it publishes an alert message.

\section Parameters
  - \b "~costmap_topic"
        [string] (default: "costmap")
        Where the costmaps will both be obtained and republished.

  - \b "frame_id"
        [string] (default: "/odom")
        The default frame of the costmap.
        If maps are received on topic {costmap_topic}, it will be overwritten.

  - \b "~ppl_input_topics"
        [string] (default: "ppl")
        The list of topics where to get the detections of the users
        ( of types people_msgs/People ).
        Topics must be separated by ";",
        for example "/foo;/bar".

\section Subscriptions
  - \b {costmap_topic}
        [nav_msgs::GridCells]
        The costmap we will subscribe to.

  - \b {ppl_input_topics}
        [people_msgs/People]
        The different input methods for people pose lists

\section Publications
  - \b {costmap_topic}
        [nav_msgs::GridCells]
        The costmap we will both subscribe to and republish.
        In the actual version, no costmap is published back.

  - \b "alert_messages"
        [games_vision::AlertMessage]
        The alert messages we publish when someone enters a forbidden zone.

 */

// ROS
#include <tf/transform_listener.h>
// AD
//#include "skill_templates/ros_vision_skill.h"
#include "vision_utils/rgb_depth_skill.h"
#include "vision_utils/multi_subscriber.h"
#include "vision_utils/timer.h"
#include "games_vision/AlertMessage.h"
// compressed_rounded_image_transport
#include "vision_utils/sensor_cv_encodings_bridge.h"
#include "vision_utils/nan_handling.h"
// people_msgs
#include "people_msgs/People.h"
#include "vision_utils/draw_ppl_on_image.h"


class CostmapWatcher : public RgbDepthSkill {
public:
  CostmapWatcher() :
    RgbDepthSkill("COSTMAP_WATCHER_START", "COSTMAP_WATCHER_STOP")
  {
    _window_name = "CostmapWatcher";

    ros::NodeHandle  _nh_private("~");
    // get params
    costmap_topic = "costmap";
    _nh_private.param("costmap_topic", costmap_topic, costmap_topic);
    ppl_input_topics = "ppl";
    _nh_private.param("ppl_input_topics", ppl_input_topics, ppl_input_topics);
    _last_map.header.frame_id = "/odom";
    _nh_private.param("frame_id", _last_map.header.frame_id, _last_map.header.frame_id);

    // init map
    _last_map.cell_width = 1;
    _last_map.cell_height = 1;
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    ROS_WARN("create_subscribers_and_publishers()");
    // subscribers
    _map_sub = _nh_public.subscribe(costmap_topic, 1, &CostmapWatcher::costmap_cb, this);
    _ppl_subs = ros::MultiSubscriber::subscribe
        (_nh_public, ppl_input_topics, 10,
         &CostmapWatcher::ppl_cb, this);

    // publishers
    _map_pub = _nh_public.advertise<nav_msgs::GridCells>(costmap_topic, 1, true);
    _alert_message_pub = _nh_public.advertise
        <games_vision::AlertMessage>("alert_messages", 1);
    ROS_WARN("CostmapWatcher: "
             "getting RGB on '%s', depth on  '%s', "
             "subscribing to and republising costmaps on '%s', "
             "Publishing alert messages on: '%s'. ",
             get_rgb_topic().c_str(), get_depth_topic().c_str(),
             _map_sub.getTopic().c_str(),
             _alert_message_pub.getTopic().c_str());

    DISPLAY = true; // get as a param?
    if (DISPLAY) {
      cv::namedWindow(_window_name);
      cv::setMouseCallback(_window_name, mouse_cb, this);
    }
  } // end create_subscribers_and_publishers();

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    ROS_WARN("shutdown_subscribers_and_publishers()");
    if (DISPLAY) {
      cv::destroyWindow(_window_name);
    }
  } // end shutdown_subscribers_and_publishers();

  //////////////////////////////////////////////////////////////////////////////

  void process_rgb_depth(const cv::Mat3b & rgb,
                         const cv::Mat1f & depth) {
    //ROS_INFO_THROTTLE(1, "process_rgb_depth(), time since last:%g ms", timer.getTimeMilliseconds());
    ROS_INFO_ONCE("process_rgb_depth(), time since last:%g ms", timer.getTimeMilliseconds());
    timer.reset();

    if (DISPLAY) {
      rgb.copyTo(_last_rgb);
      depth.copyTo(_last_depth);
      refresh_display();
      int key_code = (char) cv::waitKey(2);
      if (key_code == 27)
        exit(0);
    }
  } // end process_rgb_depth()

  //////////////////////////////////////////////////////////////////////////////

  void ppl_cb(const people_msgs::PeopleConstPtr & ppl) {
    // do not check user in costmap if list empty
    if (ppl->people.size() == 0) {
      // clear PPL for drawing
      _ppl_image_frame.poses.clear();
      return;
    }

    // convert people pose list to image frame (used for drawing)
    _ppl_image_frame = *ppl;
    std::string image_frame = _images_header.frame_id;
    bool success = vision_utils::convert_ppl_tf
        (_ppl_image_frame, image_frame, tf_listener);
    if (!success) {
      ROS_WARN("Cant transform People from frame %s to the image frame '%s'. "
               "Skipping pose.",
               ppl->header.frame_id.c_str(), image_frame.c_str());
      return;
    }

    // convert ppl to map frame if needed
    _ppl_map_frame = *ppl;
    std::string map_frame = _last_map.header.frame_id;
    success = vision_utils::convert_ppl_tf
        (_ppl_map_frame, map_frame, tf_listener);
    if (!success) {
      ROS_WARN("Cant transform People from frame %s to the map frame '%s'. "
               "Skipping pose.",
               ppl->header.frame_id.c_str(), map_frame.c_str());
      return;
    }

    // check if there is any people in the forbidden zone
    for (unsigned int people_idx = 0; people_idx < _ppl_map_frame.poses.size(); ++people_idx) {
      const people_msgs::Person* curr_pose = &(_ppl_map_frame.poses[people_idx]);
      // do nothing if point not in costmap
      bool is_user_in_forbidden_area = vision_utils::is_point_in_costmap
          (curr_pose->position, _last_map);
      if (!is_user_in_forbidden_area)
        continue;

      // send an alert message!
      ros::Duration time_since_last_alert = curr_pose->header.stamp - _last_alert_stamp;
      if (time_since_last_alert.toSec() < 5) {
        ROS_WARN("There is a person in the forbidden part of the map, "
                 "but last alert was given %g seconds ago. Skipping.",
                 time_since_last_alert.toSec());
        continue;
      }
      _last_alert_stamp = curr_pose->header.stamp;
      ROS_WARN("The people pose (%g, %g, %g) is in the forbidden part of the map. Alerting.",
               curr_pose->position.x,
               curr_pose->position.y,
               curr_pose->position.z);

      games_vision::AlertMessage alert_msg;
      alert_msg.header = curr_pose->header;
      alert_msg.message = "A person has entered the forbidden part of the map!";
      alert_msg.priority = games_vision::AlertMessage::PRIORITY_WARN;
      alert_msg.etts_sentence = "|en:Do not go there!"
                                "|en:You can't go there."
                                "|en:This is a forbidden zone."
                                "|en:Come back! Nothing interesting there."
                                "|en:It is definitely not a good idea to go there."

                                "|es:No te alejes. No me dejes sola!"
                                "|es:Adonde vas?"
                                "|es:No puedes ir alli, vuelve por favor."
                                "|es:No deberias atreverte alli."
                                "|es:No me dejes solita, vente por favor!"
                                "|es:No es una buena idea irte por alli.";
      // load and convert imaghe
      cv_bridge::CvImage cv_image;
      _last_rgb.copyTo(cv_image.image);
      cv_image.encoding = sensor_encoding_from_cv_encoding(cv_image.image.type());
      cv_image.toImageMsg(alert_msg.image);
      _alert_message_pub.publish(alert_msg);
    } // end loop people_idx
  } // end ppl_cb();

  //////////////////////////////////////////////////////////////////////////////

  void costmap_cb(const nav_msgs::GridCellsConstPtr & map_msg) {
    ROS_INFO_THROTTLE(1, "costmap_cb()");
    _last_map = *map_msg;
    if (DISPLAY)
      refresh_display();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! check if the polygon makes sense
  template<class Point2>
  bool is_polygon_in_bounds(const std::vector<Point2> & poly,
                            const int xmin, const int xmax,
                            const int ymin, const int ymax) {
    for (unsigned int pt_idx = 0; pt_idx < poly.size(); ++pt_idx) {
      if (poly[pt_idx].x < xmin
          || fabs(poly[pt_idx].x) > xmax
          || poly[pt_idx].y < ymin
          || fabs(poly[pt_idx].y) > ymax) {
        ROS_WARN_THROTTLE(1, "Dismissing polygon '%s'",
                          vision_utils::accessible_to_string(poly).c_str());
        return false;
      }
    } // end loop pt
    return true;
  } // end is_polygon_in_bounds()

  //////////////////////////////////////////////////////////////////////////////

  void refresh_display() {
    if (!DISPLAY)
      return;
    // ROS_INFO("refresh_display()");

    _last_rgb.copyTo(frame_out);

    // get corners of the cell
    vision_utils::costmap_to_polygon_list(_last_map, _map_to_corners);
    //    ROS_WARN("_map.size():%i, _map_to_corners.size():%i",
    //             _map.cells.size(), _map_to_corners.size());

    // the map frame is most likely '/odom'
    // the camera frame is most likely like "/<robot>_rgb_optical_frame"
    // it is good for converting to pixel with world2pixel_rgb()
    std::string map_frame = _last_map.header.frame_id,
        camera_frame = _images_header.frame_id;
    //  ROS_WARN("map_frame:'%s' -> camera_frame:'%s'",
    //           map_frame.c_str(), camera_frame.c_str());
    //ros::Time costmap_stamp = ros::Time(0); // -> get some timeout too
    // ros::Time costmap_stamp = _map.header.stamp; // -> this will get obsolete at some point
    ros::Time costmap_stamp = _images_header.stamp; // -> get some timeout too
    geometry_msgs::PointStamped pt_map_frame, pt_cam_frame;
    pt_map_frame.header.frame_id = map_frame;
    pt_map_frame.header.stamp = costmap_stamp;
    cv::Point3d pt_cam_frame_cv;
    std::string tf_error_msg;
    bool transform_ok = tf_listener.waitForTransform
        (camera_frame, map_frame, costmap_stamp, ros::Duration(1),
         ros::Duration(0.01), &tf_error_msg);
    if (!transform_ok) {
      ROS_WARN("Impossible to find tf '%s' -> %s':'%s'. Returning.",
               map_frame.c_str(), camera_frame.c_str(), tf_error_msg.c_str());
      cv::imshow(_window_name, frame_out);
      return;
    }

    std::vector< std::vector<cv::Point2i> > polygons;
    polygons.reserve(_last_map.cells.size());
    /* corners A B C D T , in the same order as costmap_to_polygon_list()
       y ^
         |  D     C
         |     T
         |  A     B
        0+---------> x
     */
    std::vector<cv::Point2i> one_polygon_corners;
    one_polygon_corners.resize(5);
    std::vector<cv::Point2i> one_polygon; // sequence ABCDATCBTD
    one_polygon.resize(10);

    for (unsigned int cell_idx = 0; cell_idx < _last_map.cells.size(); ++cell_idx) {
      try {
        for (unsigned int corner_idx = 0; corner_idx < 5; ++corner_idx) {
          // determine polygon corners A B C D T in map frame
          if (corner_idx == 4) { // T
            pt_map_frame.point = _last_map.cells[cell_idx]; // cell center
            pt_map_frame.point.z += 1; // 1 meter higher in map frame
          }
          else
            pt_map_frame.point = _map_to_corners[4 * cell_idx + corner_idx];
          // convert it to camera frame
          tf_listener.transformPoint(camera_frame, costmap_stamp, pt_map_frame,
                                     camera_frame, pt_cam_frame);
          vision_utils::copy3(pt_cam_frame.point, pt_cam_frame_cv);
          // and then project it to pixel
          one_polygon_corners[corner_idx] = world2pixel_rgb(pt_cam_frame_cv);
          //  ROS_WARN("#%i:pt_map_frame:%s, pt_cam_frame_cv:%s, one_polygon[]:%s",
          //           corner_idx,
          //           vision_utils::printP(pt_map_frame.point).c_str(),
          //           vision_utils::printP(pt_cam_frame_cv).c_str(),
          //           vision_utils::printP2(one_polygon_corners[corner_idx]).c_str());
        } // end loop corner_idx

      } catch (tf::TransformException e) {
        ROS_WARN("Exception while converting costmap from '%s' to '%s' ':%s",
                 map_frame.c_str(), camera_frame.c_str(), e.what());
        continue;
      }
      // now make sequence ABCDATCBTD  (A0 B1 C2 D3 T4)
      one_polygon[0] = one_polygon_corners[0]; // A
      one_polygon[1] = one_polygon_corners[1]; // B
      one_polygon[2] = one_polygon_corners[2]; // C
      one_polygon[3] = one_polygon_corners[3]; // D
      one_polygon[4] = one_polygon_corners[0]; // A
      one_polygon[5] = one_polygon_corners[4]; // T
      one_polygon[6] = one_polygon_corners[2]; // C
      one_polygon[7] = one_polygon_corners[1]; // B
      one_polygon[8] = one_polygon_corners[4]; // T
      one_polygon[9] = one_polygon_corners[3]; // D

      // dismiss polygon if too big
      if (is_polygon_in_bounds(one_polygon, -2000, 2000, -2000, 2000))
        polygons.push_back(one_polygon);
    } // end for (cell_idx)

    // draw polygons
    // cv::fillPoly(frame_out, polygons, inner_color);
    // cv::Scalar inner_color = CV_RGB(255, 200, 200);
    cv::Scalar edge_color = CV_RGB(255, 0, 0);
    int edge_width = 3;
    cv::polylines(frame_out, polygons, true, edge_color, edge_width);

    // draw found users
    vision_utils::draw_ppl_on_image
        (_ppl_image_frame, _default_rgb_camera_model, frame_out, CV_RGB(255, 0, 0), 3);

    // show frame_out
    cv::imshow(_window_name, frame_out);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void mouse_cb(int event, int x, int y, int flags, void* cookie) {
    // printf("mouse_cb(event:%i, x:%i, y:%i)\n", event, x, y);
    if (event == cv::EVENT_MBUTTONDOWN)
      ((CostmapWatcher*) cookie)->clear_map();
    else if (event == cv::EVENT_LBUTTONDOWN)
      ((CostmapWatcher*) cookie)->click(x, y);
  }

  //////////////////////////////////////////////////////////////////////////////

  void clear_map() {
    _last_map.cells.clear();
    // republish map
    _map_pub.publish(_last_map);
  }

  //////////////////////////////////////////////////////////////////////////////


  void click(int x, int y) {
    // get frames info
    std::string map_frame = _last_map.header.frame_id,
        cam_frame = _images_header.frame_id;
    ros::Time stamp = _images_header.stamp;
    geometry_msgs::PointStamped pt_map_frame, pt_cam_frame;
    pt_cam_frame.header.frame_id = cam_frame;
    pt_cam_frame.header.stamp = stamp;
    // get 3D point in camera frame
    cv::Point3d pt3Dcv_cam_frame = pixel2world_rgb(cv::Point(x, y));
    printf("pt3Dcv_cam_frame:%s\n", vision_utils::print_point(pt3Dcv_cam_frame).c_str());
    if (vision_utils::is_nan_pt(pt3Dcv_cam_frame)) {
      printf("pt3Dcv_cam_frame %s is NaN, reproj error, skipping\n",
             vision_utils::print_point(pt3Dcv_cam_frame).c_str());
      return;
    }
    vision_utils::copy3(pt3Dcv_cam_frame, pt_cam_frame.point);

    // convert cam frame -> map frame
    try {
      tf_listener.transformPoint(map_frame, stamp, pt_cam_frame, map_frame, pt_map_frame);
    }
    catch (tf::TransformException e) {
      printf("Transform exception for clicked point from cam frame to map frame:"
             "'%s'\n", e.what());
      return;
    }

    printf("pt_map_frame:%s\n", vision_utils::print_point(pt_map_frame.point).c_str());
    if (vision_utils::is_nan_pt(pt_map_frame.point)) {
      printf("pt_map_frame %s is NaN, reproj error, skipping\n",
             vision_utils::print_point(pt_map_frame.point).c_str());
      return;
    }

    vision_utils::toggle_point_in_costmap(pt_map_frame.point, _last_map);
    // republish map
    _map_pub.publish(_last_map);
  }

  //////////////////////////////////////////////////////////////////////////////

private:
  bool DISPLAY;
  cv::Mat3b _last_rgb;
  cv::Mat1f _last_depth;
  cv::Mat3b frame_out;
  std::string _window_name;

  std::string costmap_topic;
  nav_msgs::GridCells _last_map;
  std::vector<geometry_msgs::Point> _map_to_corners;
  ros::Publisher _map_pub;
  ros::Subscriber _map_sub;

  std::string ppl_input_topics;
  ros::MultiSubscriber _ppl_subs;
  people_msgs::People _ppl_map_frame;
  people_msgs::People _ppl_image_frame;

  ros::Publisher _alert_message_pub;
  ros::Time _last_alert_stamp;

  tf::TransformListener tf_listener;
  vision_utils::Timer timer;
}; // end class CostmapWatcher

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "costmap_watcher");
  CostmapWatcher skill;
  skill.check_autostart();
  ros::spin();
  return 0;
}
