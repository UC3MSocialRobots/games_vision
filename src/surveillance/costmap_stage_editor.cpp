/*!
  \file        costmap_stage_editor.cpp
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

\brief An editor of costmaps based on MiniStage visualizer.
It also displays the detected poses of the users.

\section Parameters
  - \b "costmap_topic"
        [string] (default: "costmap")
        Where the costmaps will both be obtained and republished.

  - \b "frame_id"
        [string] (default: "/odom")
        The default frame of the costmap.

  - \b "~ppl_input_topics"
        [string] (default: "ppl")
        The list of topics where to get the detections of the users
        ( of types people_msgs/People ).
        Topics must be separated by ";",
        for example "/foo;/bar".

\section Subscriptions
  - \b {costmap_topic}
        [nav_msgs::GridCells]
        The costmap we will both subscribe to and republish.

  - \b {ppl_input_topics}
        [people_msgs/People]
        The different input methods for people pose lists

\section Publications
  - \b {costmap_topic}
        [nav_msgs::GridCells]
        The costmap we will both subscribe to and republish.
 */

// ROS
#include <tf/transform_listener.h>
// people_msgs
#include <people_msgs/People.h>
// vision_utils
#include "cvstage/cvstage.h"
#include "cvstage/plugins/draw_costmap.h"
#include "vision_utils/drawCross.h"
#include "vision_utils/is_point_in_costmap.h"
#include "vision_utils/multi_subscriber.h"
#include "vision_utils/nano_skill.h"
#include "vision_utils/ppl_tf_utils.h"
#include "vision_utils/toggle_point_in_costmap.h"

class CostmapStageEditor : public vision_utils::NanoSkill {
public:
  CostmapStageEditor()
    : NanoSkill("COSTMAP_STAGE_EDITOR_START", "COSTMAP_STAGE_EDITOR_STOP") {
    // get params
    _costmap_topic = "costmap";
    _nh_private.param("costmap_topic", _costmap_topic, _costmap_topic);
    _frame_id = "/odom";
    _nh_private.param("frame_id", _frame_id, _frame_id);
    _ppl_input_topics = "ppl";
    _nh_private.param("ppl_input_topics", _ppl_input_topics, _ppl_input_topics);
    _win_name = "CostmapStageEditor";
    // init map
    _map.cell_width = .5;
    _map.cell_height = .5;
    _map.header.frame_id = _frame_id;
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  ~CostmapStageEditor() {
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    // printf("CostmapStageEditor:create_subscribers_and_publishers()\n");
    _map_sub = _nh_public.subscribe(_costmap_topic, 1,
                                   &CostmapStageEditor::costmap_cb, this);
    _map_pub = _nh_public.advertise<nav_msgs::GridCells>(_costmap_topic, 1, true);
    _ppl_subs = vision_utils::MultiSubscriber::subscribe
        (_nh_public, _ppl_input_topics, 1,
         &CostmapStageEditor::ppl_cb, this);
    ROS_WARN("CostmapStageEditor: "
             "subscribing to and republising costmaps on '%s'. "
             "Getting People on '%s'. "
             "Frame ID:'%s'",
             _map_sub.getTopic().c_str(), _ppl_subs.getTopics().c_str(),
             _frame_id.c_str());
    // init mini stage
    cv::namedWindow(_win_name);
    cv::setMouseCallback(_win_name, mouse_cb, this);
    _ms.set_dims(800, 400);
    redraw_ministage_display();
  } // end create_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

  void display_interface() {
    // printf("CostmapStageEditor:display_interface()\n");
    ROS_INFO_ONCE("display_interface()");
    cv::imshow(_win_name, _ms.get_viz());
    char c = cv::waitKey(100);
    if ((int) c == 27) {
      printf("CostmapStageEditor:blocking()\n");
      stop();
    }
  } // end display_interface()

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers()  {
    // printf("CostmapStageEditor:shutdown_subscribers_and_publishers()\n");
    _map_pub.shutdown();
    _map_sub.shutdown();
    _ppl_subs.shutdown();
    cv::destroyWindow(_win_name);
    cv::destroyAllWindows();
    cv::waitKey(5);
  } // end shutdown_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

  inline void redraw_ministage_display() {
    //ROS_INFO_THROTTLE(1, "redraw_ministage_display()\n");
    ROS_INFO_ONCE("redraw_ministage_display()");
    // draw axes and stuff
    _ms.clear();
    // _ms.set_heading(-M_PI_2, false);
    cvstage_plugins::draw_costmap(_ms, _map, _map_to_corners);
    _ms.draw_grid(_map.cell_width, 150);
    _ms.draw_axes();
    // draw people
    cv::Scalar people_color = CV_RGB(255, 0, 0);
    for (unsigned int people_idx = 0; people_idx < _ppl.people.size(); ++people_idx) {
      geometry_msgs::Point pose = _ppl.people[people_idx].position;
      vision_utils::drawCross(_ms.get_viz(), _ms.world2pixel(pose),
                             3, people_color, 2);
      cv::putText(_ms.get_viz(), _ppl.people[people_idx].name,
                  _ms.world2pixel(pose) + cv::Point(10, 0),
                  CV_FONT_HERSHEY_PLAIN, 1, people_color);
    } // end loop people_idx
  } // end redraw_ministage_display();

  //////////////////////////////////////////////////////////////////////////////

  static inline void mouse_cb(int event, int x, int y, int /*flags*/, void* param) {
    CostmapStageEditor* this_ptr = ((CostmapStageEditor*) param);
    bool need_to_redraw = this_ptr->_ms.mouse_move_callback(event, x, y);
    // convert to 3D
    cv::Point2f pt_world = this_ptr->_ms.pixel2world(x, y);
    bool was_map_changed = false;
    if (event == CV_EVENT_LBUTTONDOWN) {
      vision_utils::toggle_point_in_costmap(pt_world, this_ptr->_map);
      was_map_changed = true;
    }
    else if (event == CV_EVENT_MBUTTONDOWN) {
      this_ptr->_map.cells.clear();
      was_map_changed = true;
    }
    // send new map if changed
    if (was_map_changed) {
      this_ptr->_map.header.stamp = ros::Time::now();
      this_ptr->_map_pub.publish(this_ptr->_map);
    }

    if (need_to_redraw && !was_map_changed) // if (was_map_changed), will be redrawn in callback
      this_ptr->redraw_ministage_display();
  } // end mouse_cb()

  //////////////////////////////////////////////////////////////////////////////

  void costmap_cb(const nav_msgs::GridCellsConstPtr & map_msg) {
    // ROS_INFO_THROTTLE(1, "costmap_cb()");
    ROS_INFO_ONCE("costmap_cb()");
    if (_frame_id != map_msg->header.frame_id) {
      ROS_WARN("received costmap frame '%s' differs from MiniStage frame '%s', discarding it.",
               map_msg->header.frame_id.c_str(), _frame_id.c_str());
      return;
    }
    _map = *map_msg;
    redraw_ministage_display();
  }

  //////////////////////////////////////////////////////////////////////////////

  void ppl_cb(const people_msgs::PeopleConstPtr & ppl) {
    // printf("CostmapStageEditor:ppl_cb(%i people)\n", ppl->people.size());
    ROS_INFO_ONCE("CostmapStageEditor:ppl_cb(%li people)", ppl->people.size());
    _ppl = *ppl;
    // convert frame if needed
    bool success = vision_utils::convert_ppl_tf
        (_ppl, _frame_id, _tf_listener);
    // check it is the same frame
    if (!success) {
      ROS_WARN("Cant transform People from frame %s to the map frame '%s'. "
               "Skipping pose.",
               ppl->header.frame_id.c_str(), _frame_id.c_str());
      _ppl.people.clear();
      return;
    }
    redraw_ministage_display();
  } // end ppl_cb();

  //////////////////////////////////////////////////////////////////////////////

protected:
  nav_msgs::GridCells _map;
  std::vector<cv::Point3f> _map_to_corners;

  std::string _costmap_topic;
  ros::Publisher _map_pub;
  ros::Subscriber _map_sub;

  std::string _ppl_input_topics;
  vision_utils::MultiSubscriber _ppl_subs;
  people_msgs::People _ppl;
  MiniStage _ms;
  std::string _frame_id;
  tf::TransformListener _tf_listener;

  std::string _win_name;
}; // end CostmapStageEditor

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "costmap_stage_editor");
  CostmapStageEditor skill;
  ros::Rate rate(10);
  while (ros::ok()) {
    if (skill.is_running())
      skill.display_interface();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

