/*!
  \file        red_light_green_light_skill.h
  \author      Irene Pérez, Arnaud Ramey
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/13
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
  - \b "~ppl_topic"
        [string] (default: "ppl")
        The topic where to get the detections/recognitions of the users
        ( of type people_msgs/People ).

  - \b "~display"
        [bool, default false]
        True to call the display function upon receiving the images.

\section Subscriptions
  - \b "RED_LIGHT_GREEN_LIGHT_SKILL_START"
        [std_msgs/Int16.]
        The skill is started when a message is received on that topic.

  - \b "RED_LIGHT_GREEN_LIGHT_SKILL_STOP"
        [std_msgs/Int16.]
        The skill is stopped when a message is received on that topic.

  - \b {ppl_topic}
        [people_msgs/People]
        The different input methods for people pose lists

\section Publications
  - \b "ETTS_SAY_PLAIN_TEXT"
        [std_msgs/String.]
        Send this message to say a simple text with etts

  - \b "keyframe_gesture_filename"
        [std_msgs/String.]
        Send a message to instruct the multimodal fission node to perform the desired gesture.

 */
#ifndef RED_LIGHT_GREEN_LIGHT_SKILL_H
#define RED_LIGHT_GREEN_LIGHT_SKILL_H
//RLGL
#include "games_vision/red_light_green_light.h"
//AD
#include "vision_utils/images2ppl.h"
#include "vision_utils/nano_skill.h"
#include "vision_utils/nano_etts_api.h"
//ROS
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#define GESTURE_LOOK_WALL "flori_look_right"
#define GESTURE_LOOK_USER "flori_look_left"

class RedLightGreenLightSkill : public NanoSkill, public RedLightGreenLight {
public:

  RedLightGreenLightSkill() : NanoSkill("RED_LIGHT_GREEN_LIGHT_SKILL_START", "RED_LIGHT_GREEN_LIGHT_SKILL_STOP"){
    _nh_private.param<std::string>("ppl_topic", _ppl_topic, "ppl");
    // check display param
    _nh_private.param("display", _display, _display);
    _nb_frames_analyzed = 0;
    ROS_INFO("Starting the  Red Light Green Light Skill with the topic for ppl '%s'",
             get_ppl_topic().c_str());
  }

  //////////////////////////////////////////////////////////////////////////////

  inline std::string get_ppl_topic() const {
    return _ppl_topic;
  }
  inline bool check_ppl_connectivity() const {
    return (_ppl_sub.getNumPublishers()>0);
  }

protected:
  //////////////////////////////////////////////////////////////////////////////

  virtual void create_subscribers_and_publishers() {
    ROS_WARN("create_subscribers_and_publishers()");
    _ppl_sub = _nh_public.subscribe(_ppl_topic, 1, &RedLightGreenLightSkill::ppl_cb, this);
    _keyframe_gesture_pub = _nh_public.advertise<std_msgs::String>("keyframe_gesture_filename", 100);
    _etts_api.advertise();
    sleep(1);
    if(!check_ppl_connectivity()){
      ROS_WARN("RedLightGreenLightSkill: there is a problem with the ppl publisher '%s'",
               get_ppl_topic().c_str());
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void shutdown_subscribers_and_publishers() {
    ROS_WARN("shutdown_subscribers_and_publishers()");
    stop_game();
    _ppl_sub.shutdown();
    _keyframe_gesture_pub.shutdown();
    _etts_api.shutdown();
  }

  static float mean_depth(const cv::Mat1f & depth,
                          const cv::Mat1b mask = cv::Mat()) {
    unsigned int rows = depth.rows, cols = depth.cols, nvals = 0;
    float sumvals = 0;
    bool use_mask = (!mask.empty());
    const uchar* mask_data;
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      const float* depth_data = depth.ptr<float>(row);
      if (use_mask)
        mask_data = mask.ptr<uchar>(row);
      for (int col = 0; col < cols; ++col) {
        if (vision_utils::is_nan_depth(depth_data[col]))
          continue;
        if (use_mask && !mask_data[col])
          continue;
        sumvals += depth_data[col];
        ++nvals;
      } // end loop col
    } // end loop row
    return sumvals  / nvals;
  } // end mean_depth()

  //////////////////////////////////////////////////////////////////////////////

  inline void ppl_cb(const people_msgs::PeopleConstPtr & msg) {
    ROS_INFO("ppl_cb() : Status:%s",
             RedLightGreenLight::game_status2string(_game_status).c_str());
    // start game if needed
    if (_game_status == GAME_STOPPED) { // gather all names
      if (msg->people.empty())
        return;
      std::vector<RedLightGreenLight::PlayerName> player_names;
      unsigned int npeople = msg->people.size();
      for (unsigned int i = 0; i < npeople; ++i)
        player_names.push_back(msg->people[i].name);
      start_game(player_names);
    }

    //If the game is finished, we don't have to do any computation
    if(_game_status == GAME_WON_BY_PLAYER) {
      return;
    }

    //From the PPL msg, take the PP for each user and obtain his name and his user mask
    //Convert this user mask into cv::Mat to compute the contour
    //Transform the contour into a vector or points and calculate the distance
    std::vector<RedLightGreenLight::PlayerMask> player_masks;
    std::vector<cv::Mat1f> player_depths_images;
    std::vector<unsigned int> pp_indices;
    std::vector<cv::Point> masks_offsets;
    if (!vision_utils::vision_utils::convert(*msg, NULL, &player_depths_images, &player_masks,
                                        &masks_offsets, &pp_indices, true)) {
      ROS_WARN("vision_utils::convert() failed!");
      return;
    }
    unsigned int nplayers = player_depths_images.size();

    // get names
    std::vector<RedLightGreenLight::PlayerName> player_names;
    vision_utils::vision_utils::indices2names(*msg, pp_indices, player_names);
    // convert player_depths_images -> player_depths
    std::vector<RedLightGreenLight::PlayerDistance> player_distances(nplayers);
    for (unsigned int i = 0; i < nplayers; ++i) {
      ROS_WARN("%i: depth:%s, mask:%s",
               i, vision_utils::infosImage(player_depths_images[i]).c_str(),
               vision_utils::infosImage(player_masks[i]).c_str());
      cv::imshow("depth", vision_utils::depth2viz(player_depths_images[i]));
      cv::imshow("mask", player_masks[i]);
      cv::waitKey(20);
      //player_distances[i] = (cv::mean(player_depths_images[i], player_masks[i]))[0];
      player_distances[i] = mean_depth(player_depths_images[i], player_masks[i]);
    } // end loop i

    if (!update(player_names, player_distances, player_masks, masks_offsets)) {
      ROS_WARN("RedLightGreenLightSkill: error in update()!");
      return;
    }

    // analyse results?

    if (_display)
      display(player_names, player_masks);
  } // end ppl_cb();

  //////////////////////////////////////////////////////////////////////////////

  void start_looking_wall(){
    //Start looking wall and get current simulation time to
    //use it to change turns (looking wall / looking user)
    _wall_time_start = ros::Time::now();
    _nb_frames_analyzed = 0;

    std_msgs::String msg;
    msg.data = GESTURE_LOOK_WALL;
    _keyframe_gesture_pub.publish(msg);
    RedLightGreenLight::start_looking_wall();
  }

  //////////////////////////////////////////////////////////////////////////////

  void start_looking_players(){
    RedLightGreenLight::start_looking_players();
    //_wall_time_start = ros::Time::now();
    std_msgs::String msg;
    msg.data = GESTURE_LOOK_USER;
    _keyframe_gesture_pub.publish(msg);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline virtual void say_text(const std::string & sentence) {
    _etts_api.say_text(sentence);
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  ros::Publisher _keyframe_gesture_pub;
  NanoEttsApi _etts_api;
  ros::Subscriber _ppl_sub;
  std::string _ppl_topic;
  ros::Time _wall_time_start;
  int _nb_frames_analyzed;
  bool _display; //!< true to call display
};

#endif // RED_LIGHT_GREEN_LIGHT_SKILL_H
