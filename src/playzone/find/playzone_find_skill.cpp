#include "games_vision/playzone_find_skill.h"


// gesture_player
//#include <gesture_definitions.h>
#include <std_msgs/String.h>

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   ROS_INFO(__VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

PlayzoneFindSkill::PlayzoneFindSkill(const int BOARD_OUT_WIDTH_ /* = 400 */,
                                     const int BOARD_OUT_HEIGHT_ /* = 400 */) :
  NanoSkill("FIND_PLAYZONE_START", "FIND_PLAYZONE_STOP"),
  _pz_finder(BOARD_OUT_WIDTH_, BOARD_OUT_HEIGHT_),
  _it(_nh_public)
{
  std::string image_topic = "rgb";
  _nh_private.param("image_topic", image_topic, image_topic);
  _image_resolved_topic = _nh_public.resolveName(image_topic);
  printf("PlayzoneFindSkill: getting color images on '%s'\n",
         _image_resolved_topic.c_str());
}

////////////////////////////////////////////////////////////////////////////////

PlayzoneFindSkill::~PlayzoneFindSkill() {
}

////////////////////////////////////////////////////////////////////////////////

/** say a sentence to sum up the results */
void PlayzoneFindSkill::sayResult(int method, double t) {
  std::ostringstream m;
  m << "Acabado in " << cvRound(t) << " millisegundos, ";
  m << "con el metodo numero " << method << ". ";
  _etts_api.say_text(m.str());

  //  m.str("board :");
  //  for (int i = 0; i < 4; ++i)
  //      m << "(" << board_corners[i].x << ", " << board_corners[i].y << "), ";
  //  DEBUG_PRINT(m.str());
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFindSkill::create_subscribers_and_publishers() {
  DEBUG_PRINT("PlayzoneFindSkill::create_subscribers_and_publishers()\n");

  // subscribe to the topic for finding playzone
  _pz_analyze_sub = _nh_public.subscribe("FIND_PLAYZONE_ANALYZE", 1,
                                         &PlayzoneFindSkill::find_playzone, this);

  // advertise gesture publisher
  _gesture_pub = _nh_public.advertise<std_msgs::String>
      //(gesture_player::gesture_filename_topic, 1);
                 ("keyframe_gesture_filename", 1);

  _pz_pub = _it.advertise("PLAYZONE", 1);
  _etts_api.advertise();
  // look at the table
  std_msgs::String gesture_name; gesture_name.data = "flori_look_table_mute";
  _gesture_pub.publish(gesture_name);
  sleep(1);
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFindSkill::shutdown_subscribers_and_publishers() {
  DEBUG_PRINT("PlayzoneFindSkill::shutdown_subscribers_and_publishers()\n");
  _etts_api.shutdown();
  _pz_analyze_sub.shutdown();
  _gesture_pub.shutdown();
  _pz_pub.shutdown();
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFindSkill::find_playzone(const std_msgs::Int16ConstPtr & msg) {
  bool speak = msg->data > 0;
  DEBUG_PRINT("PlayzoneFindSkill:find_playzone(speak:%i)\n", speak);

  // look at the table (in case of)
  std_msgs::String gesture_name; gesture_name.data = "flori_look_table_mute";
  _gesture_pub.publish(gesture_name);
  // but don't wait here

  if (speak)
    _etts_api.say_text("|es:Busco el sitio de juego."
                       "|en:Let us look for the marker.");
  vision_utils::Timer timer;

  // get one frame
  _pz_msg.encoding = sensor_msgs::image_encodings::BGR8;
  _pz_msg.header.stamp = ros::Time::now();
  sensor_msgs::ImageConstPtr rgb_msg = ros::topic::waitForMessage<sensor_msgs::Image>
      (_image_resolved_topic, _nh_public, ros::Duration(1));
  if (!rgb_msg) {
    printf("PlayzoneFindSkill: could not obtain an image on '%s'!\n",
           _image_resolved_topic.c_str());
    _pz_msg.image.release();
    _pz_pub.publish(_pz_msg.toImageMsg());
    return;
  }
  cv_bridge::CvImageConstPtr rgb_bridge = cv_bridge::toCvShare(rgb_msg);
  rgb_bridge->image.copyTo(_rgb);

  // keep the frame
  DEBUG_PRINT("frame:%s", vision_utils::infosImage(_rgb).c_str());
  _pz_finder.set_input(&_rgb);
  _playzone_success = _pz_finder.find_playzone();
  if (_playzone_success)
    _pz_finder.get_playzone()->copyTo(_pz_msg.image);
  else // clear image
    _pz_msg.image.release();

  /* save to MCP */
  _pz_pub.publish(_pz_msg.toImageMsg());

  if (speak)
    sayResult(_pz_finder.get_corner_successful_method_number(), timer.getTimeMilliseconds());
} // end find_playzone();
