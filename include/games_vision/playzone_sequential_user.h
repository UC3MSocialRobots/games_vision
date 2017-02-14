#ifndef PLAYZONESEQUENCIALUSER_H
#define PLAYZONESEQUENCIALUSER_H

/*!
  2 modes:
   with "tacto" message:
    do_stuff_before_get_playzone()
    for (unsigned int i = 0; i < total; ++i) {
      send an event asking to retrieve playzone
      retrieve playzone
      process_pz(cv::Mat3b &)
    }
    do_stuff_after_get_playzone(bool was_find_and_process_success)

  with "drawer_out" message:
    do_stuff_before_get_playzone()
    send an event asking to retrieve playzone
    retrieve playzone
    process_pz(cv::Mat3b &)
    do_stuff_after_get_playzone(bool was_find_and_process_success)
  */

// AD
#include "games_vision/nano_skill.h"
#include "games_vision/nano_etts_api.h"
//#include "vision_utils/timer.h"
#include <games_vision/GetPlayzone.h>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// ros
#include <std_msgs/String.h>

//#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
#define DEBUG_PRINT(...)   ROS_INFO(__VA_ARGS__)

class PlayzoneSequentialUser: public NanoSkill {
public:
  //! the possible different status
  enum Status {
    NEVER_RUN = 0,
    SUCCESS_FOUND_AND_PROCESSED = 2,
    FAILURE_NO_PZ = 3,
    FAILURE_PROCESSING_FAILED = 4
  };
  static const int MAX_CONSECUTIVE_FAILURES = 3;

  //! ctor
  PlayzoneSequentialUser(const std::string & signal_start,
                         const std::string & signal_stop) :
    NanoSkill(signal_start, signal_stop),
    _it(_nh_public)
  {
    // DEBUG_PRINT("ctor");

    _win1_name = "PSU-playzone";
    _win2_name = "PSU-playzone_illus";
    /* creating the frames */
    _playzone.create(50, 50);
    _playzone.setTo(0);
    _playzone.copyTo(_playzone_illus);
    _playzone.copyTo(_playzone_modified_for_drawer);
    _playzone_service = "playzone";
    _nh_private.param("playzone_service", _playzone_service, _playzone_service);
    _drawer_topic = "drawer_in";
    _nh_private.param("drawer_topic", _drawer_topic, _drawer_topic);
    _status = NEVER_RUN;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! dtor
  ~PlayzoneSequentialUser() {
    // DEBUG_PRINT("dtor");
  }

  //////////////////////////////////////////////////////////////////////////////

  /** called when the child skill is launched - from NanoSkill  */
  virtual void create_subscribers_and_publishers_playzone() = 0;

  /** called when the child skill is stopped - from NanoSkill */
  virtual void shutdown_subscribers_and_publishers_playzone() = 0;

  /*! the function that will perform on the playzone
      return true if it was a success, false otherwise */
  virtual bool process_pz(const cv::Mat3b & pz) = 0;

  /*! the function that will be called before trying to get the playzone */
  virtual void do_stuff_before_get_playzone() {
    DEBUG_PRINT("PlayzoneSequentialUser:do_stuff_before_get_playzone()");
  }

  /*! the function that will be called after trying to get the playzone */
  virtual void do_stuff_after_get_playzone(bool was_find_and_process_success) {
    DEBUG_PRINT("PlayzoneSequentialUser:do_stuff_after_get_playzone(success:%i)",
               was_find_and_process_success);
    if (was_find_and_process_success) { // gesture yes
      std_msgs::String gesture_name; gesture_name.data = "flori_yes_mute";
      _gesture_pub.publish(gesture_name);
    } else {
      _etts_api.say_text("|en:I couldn't find the marker, sorry."
                         "|es:No he conseguido reconocer el marcador, lo siento.");
      // play gesture "no_mute"
      std_msgs::String gesture_name; gesture_name.data = "flori_no_mute";
      _gesture_pub.publish(gesture_name);
    }
  }

  void get_and_process_pz() {
    DEBUG_PRINT("get_and_process_pz()");
    // do nothing if not activated
    if (!is_running())
      return;
    do_stuff_before_get_playzone();

    // reset counter
    int consecutive_failures = 0;
    while(consecutive_failures < MAX_CONSECUTIVE_FAILURES) {
      _status = FAILURE_NO_PZ;
      games_vision::GetPlayzone srv;
      bool ok = _pz_client.call(srv.request, srv.response);
      // if not obtained, die!
      if (!ok) {
        ROS_FATAL("Could not get PZ on '%s', dying, argh!", _playzone_service.c_str());
        consecutive_failures++;
        continue;
      }
      bool pz_valid = (srv.response.success && srv.response.playzone.width > 0);
      if (!pz_valid) {
        ROS_WARN(" INVALID pz received while waiting for it!");
        consecutive_failures++;
        continue;
      }

      DEBUG_PRINT(" valid pz received while waiting for it!");
      cv_bridge::CvImageConstPtr _pz_bridge = cv_bridge::toCvCopy(srv.response.playzone);
      _pz_bridge->image.copyTo(_playzone);

      // then, copy it to playzone_modified_for_drawer
      _playzone.copyTo(_playzone_illus);
      _playzone.copyTo(_playzone_modified_for_drawer);

      // call process_pz(const cv::Mat3b & pz)
      bool process_success = process_pz(_playzone);
      if (process_success)
        break;
      ROS_WARN("Impossible to perform process_pz() successfully, restarting.");
      _status = FAILURE_PROCESSING_FAILED;
      ++consecutive_failures;
    } // end while(_consecutive_failures < MAX_CONSECUTIVE_FAILURES)

    // too many failures - either because of no PZ or because of processing failure
    if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
      ROS_WARN("Impossible to find the playzone %i times."
               "No more tries.", MAX_CONSECUTIVE_FAILURES);
      do_stuff_after_get_playzone(false);
      return;
    }

    _status = SUCCESS_FOUND_AND_PROCESSED;
    do_stuff_after_get_playzone(true);
    // share the image for an eventual drawer
    send_playzone_out_to_drawer();
  }

  //! \return true if the playzone was obtained
  inline bool is_playzone_detection_success() const {
    return _status == SUCCESS_FOUND_AND_PROCESSED
        || _status == FAILURE_PROCESSING_FAILED;
  }
  //! \return true if process_pz() was a success
  inline bool is_playzone_processing_success() const {
    return _status == SUCCESS_FOUND_AND_PROCESSED;
  }

  inline bool has_playzone_service() {
    return _pz_client.exists();
  }

  //! \return the status of the playzone detection
  inline Status get_status() const { return _status; }

  virtual void display() {
    cv::imshow(_win1_name, _playzone);
    cv::imshow(_win2_name, _playzone_illus);
    cv::imshow("playzone_modified_for_drawer", _playzone_modified_for_drawer);
    cv::waitKey(100);
  }

  inline void set_playzone_service(const std::string & service) { _playzone_service = service; }
  inline std::string get_playzone_service() const { return _playzone_service; }

  inline void set_drawer_topic(const std::string & topic) { _drawer_topic = topic; }
  inline std::string get_drawer_topic() const { return _drawer_topic; }

protected:

  //! shut up as soon as possible
  inline void say_text(const std::string & sentence) {
    _etts_api.say_text(sentence);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! shut up as soon as possible
  inline void shut_up() {
    // etts.shutUpImmediatly();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_eyes(const std::string emotion) {
    std_msgs::String eyes_msg;
    eyes_msg.data = emotion;
    _eyes_pub.publish(eyes_msg);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! the image for the drawer - modify it in ctor if you want a drawer to use it
  cv::Mat3b _playzone_modified_for_drawer;

  //! the output - each time a new playzone is obtained, it is copied into it.
  cv::Mat3b _playzone_illus;

  //! the publisher for gestures
  ros::Publisher _gesture_pub;
  //! the publisher for expressions for eyes
  ros::Publisher _eyes_pub;

  //! the name of the first window
  std::string _win1_name;
  //! the name of the second window
  std::string _win2_name;

private:
  //////////////////////////////////////////////////////////////////////////////

  /** called when this skill is launched */
  void create_subscribers_and_publishers() {
    DEBUG_PRINT("create_subscribers_and_publishers()");

    // advertise events
    _pz_client =  _nh_public.serviceClient<games_vision::GetPlayzone>("playzone");
    _camera_blocker_start_pub = _nh_public.advertise<std_msgs::Int16>("CAMERA_BLOCKER_SKILL_START", 1);
    _camera_blocker_stop_pub = _nh_public.advertise<std_msgs::Int16>("CAMERA_BLOCKER_SKILL_STOP", 1);
    // node_double_screen_4D.cpp
    _eyes_pub = _nh_public.advertise<std_msgs::String>("SCREENS_PLAIN_TEXT", 1);

    _gesture_pub = _nh_public.advertise<std_msgs::String>
        //(gesture_player::gesture_filename_topic, 1);
        ("keyframe_gesture_filename", 1);
    _drawer_start_pub = _nh_public.advertise<std_msgs::Int16>("DRAWER_START", 1);
    _drawer_stop_pub = _nh_public.advertise<std_msgs::Int16>("DRAWER_STOP", 1);
    _drawer_pub = _it.advertise(_drawer_topic, 1);
    _etts_api.advertise();

    // subscribers
    touch_sub = _nh_public.subscribe<std_msgs::String>
        ("capacitive_touch", 1, &PlayzoneSequentialUser::touch_cb, this);
    _drawer_sub = _it.subscribe("drawer_out", 1, &PlayzoneSequentialUser::drawer_cb, this);
    // call the child create_subscribers_and_publishers_playzone()
    create_subscribers_and_publishers_playzone();

    /*
    * configure an eventual drawer
    */
    std_msgs::Int16 empty_msg;
    empty_msg.data = 0;
    _drawer_start_pub.publish(empty_msg);
    send_playzone_out_to_drawer();

    /* init the other things */
    // start the drawer
    _drawer_start_pub.publish(empty_msg);
    // start the fake touch listener
    _camera_blocker_start_pub.publish(empty_msg);

    // gesture flori_look_table_mute
    std_msgs::String gesture_name; gesture_name.data = "flori_look_table_mute";
    _gesture_pub.publish(gesture_name);

    // set eyes as NORMAL
    set_eyes("NORMAL");

    sleep(1); // time for playzone skill to be started
  } // end create_subscribers_and_publishers()

  //////////////////////////////////////////////////////////////////////////////

  /** called when this skill is stopped */
  void shutdown_subscribers_and_publishers() {
    DEBUG_PRINT("shutdown_subscribers_and_publishers()");

    // call the child shutdown_subscribers_and_publishers()
    shutdown_subscribers_and_publishers_playzone();

    std_msgs::Int16 empty_msg;
    empty_msg.data = 0;
    // stop the drawer
    _drawer_stop_pub.publish(empty_msg);
    // stop the fake touch listener
    _camera_blocker_stop_pub.publish(empty_msg);

    _camera_blocker_start_pub.shutdown();
    _camera_blocker_stop_pub.shutdown();
    _eyes_pub.shutdown();
    _gesture_pub.shutdown();
    _drawer_start_pub.shutdown();
    _drawer_stop_pub.shutdown();
    _drawer_pub.shutdown();

    _etts_api.shutdown();
  } // end shutdown_subscribers_and_publishers_playzone();

  //////////////////////////////////////////////////////////////////////////////

  //! the routine for touch sensors, from TouchListener
  void touch_cb(const std_msgs::StringConstPtr & /*msg*/) {
    get_and_process_pz();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! the routine for touch sensors, from TouchListener
  void drawer_cb(const sensor_msgs::ImageConstPtr & msg) {
    DEBUG_PRINT("drawer_cb()");
    // do nothing if not activated
    if (!is_running())
      return;
    do_stuff_before_get_playzone();

    cv_bridge::CvImageConstPtr pz_bridge = cv_bridge::toCvShare(msg);
    pz_bridge->image.copyTo(_playzone);

    // then, copy it to playzone_modified_for_drawer
    _playzone.copyTo(_playzone_illus);
    _playzone.copyTo(_playzone_modified_for_drawer);

    // call process_pz(const cv::Mat3b & pz)
    bool process_success = process_pz(_playzone);
    if (!process_success) {
      do_stuff_after_get_playzone(false);
      _status = FAILURE_PROCESSING_FAILED;
      return;
    }

    _status = SUCCESS_FOUND_AND_PROCESSED;
    do_stuff_after_get_playzone(true);
    // share the image for an eventual drawer
    send_playzone_out_to_drawer();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! share the image for an eventual drawer
  void send_playzone_out_to_drawer() {
    DEBUG_PRINT("send_playzone_out_to_drawer()");
    _playzone_modified_for_drawer.copyTo(_drawer_bg_msg.image);
    _drawer_bg_msg.encoding = sensor_msgs::image_encodings::BGR8;
    _drawer_bg_msg.header.stamp = ros::Time::now();
    _drawer_pub.publish(_drawer_bg_msg.toImageMsg());
  }

  //////////////////////////////////////////////////////////////////////////////

  Status _status;
  //! the acquired playzone - you should instead use the "ps" argument
  cv::Mat3b _playzone;
  NanoEttsApi _etts_api;
  image_transport::ImageTransport _it;
  //! controls for the playzone skill
  std::string _playzone_service, _drawer_topic;
  ros::ServiceClient _pz_client;
  //! controls for the drawer
  ros::Publisher _drawer_start_pub, _drawer_stop_pub;
  image_transport::Publisher _drawer_pub;
  image_transport::Subscriber _drawer_sub;
  cv_bridge::CvImage _drawer_bg_msg;
  //! controls for the camera blocker
  ros::Publisher _camera_blocker_start_pub, _camera_blocker_stop_pub;
  //! touch subscriber
  ros::Subscriber touch_sub;
};

#endif // PLAYZONESEQUENCIALUSER_H
