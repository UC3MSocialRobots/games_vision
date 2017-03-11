#ifndef FIND_PLAYZONE_SKILL_H
#define FIND_PLAYZONE_SKILL_H

#include <opencv2/highgui/highgui.hpp>
// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
// AD
#include "vision_utils/nano_skill.h"
#include "vision_utils/nano_etts_api.h"
// games_vision
#include "games_vision/playzone_find.h"
#include "games_vision/GetPlayzone.h"

/*! \class  PlayzoneFindSkill
 *
 */
class PlayzoneFindSkill : public vision_utils::NanoSkill {
public:
  /*! constructor */
  PlayzoneFindSkill(const int BOARD_OUT_WIDTH_ = 400,
                    const int BOARD_OUT_HEIGHT_ = 400);

  virtual ~PlayzoneFindSkill();

  virtual void create_subscribers_and_publishers();
  virtual void shutdown_subscribers_and_publishers();

  inline std::string get_image_topic() const {
    return _image_resolved_topic;
  }
  inline void set_playzone_topic(const std::string & topic) {
    _playzone_topic = topic;
  }
  inline std::string get_playzone_topic() const {
    return _playzone_pub.getTopic();
  }
  inline void set_playzone_service(const std::string & service) {
    _playzone_service = service;
  }
  inline std::string get_playzone_service() const {
    return _playzone_server.getService();
  }
  bool find_playzone(bool speak);

  void display() {
    _playzone_finder.display();
    if (_rgb.cols > 0)
      cv::imshow("rgb", _rgb);
    cv::waitKey(30);
  }

protected:
  void playzone_cb(const std_msgs::Int16ConstPtr &msg);
  bool playzone_service(games_vision::GetPlayzone::Request &request,
                        games_vision::GetPlayzone::Response &response);

  /** the results */
  void sayResult(int method, double t);

  vision_utils::NanoEttsApi _etts_api;

  ros::Subscriber _playzone_analyze_sub;
  std::string _image_resolved_topic, _playzone_service, _playzone_topic;
  PlayzoneFind _playzone_finder;
  cv::Mat3b _rgb;
  bool _playzone_success;

  image_transport::ImageTransport _it;
  image_transport::Publisher _playzone_pub;
  ros::ServiceServer _playzone_server;
  cv_bridge::CvImage _playzone_msg;

  //! the publisher for gestures
  ros::Publisher _gesture_pub;
}; // end class PlayzoneFindSkill

#endif // FIND_PLAYZONE_SKILL_H
