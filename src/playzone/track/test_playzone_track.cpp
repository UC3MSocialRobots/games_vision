#include "games_vision/playzone_track.h"
#include <vision_utils/img_path.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

/** test */
int main(int argc, char** argv) {
  ROS_INFO("main()");
  ros::init(argc, argv, "test_playzone_track");
  ros::NodeHandle nh_private("~");
  int mode = PlayzoneTrack::MODE_ONLY_TRACK;
  // 0:track, 1:image, 2:video
  nh_private.param("mode", mode, mode);
  // the file that goes with the mode
  //std::string filename = IMG_DIR "snoopy.avi";
  std::string filename = IMG_DIR "powerXML/power2.png";
  nh_private.param("filename", filename, filename);

  // create the PlayzoneTrack
  PlayzoneTrack pzt(300, 300);
  pzt.set_mode((PlayzoneTrack::Mode) mode, filename);

  // init the capture
  cv::VideoCapture cap(0); // open the default camera
  if(!cap.isOpened()) { // check if we succeeded
    ROS_WARN("VideoCapture not opened, exiting");
    return -1;
  }
  //CvCapture* cap = cvCaptureFromCAM(-1);

  cv::namedWindow("frame",1);
  cv::namedWindow("frame_with_model",1);

  cv::Mat3b frame;
  for (;;) {
    // get a new frame from camera
    cap >> frame;
    //        cvGrabFrame(cap);
    //        IplImage* frame_ipl = cvRetrieveFrame(cap);
    //        cv::Mat3b frame = cv::cvarrToMat(frame_ipl);

    pzt.set_input(&frame);
    pzt.find_playzone();
    cv::imshow("frame", frame);
    cv::imshow("frame_with_model", pzt.frame_with_model);
    if(cv::waitKey(20) >= 0)
      break;
  }
  return 0;
}

