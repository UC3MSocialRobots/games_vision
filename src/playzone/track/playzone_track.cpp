#include "games_vision/drawer.h"
#include "games_vision/playzone_track.h"
#include "vision_utils/barycenter.h"
#include "vision_utils/drawPolygon.h"
#include "vision_utils/region_growth.h"
#include "vision_utils/boundingBox_vec.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

PlayzoneTrack::PlayzoneTrack(int BOARD_OUT_WIDTH_,
                             int BOARD_OUT_HEIGHT_) :
  PlayzoneFind(BOARD_OUT_WIDTH_, BOARD_OUT_HEIGHT_) {
  ROS_INFO("PlayzoneTrack ctor");
  /* creating the frames */
  region_growth_seen_points = NULL;

  /* init the other things */
  if (reproject_mode == MODE_REPROJECT_IMAGE
      || reproject_mode == MODE_REPROJECT_VIDEO)
    reproject_init();
}

////////////////////////////////////////////////////////////////////////////////

PlayzoneTrack::~PlayzoneTrack() {
  delete[] region_growth_seen_points;

  if (reproject_mode == MODE_REPROJECT_VIDEO)
    model_capture.release();
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneTrack::set_mode(const Mode mode,
                             const std::string & file_with_mode) {
  ROS_INFO("set_mode(%i)", mode);
  reproject_mode = mode;
  filename_with_reproject_mode = file_with_mode;
}

////////////////////////////////////////////////////////////////////////////////

bool PlayzoneTrack::find_playzone() {
  ROS_INFO("find_playzone()");

  //PlayzoneFindSkill::proceso();

  /*
     * find playzone
     */
  is_tracking_OK = false;

  /* prepare image */
  find_playzone_part1();
  global_timer.printTime("after find_playzone_part1()");

  find_with_tracking();
  global_timer.printTime("after find_with_tracking()");
  if (!is_tracking_OK) {
    find_from_playzone();
  }

  // illustrate
  if (reproject_mode == MODE_ONLY_TRACK) {
    //frame_with_model.create(frame_buffer.size());
    //frame_with_model.setTo(0);
    frame_thres.copyTo(frame_with_model);
    if (is_tracking_OK)
      vision_utils::drawPolygon(frame_with_model, board_corners, true,
                               CV_RGB(255, 0, 0), 2);
  } // end if MODE_ONLY_TRACK


  /* display */
#ifdef PZ_FIND_IMAGES
  cv::cvtColor(frame_thres, frameOut, CV_GRAY2RGB);
  if (is_tracking_OK) {
    vision_utils::drawListOfPoints(frameOut, *best_component_points,
                                  cv::Vec3b(255, 150, 150));
    int i = 0;
    for (Component::iterator it = board_corners.begin(); it
         < board_corners.end(); ++it) {
      vision_utils::drawCross(frameOut, *it, 10,
                             vision_utils::color_scalar<cv::Scalar>(i++, 4),
                             2);
    }
  } // end if is_tracking_OK
  global_timer.printTime("after find_playzone()");
#endif // PZ_FIND_IMAGES

  if (reproject_mode == MODE_REPROJECT_IMAGE || reproject_mode == MODE_REPROJECT_VIDEO) {
    reproject();
    global_timer.printTime("after reproject()");
  }

  /* record the videos */
#ifdef PZ_TRACK_VIDEOS
  if (writer_frame.isOpened() == false) {
    int codec = //-1,
        //CV_FOURCC('M','J','P','G')    //= motion-jpeg codec (does not work well)
        CV_FOURCC('D', 'I', 'V', 'X') //= MPEG-4 codec
        ;
    writer_frame.open(
          "01-writer_frame.avi",codec, 10, frame_buffer.size());
    writer_frameOut.open(
          "02-writer_frameOut.avi", codec, 10, frame_buffer.size());
    writer_frame_with_model.open(
          "04-writer_frame_with_model.avi", codec, 10, frame_buffer.size());
  }

  writer_frame.write(frame_buffer );
  writer_frameOut.write(frameOut);
  writer_frame_with_model.write(frame_with_model );
#endif


  global_timer.printTime("at the end of find_playzone()");
  return is_tracking_OK;
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneTrack::find_from_playzone() {
  ROS_INFO("find_from_playzone()");

  find_playzone_part2(false);
  is_tracking_OK = (get_current_status() == SUCCESS_FOUND);
}

////////////////////////////////////////////////////////////////////////////////

static inline double dist_angles(double a1, double a2) {
  double DPI = 2 * CV_PI;
  double diff = a2 - a1;
  while (diff < -CV_PI)
    diff += DPI;
  while (diff > CV_PI)
    diff -= DPI;
  return fabs(diff);
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneTrack::find_with_tracking() {
  ROS_INFO("find_with_tracking()");

  if (board_corners.size() == 0) {
    ROS_INFO("board_corners.size() == 0, return");
    return;
  }
  if (best_component_index == -1) {
    ROS_INFO("best_component_index == -1, return");
    return;
  }
  if (best_component_points ==NULL || best_component_points->size() == 0) {
    ROS_INFO("component.size() == 0, return");
    return;
  }

  // find a point that is still white in the position of the former component
  bool OK = false;
  cv::Point* seed;
  for (unsigned int try_nb = 0; try_nb < best_component_points->size(); ++try_nb) {
    //        ROS_INFO("try_nb:%i / %i", try_nb,
    //                     best_component_points->size());
    seed = &best_component_points->at(rand() % best_component_points->size());
    //ROS_DEBUG("seed_pt:%s", vision_utils::printP2(*seed).c_str());
    if (frame_thres(*seed) == 255) {
      OK = true;
      break;
    }
  } // end loop try_nb
  if (!OK) {
    ROS_INFO("Impossible to find a common point between "
                 "the last pos and now, returning");
    return;
  }
  //ROS_INFO("seed_pt:%s", vision_utils::printP2(*seed).c_str());

  /*
     * method 1 - using cv::floodFill()
     */

  //    /* flood fill the new component from the found seed */
  //    cv::floodFill(frame_thres, *seed, cvScalarAll(127));

  //    /* get  the points from the new component */
  //    int old_size = best_component_points->size();
  //    best_component_points->clear();
  //    xMin = 100000;
  //    yMin = xMin;
  //    xMax = -1;
  //    yMax = -1;
  //    cv::Mat1b::const_iterator frame_thres_it = frame_thres.begin();
  //    for (int y = 0; y < frame_thres.rows; ++y) {
  //        for (int x = 0; x < frame_thres.cols; ++x) {
  //            if (*frame_thres_it == 127) {
  //                xMin = std::min(x, xMin);
  //                yMin = std::min(y, yMin);
  //                xMax = std::max(x, xMax);
  //                yMax = std::max(y, yMax);
  //                best_component_points->push_back(cv::Point(x, y));
  //            }
  //            ++frame_thres_it;
  //        } // end loop x
  //    } // end loop y
  //    ROS_INFO("best_component_points->size(): %i",
  //                 best_component_points->size());

  //    // update the bbox
  //    component_bbox.x = xMin;
  //    component_bbox.y = yMin;
  //    component_bbox.width = xMax - xMin + 1;
  //    component_bbox.height = yMax - yMin + 1;
  //    component_center.x = (int) (.5f * (xMax + xMin));
  //    component_center.y = (int) (.5f * (yMax + yMin));

  //cv::imshow(window2Name, frame_thres);
  //cv::waitKey(5000);

  /*
     * method 2 - using vision_utils::region_growth()
     */

  int old_size = best_component_points->size();
  if (region_growth_seen_points == NULL)
    region_growth_seen_points = new bool[frame_thres.cols * frame_thres.rows];
  vision_utils::region_growth(frame_thres,
                             region_growth_seen_points,
                             *seed,
                             0, 0,
                             *best_component_points);
  ROS_INFO("best_component_points->size(): %i",
               (int) best_component_points->size());
  component_bbox = vision_utils::boundingBox_vec
      <Component, cv::Rect>(*best_component_points);
  component_center.x = component_bbox.x + component_bbox.width / 2;
  component_center.y = component_bbox.y + component_bbox.height / 2;


  global_timer.printTime("after finding best_component_points");

  if (best_component_points->size() == 0) {
    ROS_INFO("best_component_points->size() == 0, return");
    return;
  }

  /* look for big variations */
  double size_ratio = 1.f * best_component_points->size() / old_size;
  ROS_INFO("size_ratio:%f", size_ratio);

  if (size_ratio > SIZE_VARIATION_MAX_RATIO || size_ratio
      < SIZE_VARIATION_MIN_RATIO) {
    ROS_INFO("size_ratio out of bounds, return");
    return;
  }

  /* find the corners */
  PlayzoneFind::corner_finder.setComponent(best_component_points,
                                           component_bbox,
                                           component_center);
  PlayzoneFind::corner_finder.set_M3_frame_size(frame_buffer.size());
  PlayzoneFind::corner_finder.allow_all_methods();
  PlayzoneFind::corner_finder.M20_allowed = false;
  PlayzoneFind::corner_finder.M30_allowed = false;
  PlayzoneFind::corner_finder.findCorners();

  if (!PlayzoneFind::corner_finder.were_corners_found) {
    ROS_INFO("findCorners with all methods was a failure, return");
    return;
  }

  /*
     * copy the results
     */
  Component new_corners;
  PlayzoneFind::corner_finder.getCorners(new_corners);
  cv::Point center_old = vision_utils::barycenter(board_corners);
  cv::Point center_new = vision_utils::barycenter(new_corners);
  //      cout << "center_old:" << center_old.x << ", " << center_old.y << endl;
  //      cout << "center_new:" << center_new.x << ", " << center_new.y << endl;

  double angle_old[4], angle_new[4];
  for (int i = 0; i < 4; ++i) {
    angle_old[i] = atan2(board_corners[i].y - center_old.y,
                         board_corners[i].x - center_old.x);
    angle_new[i] = atan2(new_corners[i].y - center_new.y,
                         new_corners[i].x - center_new.x);
    //              cout << "board_corners[i]:" << board_corners[i].x << "-" << board_corners[i].y << endl;
    //              cout << "found_corners[i]:" << (*newP)[i].x << "-" << (*newP)[i].y << endl;
    //cout << angle_old[i] << " - " << angle_new[i] << endl;
  }

  /* find the best affectation, trying all the possibilities */
  int best_affect[4];
  for (int var = 0; var < 4; ++var)
    best_affect[var] = -1;
  double cost, lowest_cost = INFINITY;

  for (int p0 = 0; p0 < 4; ++p0) {
    for (int p1 = 0; p1 < 4; ++p1) {
      if (p1 == p0)
        continue;
      for (int p2 = 0; p2 < 4; ++p2) {
        if (p2 == p0 || p2 == p1)
          continue;
        for (int p3 = 0; p3 < 4; ++p3) {
          if (p3 == p2 || p3 == p1 || p3 == p0)
            continue;

          cost = dist_angles(angle_old[0], angle_new[p0]);
          cost += dist_angles(angle_old[1], angle_new[p1]);
          cost += dist_angles(angle_old[2], angle_new[p2]);
          cost += dist_angles(angle_old[3], angle_new[p3]);

          if (cost < lowest_cost) { /* new best */
            lowest_cost = cost;
            best_affect[0] = p0;
            best_affect[1] = p1;
            best_affect[2] = p2;
            best_affect[3] = p3;
          }

        } // end loop p3
      } // end loop p2
    } // end loop p1
  } // end loop p0

  //cout << lowest_cost << endl;
  if (lowest_cost > ANGLE_VARIATION_MAX_DIST) {
    ROS_INFO(
          "RETURN because big variation in the angles, ANGLE_VARIATION_MAX_DIST < lowest_cost = %f",
          lowest_cost);
    return;
  }

  board_corners.clear();
  //PlayzoneFind::corner_finder.getCorners( &board_corners );
  for (int i = 0; i < 4; ++i) {
    //cout << i << "->" << best_affect[i] << "\t";
    board_corners.push_back(new_corners[best_affect[i]]);
  }
  is_tracking_OK = true;
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneTrack::reproject_init() {
  ROS_INFO("reproject_init()");

  model_warp_matrix.create(3, 3, CV_32FC1);

  /* init the images */
  if (reproject_mode == MODE_REPROJECT_IMAGE) {
    model_image = cv::imread(filename_with_reproject_mode);
    ROS_INFO("model_image:%s", vision_utils::infosImage(model_image).c_str());
  }
  else if (reproject_mode == MODE_REPROJECT_VIDEO) {
    model_capture.open(filename_with_reproject_mode);
    model_capture >> model_image;
  }

  if (reproject_mode == MODE_REPROJECT_IMAGE
      || reproject_mode == MODE_REPROJECT_VIDEO) {
    source_pts.push_back(cv::Point2f(model_image.cols, model_image.rows));
    source_pts.push_back(cv::Point2f(0, model_image.rows));
    source_pts.push_back(cv::Point2f(0, 0));
    source_pts.push_back(cv::Point2f(model_image.cols, 0));
  }
  //    for (int i = 0; i < 4; ++i)
  //        ROS_INFO("[%i] src:%f, %f -> dest:%f, %f", i,
  //                     source[i].x, source[i].y, dest[i].x, dest[i].y);

}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   reproject the chosen image in the camera frame
 */
void PlayzoneTrack::reproject() {
  ROS_INFO("reproject()");

  // copy the image
  frame_buffer.copyTo(frame_with_model);

  ///// image mode
  if (reproject_mode == MODE_REPROJECT_IMAGE) {
  }
  ///// video mode
  else if (reproject_mode == MODE_REPROJECT_VIDEO) {
    /* read the next image in the video */
    model_capture >> model_image;
    if (model_capture.grab() == false) { // end of video => rewind
      model_capture.set(CV_CAP_PROP_POS_MSEC, 0);
      model_capture >> model_image;
    }
    ROS_INFO("model_image:%s", vision_utils::infosImage(model_image).c_str());
  } // end if MODE == MODE_REPROJECT_VIDEO

  if (!is_tracking_OK) {
    ROS_INFO("is_tracking_OK = FALSE, returning.");
    return;
  }

  /* calc matrix */
  dest_pts.clear();
  dest_pts.reserve(board_corners.size());
  for(std::vector<cv::Point>::const_iterator pt = board_corners.begin();
      pt != board_corners.end() ; ++pt) {
    dest_pts.push_back(cv::Point2f(pt->x, pt->y));
  } // end loop pt
  model_warp_matrix = cv::getPerspectiveTransform(source_pts.data(), dest_pts.data());

  // rectify
  cv::warpPerspective(model_image,
                      frame_with_model,
                      model_warp_matrix,
                      frame_with_model.size(),
                      cv::INTER_LINEAR,
                      cv::BORDER_TRANSPARENT);
}

////////////////////////////////////////////////////////////////////////////////

