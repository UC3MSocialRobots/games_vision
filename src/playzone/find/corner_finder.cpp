
// maggie
#include "games_vision/playzone_find.h"
#include "games_vision/corner_finder.h"
#include "vision_utils/absolute_angle_between_three_points.h"
#include "vision_utils/barycenter_of_two_points.h"
#include "vision_utils/distance_point_polygon.h"
#include "vision_utils/point_inside_polygon.h"
#include "vision_utils/rand_gaussian.h"
#include "vision_utils/rotate_dilate_polygon.h"
#include "vision_utils/drawListOfPoints.h"
#include "vision_utils/line_equation.h"
#include "vision_utils/interLine.h"
// include find playzone for the options
// opencv
#include <opencv2/imgproc/imgproc.hpp>

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   ROS_INFO(__VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

#ifdef PZ_FIND_IMAGES
#include <opencv2/highgui/highgui.hpp>
#endif // PZ_FIND_IMAGES

/* init the video writer */
#ifdef PZ_FIND_VIDEOS
cv::VideoWriter writer_M20  ("M20-anim.avi",codec, 10, cv::Size(640, 480));
cv::VideoWriter writer_M17  ("M17-anim.avi",codec, 10, cv::Size(640, 480));
#endif // PZ_FIND_VIDEOS

/*!
 * \brief   destructor - a bit of cleaning
 */
CornerFinder::CornerFinder() {
  DEBUG_PRINT("CornerFinder ctor\n");
  findCorners_init();
}

/*!
 * \brief   destructor - a bit of cleaning
 */
CornerFinder::~CornerFinder() {
  DEBUG_PRINT("CornerFinder dtor\n");
  delete[] M10_biggestDistance_slice;
  delete[] M10_furthestPoint_slice;
  delete[] M15_distance_diffs;
  delete[] M17_average_angles;
  //delete[] M20_quad;
}



/*!
 * \brief   allow the finder to use all methods
 */
void CornerFinder::allow_all_methods() {
  DEBUG_PRINT("allow_all_methods()\n");
  M10_allowed = true;
  M15_allowed = true;
  M17_allowed = true;
  M20_allowed = true;
  M30_allowed = true;
}

/*!
 * \brief   fordbid the finder to use all methods
 */
void CornerFinder::forbid_all_methods() {
  DEBUG_PRINT("forbid_all_methods()\n");
  M10_allowed = false;
  M15_allowed = false;
  M17_allowed = false;
  M20_allowed = false;
  M30_allowed = false;
}

/*!
 * \brief   init the values for the findCorners methods
 */
void CornerFinder::findCorners_init() {
  DEBUG_PRINT("findCorners_init()\n");
  /* method 1 */
  M10_biggestDistance_slice = new double[M10_NB_ANGULAR_SLICES];
  M10_furthestPoint_slice = new cv::Point[M10_NB_ANGULAR_SLICES];
  /* method 15 */
  M15_distance_diffs = new double[M10_NB_ANGULAR_SLICES];
  /* method 17 */
  M17_average_angles = new double[M10_NB_ANGULAR_SLICES];
  /* method 2 */
  //M20_quad = new cv::Point[4];
  /* method 3 */
  /* allow the methods */
  allow_all_methods();
}

/*!
 * \brief   change the current component
 */
void CornerFinder::setComponent(Component* c,
                                cv::Rect & bbox, cv::Point & center) {
  component = c;
  center_of_component = center;
  // change bbox
  bounding_box = bbox;
  xMin = bounding_box.x;
  yMin = bounding_box.y;
  xMax = xMin + bounding_box.width - 1;
  yMax = yMin + bounding_box.height - 1;
}

/*!
 * \brief   change the image used in method 3
 */
void CornerFinder::set_M3_frame_size(cv::Size s) {
  M3_frame_size = s;
}

/*!
 * \brief   copying the results in the argument
 */
void CornerFinder::getCorners(Component & rep) {
  rep.clear();
  for (Component::iterator it = found_corners.begin(); it
       < found_corners.end(); ++it)
    rep.push_back(*it);
}

/*!
 * \brief   a routine to find corners
 */
void CornerFinder::findCorners() {
  DEBUG_PRINT("findCorners(M10: %i, M15: %i, M17: %i, M20: %i, M30: %i)\n",
               M10_allowed, M15_allowed, M17_allowed, M20_allowed, M30_allowed);

  found_corners.clear();
  were_corners_found = false;
  successful_method_number = -1;

  if (M10_allowed || M15_allowed || M17_allowed) {
    findCorners10();
    if (found_corners.size() == 4 && M10_allowed) {
      successful_method_number = 10;
      were_corners_found = true;
      return;
    }

    if (M17_allowed) {
      findCorners17(); /* don't call it without findCorners10() before ! */
      if (found_corners.size() == 4) {
        successful_method_number = 17;
        were_corners_found = true;
        return;
      }
    } // end of if M17

    if (M15_allowed) {
      findCorners15(); /* don't call it without findCorners10() before ! */
      if (found_corners.size() == 4) {
        successful_method_number = 15;
        were_corners_found = true;
        return;
      }
    } // end of if M15
  } // end of if M10, M15, M17

  if (M20_allowed) {
    findCorners20();
    if (found_corners.size() == 4) {
      successful_method_number = 20;
      were_corners_found = true;
      return;
    }
  }

  if (M30_allowed) {
    findCorners30();
    if (found_corners.size() == 4) {
      successful_method_number = 30;
      were_corners_found = true;
      return;
    }
  }

  // if we reach here, it is a failure
}

/*!
 * \brief   a qucik and dirty method to find corners
 *
 * It makes an histogram of furthest M30_border_Points around the center of the zone,
 * and keeps the corners for which the sectors are the highest,
 * compared to left and right neighbors.
 *
 * It is not very robust, it can return for instance 2, or 5 M30_border_Points
 */
void CornerFinder::findCorners10() {
  DEBUG_PRINT("findCorners10()\n");

  double* biggestDistancePtr = M10_biggestDistance_slice;
  for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i)
    *biggestDistancePtr++ = 0;

  /* compute the furthest point in each direction */
  double currentDistance, currentAngle;
  for (Component::iterator currentPoint = component->begin();
       currentPoint < component->end(); ++currentPoint) {
    currentDistance = hypot(currentPoint->x - center_of_component.x,
                            currentPoint->y - center_of_component.y);
    currentAngle = atan2(currentPoint->y - center_of_component.y,
                         currentPoint->x - center_of_component.x);
    if (currentAngle < 0)
      currentAngle += 2.f * CV_PI;
    int goodCase = (int) (M10_NB_ANGULAR_SLICES * currentAngle / (2.f
                                                                  * CV_PI));
    //      cout << "center:(" << center.x << "," << center.y << "), "
    //      << "current:(" << currentPoint->x << "," << currentPoint->y << "), "
    //      << "d:" << currentDistance << ", angle:" << currentAngle << ", "
    //      << "good case :" << goodCase << endl;
    //cout << angle << "\t" << distance << endl;
    if (M10_biggestDistance_slice[goodCase] < currentDistance) {
      M10_biggestDistance_slice[goodCase] = currentDistance;
      M10_furthestPoint_slice[goodCase] = *currentPoint;
    }
  }

  found_corners.clear();

  /* keep only the ones which are singles */
  for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i) {
    //double angle = 2.f * CV_PI * i / M10_NB_ANGULAR_SLICES;
    double distCurr = M10_biggestDistance_slice[i];
    bool OK = 1;
    for (int j = 1; j <= M1_HISTSIZE; ++j) {
      if (distCurr <= M10_biggestDistance_slice
          [(i - j + M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES]
          || distCurr <= M10_biggestDistance_slice
          [(i + j + M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES]) {
        OK = 0;
        break;
      }
    }
    if (OK) { /* keep the point */
      // push a point very close to this furthest point (barycenter_of_two_points)
      cv::Point pointToPush = vision_utils::barycenter_of_two_points(
            M1_BARY, M10_furthestPoint_slice[i], center_of_component);
      found_corners.push_back(pointToPush);
      DEBUG_PRINT("M10:pushing back:%i\n", i);
    }
    //cout << "i:" << i << " - angle:" << angle << "\t CURR :" << distCurr << endl;
    //cout << angle << "\t" << distCurr << endl;
  }
  //cout << "###############################" << endl;
}

/*!
 * \brief   a quick and dirty method to find corners
 *
 * It is the same than the method 1, except it compares the derivative of the highest distances,
 * and not the distances themselves.
 *
 * We look for points where the derivative is =0.
 *
 * It is more robust.
 */
void CornerFinder::findCorners15() {
  DEBUG_PRINT("findCorners15()\n");
  found_corners.clear();

  double* corners15_diffs_ptr = M15_distance_diffs;
  for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i)
    *corners15_diffs_ptr++ = M10_biggestDistance_slice[i]
      - M10_biggestDistance_slice[(i - 1 + M10_NB_ANGULAR_SLICES)
      % M10_NB_ANGULAR_SLICES];

  int idx_prev, idx_next, index_to_push;
  double bd_i, bd_next;//, bd_prev = 0, angle = 0;
  double c15d_i, c15d_next, c15d_prev;
  double* biggestDistance_ptr = M10_biggestDistance_slice;
  corners15_diffs_ptr = M15_distance_diffs;

  /* keep only the ones which are singles */
  for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i) {
    //angle = 2.f * CV_PI * i / M10_NB_ANGULAR_SLICES;
    bd_i = *biggestDistance_ptr;
    c15d_i = *corners15_diffs_ptr;

    //      cout << "i:" << i << "\t angle:" << angle
    //      << "\t M10_biggestDistance_slice[i]:" << bd_i
    //      << "\t M15_distance_diffs[i]:" << c15d_i << endl;

    bool OK = 1;
    // we keep only the indices such as the diff is > than before and < than after
    for (int j = 1; j <= M1_HISTSIZE; ++j) {
      c15d_prev = M15_distance_diffs[(i - j + M10_NB_ANGULAR_SLICES)
          % M10_NB_ANGULAR_SLICES];
      c15d_next = M15_distance_diffs[(i + j + M10_NB_ANGULAR_SLICES)
          % M10_NB_ANGULAR_SLICES];
      //          if (c15d_i < M15_distance_diffs[(i - j + M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES]
      //                  || c15d_i
      //                          < M15_distance_diffs[(i + j + M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES]) {
      //              OK = 0;
      //              break;
      //          }
      if (c15d_prev > c15d_i || c15d_next > c15d_i) {
        OK = 0;
        break;
      }
    }
    if (OK) {
      //          cout << "OK:" << i << endl;

      idx_prev = (i - 1 + M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES;
      idx_next = (i + 1 + M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES;
      index_to_push = i;

      /* try to find the best between i, previous and next */
      //bd_prev = M10_biggestDistance_slice[idx_prev];
      bd_next = M10_biggestDistance_slice[idx_next];

      // if i is clearly the best => push it
      if (bd_i > M15_DIFF_RATIO_THRES * bd_next) {
        //&& bd_i > M15_DIFF_RATIO_THRES * bd_prev) {
        //              cout << "case (1)" << endl;
        index_to_push = i;
      }
      // if next is clearly the best => push it
      else if (bd_next > M15_DIFF_RATIO_THRES * bd_i) {
        //&& bd_next > M15_DIFF_RATIO_THRES * bd_prev) {
        //              cout << "case (2)" << endl;
        index_to_push = idx_next;
      }
      //          // if previous is clearly the best => push it
      //          else if (bd_prev > M15_DIFF_RATIO_THRES * bd_i && bd_prev
      //                  > M15_DIFF_RATIO_THRES * bd_next) {
      //              cout << "case (3)" << endl;
      //              index_to_push = idx_prev;
      //          }
      // if nbody is clearly the best, but i has the best derivative => push it
      else {
        c15d_prev = M15_distance_diffs[idx_prev];
        c15d_next = M15_distance_diffs[idx_next];

        if (c15d_i > c15d_next) {
          //&& c15d_i > c15d_prev) {
          //                  cout << "case (4)" << endl;
          index_to_push = i;
        }
        // if nbody is clearly the best, but next has the best derivative => push it
        else if (c15d_next > c15d_i) {
          //&& c15d_next > c15d_prev) {
          //                  cout << "case (5)" << endl;
          index_to_push = idx_next;
        }
        // if nbody is clearly the best, but previous has the best derivative => push it
        //              else {
        //                  cout << "case (6)" << endl;
        //                  index_to_push = idx_prev;
        //              }
      }

      // push a point very close to this furthest point (barycenter_of_two_points)
      cv::Point pointToPush = vision_utils::barycenter_of_two_points(
            M1_BARY, M10_furthestPoint_slice[index_to_push],
            center_of_component);
      found_corners.push_back(pointToPush);
      DEBUG_PRINT("M15:pushing back:%i\n", index_to_push);
    } // end if OK

    ++biggestDistance_ptr;
    ++corners15_diffs_ptr;
  } // end loop i : 0 . NB_SLICES
  //cout << "###############################" << endl;
}

/*!
 * \brief   another method to find the corners
 * It looks the average angle of each slice maximum with its closest neighbours
 * And keep the one with an angle far from 0
 *
 * \param
 * \return
 */
void CornerFinder::findCorners17() {
  DEBUG_PRINT("findCorners17()\n");

  /* compute median angle */
  for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i) {
    cv::Point2f curr = M10_furthestPoint_slice[i];
    double angle_med = 0;
    for (int j = 1; j <= M17_HIST_SIZE; ++j) {
      cv::Point2f prev = //
          M10_furthestPoint_slice[(i - j
                                   + M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES];
      cv::Point2f next = //
          M10_furthestPoint_slice[(i + j
                                   + M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES];
      angle_med += vision_utils::absolute_angle_between_three_points(prev, curr, next);
    }
    angle_med = angle_med / M17_HIST_SIZE;
    while (angle_med > CV_PI / 2)
      angle_med -= CV_PI;
    if (isnan(angle_med))
      angle_med = 0;
    M17_average_angles[i] = fabs(angle_med);
  }

  /* keep the maximum locals */
  found_corners.clear();
  for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i) {
    bool OK = 1;
    if (M17_average_angles[i] < M17_ANGLE_MIN)
      OK = 0;
    else
      for (int j = 1; j <= M17_HIST_SIZE; ++j) {
        if (M17_average_angles[(i - j + M10_NB_ANGULAR_SLICES)
            % M10_NB_ANGULAR_SLICES] > M17_average_angles[i])
          OK = 0;
        else if (M17_average_angles[(i + j + M10_NB_ANGULAR_SLICES)
                 % M10_NB_ANGULAR_SLICES] > M17_average_angles[i])
          OK = 0;
      }

    if (OK) {
      DEBUG_PRINT("M17:pushing back:%i\n", i);
      cv::Point pointToPush = vision_utils::barycenter_of_two_points(
            M1_BARY, M10_furthestPoint_slice[i], center_of_component);
      found_corners.push_back(pointToPush);
    }

    //      cout << endl << "i:" << i << "\t";
    //      cout << " - M10_furthestPoint_slice[i]:("
    //      << M10_furthestPoint_slice[i].x << ","
    //      << M10_furthestPoint_slice[i].y << ")\t";
    //      cout << " - M17_average_angles[i]:" << M17_average_angles[i];
  } // end loop i : 0 . M10_NB_ANGULAR_SLICES

#ifdef PZ_FIND_VIDEOS
  /* drawing the points one after one */
  cv::Mat3b illus (M3_frame_size);
  illus.setTo(0);
  vision_utils::drawListOfPoints( illus, *component, cv::Vec3b(160, 160, 160) );
  for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i) {
    vision_utils::drawCross( illus, M10_furthestPoint_slice[i], 5,
                            vision_utils::color_scalar<cv::Scalar>(i, M10_NB_ANGULAR_SLICES),
                            2 );
    writer_M17.write(illus);
  }

  /* drawing the lines in direction of the neighbours */
  cv::Mat illus2 = illus.clone();
  for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i) {
    illus.copyTo(illus2);
    cv::Point curr = M10_furthestPoint_slice[i];
    // find the angle
    double dir_left = 0, dir_right = 0;
    int on_left = 0, on_right = 0;
    for (int j = 1; j <= M17_HIST_SIZE; ++j) {
      cv::Point left = M10_furthestPoint_slice[(i-j+M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES];
      if (left.x - curr.x != 0 ) {
        dir_left += atan2( left.y - curr.y, left.x - curr.x );
        on_left++;
      }
      cv::Point right = M10_furthestPoint_slice[(i+j+M10_NB_ANGULAR_SLICES) % M10_NB_ANGULAR_SLICES];
      if (right.x - curr.x != 0 ) {
        dir_right += atan2( right.y - curr.y, right.x - curr.x );
        on_right++;
      }
    }
    if (on_left != 0)
      dir_left = dir_left / on_left;
    if (on_right != 0)
      dir_right = dir_right / on_right;
    cv::Point inf_left = cv::Point( (int) (curr.x + 1000 * cos(dir_left)) , (int) (curr.y + 1000 * sin(dir_left)) );
    cv::Point inf_right = cv::Point( (int) (curr.x + 1000 * cos(dir_right)) , (int) (curr.y + 1000 * sin(dir_right)) );
    cv::line( illus2, curr, inf_left,
              vision_utils::color_scalar<cv::Scalar>(i, M10_NB_ANGULAR_SLICES), 2 );
    cv::line( illus2, curr, inf_right,
              vision_utils::color_scalar<cv::Scalar>(i, M10_NB_ANGULAR_SLICES), 2 );

    vision_utils::drawCross( illus2, curr, 20,
                            vision_utils::color_scalar<cv::Scalar>(i, M10_NB_ANGULAR_SLICES), 2 );

    int nb_times = 2;
    double angle_med = dir_right - dir_left + 2 * CV_PI;
    while (angle_med > CV_PI / 2)
      angle_med -= CV_PI;
    angle_med = fabs(angle_med);
    if (angle_med > M17_ANGLE_MIN) {

      cv::putText( illus2, "Potential corner", cv::Point(10,50),
                   cv::FONT_HERSHEY_DUPLEX, 1,
                   vision_utils::color_scalar<cv::Scalar>(i, M10_NB_ANGULAR_SLICES) );
      nb_times = 10;
    }
    for (int var = 0; var < nb_times; ++var)
      writer_M17.write(illus2 );

  }

#endif
}

/*!
 * \brief   evaluate the random evolution of the M20_quad of the method 2
 */
void CornerFinder::findCorners20_eval() {
  double newMark = 0;
  int maxGap = 50;
  for (int i = 0; i < 4; ++i) {
    if (M20_quad[i].x < -maxGap + xMin || M20_quad[i].x > maxGap + xMax)
      newMark = INFINITY;
    if (M20_quad[i].y < -maxGap + yMin || M20_quad[i].y > maxGap + yMax)
      newMark = INFINITY;
  }

  if (newMark != INFINITY)
    for (unsigned int i = 0; i < component->size(); ++i) {
      if (!vision_utils::point_inside_polygon(component->at(i),
                                                M20_quad)) {
        newMark = INFINITY;
        break;
      }
      newMark += vision_utils::distance_point_polygon(component->at(i),
                                                        M20_quad);
    }

  /* ** now, do we keep the change ? ** */
  if (newMark < M20_mark) { /* change accepted */
    //cout << "new M20_mark:" << newMark << " - kept" << endl;
    M20_mark = newMark;
    M20_consec_reverts = 0;
    for (int i = 0; i < 4; ++i) {
      M20_quad_back_up[i].x = M20_quad[i].x;
      M20_quad_back_up[i].y = M20_quad[i].y;
    }

    /* display */
#ifdef PZ_FIND_VIDEOS
    cv::Mat3b illus (M3_frame_size);
    illus.setTo(0);
    vision_utils::drawListOfPoints( illus, *component, cv::Vec3b(255) );
    vision_utils::drawPolygon(illus, M20_quad, true, CV_RGB(0,255,0), 2);
    //      cv::imshow(window1Name, illus);
    //      cv::waitKey( 2);
    writer_M20.write(illus);
#endif

  } else { /* revert ! */
    //cout << "new M20_mark:" << newMark << " - revert" << endl;
    /* display */
#ifdef PZ_FIND_VIDEOS
    cv::Mat3b illus (M3_frame_size);
    illus.setTo(0);
    vision_utils::drawListOfPoints( illus, *component, cv::Vec3b(255) );
    vision_utils::drawPolygon(illus, M20_quad, true, CV_RGB(255, 0, 0), 2);
    //      cv::imshow(window1Name, illus);
    //      cv::waitKey( 2);
    std::ostringstream txt;
    txt << "Reverts :" << M20_consec_reverts;

    cv::putText( illus, txt.str().c_str(), cv::Point(10,50),
                 cv::FONT_HERSHEY_DUPLEX, 1, cvScalarAll(255) );
    writer_M20.write(illus );
#endif

    ++M20_consec_reverts;
    M20_quad.clear();
    // copy M20_quad_back_up -> M20_quad
    M20_quad.insert(M20_quad.begin() , M20_quad_back_up.begin() , M20_quad_back_up.end());
  }
}

/*!
 * \brief   a randomized method to find corners.
 *
 * It initiates a quadrilater with the bounding box of the M30_border_Points,
 * and will try to rotate randomly the M20_quad and to move randomly the corner.
 *
 * Each new shape of the M20_quad is evaluated. If a point is outside, then the M20_mark is INFTY.
 * Else, M20_mark = sum {for p in M30_border_Points} (dist(p, M20_quad)) .
 * If this M20_mark is worse than before, then we revert the change.
 * Else we keep it.
 */
void CornerFinder::findCorners20() {
  DEBUG_PRINT("findCorners20()\n");

  /* init our moving M20_quad */
  M20_quad.clear();
  M20_quad.push_back(cv::Point(xMin - 3, yMin - 3));
  M20_quad.push_back(cv::Point(xMin - 3, yMax + 3));
  M20_quad.push_back(cv::Point(xMax + 3, yMax + 3));
  M20_quad.push_back(cv::Point(xMax + 3, yMin - 3));

  M20_mark = INFINITY; // the M20_mark of the rectangle
  M20_quad_back_up.clear();
  M20_quad_back_up.insert(M20_quad_back_up.begin(),
                          M20_quad.begin(), M20_quad.end());

  /* the termination limit */
  M20_consec_reverts = 0;
  // do we quit the loop ?
  bool quit_loop = 0;
  // 1 at beginning, almost 0 at end
  double speed;
  // a size factor, = 1 for a big squares, more small for more small shapes
  double sizeFactor = fmax(0, fmin(1, (xMax - xMin) / 200.f));
  // variables evol1
  double theta, module;
  int corner_to_evol;
  // variables evol2
  double dilat;
  // variables evol3
  double ratio;
  int direction, corner1, corner2;

  while (!quit_loop) {
    speed = 1.f - 1.f * M20_consec_reverts / M2_MAX_CONSEC_REVERTS;

    /* random evolution of the M20_quad */
    int type_evol = rand() % 3;

    if (type_evol == 0) { /* evol = aleatory */
      corner_to_evol = rand() % 4;
      theta = atan2(center_of_component.y - M20_quad[corner_to_evol].y,
                    center_of_component.x - M20_quad[corner_to_evol].x); // direction = center
      theta += (-1 + 2 * (rand() % 2)) * CV_PI / 4.f; // +/- Pi/4
      theta += vision_utils::rand_gaussian() * .1f * CV_PI; // + some random
      module = sizeFactor * (5 + vision_utils::rand_gaussian());

      M20_quad[corner_to_evol].x += (int) (module * cos(theta));
      M20_quad[corner_to_evol].y += (int) (module * sin(theta));

      findCorners20_eval();
      if (M20_consec_reverts > M2_MAX_CONSEC_REVERTS)
        quit_loop = 1;

    } else if (type_evol == 1) { /* evol = rotate */
      theta = CV_PI * speed * 30 / 180.f * vision_utils::rand_gaussian();
      dilat = sizeFactor * (1.f + .10 * speed
                            * vision_utils::rand_gaussian());
      vision_utils::rotate_dilate_polygon(M20_quad, theta, dilat);

      findCorners20_eval();
      if (M20_consec_reverts > M2_MAX_CONSEC_REVERTS)
        quit_loop = 1;

    } else { /* evol = barycenter_of_two_points of two M30_border_Points */
      direction = -1 + 2 * (rand() % 2);
      //int corner1 = rand() % 4;
      for (corner1 = 0; corner1 < 4; ++corner1) {
        ratio = .1f * (1.f + vision_utils::rand_gaussian());
        corner2 = (corner1 + 4 + direction) % 4;

        M20_quad[corner1] = vision_utils::barycenter_of_two_points(
              1 - ratio, M20_quad[corner1], M20_quad[corner2]);

        findCorners20_eval();
        if (M20_consec_reverts > M2_MAX_CONSEC_REVERTS) {
          break;
          quit_loop = 1;
        }
      } // end of corner loop

    } // end of type_evol
    // the display is in the eval function
  } // end of while loop

  /* copy the results */
  found_corners.clear();
  for (int i = 0; i < 4; ++i)
    found_corners.push_back(vision_utils::barycenter_of_two_points(93
                                                                     / 100.f, cv::Point(M20_quad[i].x, M20_quad[i].y),
                                                                     center_of_component));
}

/*!
 * \brief   a method to find corners - it uses openCV methods.
 *
 * First you detect the external contour of the component,
 * then you simplify this contour to obtain only a polygon with few points.
 * Then, you compute all the lines between each point and the next one.
 * Then, you group this lines into 4 clusters usnig kMeans.
 * Then, you compute the average equation of each cluster,
 * normally each correspond to the equation of a side of the quad.
 * Finally, you compute the intersection between the lines.
 */
void CornerFinder::findCorners30() {
  DEBUG_PRINT("findCorners30()\n");

  /* copy the M30_border_Points of the current components */
  // DEBUG_PRINT("M3_frame_size:%ix%i\n", M3_frame_size.width, M3_frame_size.height);
  M30_bestComponent.create(M3_frame_size);
  M30_bestComponent = 0;
  vision_utils::drawListOfPoints(M30_bestComponent, *component,
                                (uchar) 255);

  /* compute the contour */
  //    findContours( Mat& image, CV_OUT vector<vector<Point> >& contours,
  //                 int mode, int method, Point offset=Point());
  cv::findContours(M30_bestComponent, M30_contours,
                   CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  M30_border_Points = M30_contours.front();

  /* draw */
#ifdef PZ_FIND_IMAGES
  // the contour
  cv::Mat3b contour_frame0(M3_frame_size);
  contour_frame0.setTo(0);
  vision_utils::drawListOfPoints(contour_frame0, *component,
                                cv::Vec3b(255, 255, 255));

  //    drawContours( Mat& image, const vector<vector<Point> >& contours,
  //                  int contourIdx, const Scalar& color);
  cv::drawContours( contour_frame0, M30_contours, 0, CV_RGB(255,0,255), 3);
  // draw the points of the first comp of M30_contours
  vision_utils::drawListOfPoints(contour_frame0, M30_contours.at(0),
                                cv::Vec3b(100,100,255));

  // lineswriter_M20
  //  for (int i = 0; i < points0.size()-1; ++i) {
  //      cv::Point3f e = vision_utils::line_equation(points0.at(i),
  //              points0.at( (i+4) % points0.size()) );
  //      cout << "equa:x" << e.x << " - y:" << e.y << " - z:" << e.z << endl;
  //      vision_utils::drawLine(contour_frame0, e.x, e.y, e.z, CV_RGB(0,0,255), 1);
  //  }

  cv::imwrite("Method30-1-contour0.png", contour_frame0);
#endif

  /* approximate it */
  //    approxPolyDP( const Mat& curve, CV_OUT vector<Point2f>& approxCurve,
  //                 double epsilon, bool closed );
  cv::approxPolyDP(cv::Mat(M30_border_Points), M30_border_Points,
                   10,  0);

  //DEBUG_PRINT("nb M30_border_Points:%i\n", M30_border_Points.size() );

  /* compute the lines */
  cv::Mat equations (M30_border_Points.size(), 3, CV_32F);
  cv::Point3f eqn;
  for (unsigned int i = 0; i < M30_border_Points.size(); ++i) {
    eqn = vision_utils::line_equation<cv::Point, cv::Point3f>
        (M30_border_Points.at(i),
         M30_border_Points.at((i + 1) % M30_border_Points.size()));
    // normalization
    // already done in the image_utils function
    //      if (eqn.y != 0) {
    //          eqn.x = eqn.x / eqn.y;
    //          eqn.z = eqn.z / eqn.y;
    //          eqn.y = 1;
    //      }
    // copy
    equations.at<float>(i, 0) = eqn.x;
    equations.at<float>(i, 1) = eqn.y;
    equations.at<float>(i, 2) = eqn.z;
  }

  /* kMeans - https://github.com/Itseez/opencv/blob/master/samples/cpp/kmeans.cpp */
  cv::Mat labels, centers;
  // http://docs.opencv.org/modules/core/doc/clustering.html#kmeans
  //    double kmeans( InputArray data, int K, CV_OUT InputOutputArray bestLabels,
  //                  TermCriteria criteria, int attempts,
  //                  int flags, OutputArray centers=noArray() );
  cv::kmeans(equations,  4,  labels,
             cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 1E-5),
              3, cv::KMEANS_PP_CENTERS, centers);

  /* compute the average */
  for (int i = 0; i < 4; ++i) {
    M30_average_equations[i].x = centers.at<float>(i, 0);
    M30_average_equations[i].y = centers.at<float>(i, 1);
    M30_average_equations[i].z = centers.at<float>(i, 2);
  }

  /* compute the intersections */
  found_corners.clear();
  for (int i = 0; i < 4; ++i)
    for (int j = i + 1; j < 4; ++j) {
      cv::Point2f inter = vision_utils::interLine<cv::Point2f>(
            M30_average_equations[i].x, M30_average_equations[i].y,
            M30_average_equations[i].z, M30_average_equations[j].x,
            M30_average_equations[j].y, M30_average_equations[j].z);
      //printf("M30:inter (%g, %g)\n", inter.x, inter.y);
      // copy the results
      int bd = 0;
      if (inter.x >= xMin - bd && inter.x <= xMax + bd
          && inter.y >= yMin - bd && inter.y <= yMax + bd) {
        found_corners.push_back(inter);
        DEBUG_PRINT("M30:pushing back (%g, %g)\n", inter.x, inter.y);
      }
    } // end loop j
} // end find_corners30()
