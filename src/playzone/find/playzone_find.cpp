/***************************************************************************//**
 * \file PlayzoneFind.cpp
 *
 * \brief The implementation of playzone_find.h
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 25/05/2009
 *******************************************************************************/

/*! uncomment to use an adaptative thresold instead of the simple value one
  (it will become slower, but better for poor light conditions) */
#define USE_ADAPTATIVE_THRES

/*! use the "open" morphological filter to go faster on components labelling
  (it will become slower and worse) */
//#define THRES_OPEN_IMAGE

/*! uncomment to use the contour finding instead of the connected comps
  (it will become faster but worse) */
//#define USE_CONTOURS

/*! uncomment to compare the components to a flat view of the playzone
    AND also a tilted view
    (it will become worse) */
//#define USE_TWO_PATTERNS

///// my imports
#include "games_vision/playzone_find.h"
// vision_utils
#include "vision_utils/barycenter4.h"
#include "vision_utils/color_utils.h"
#include "vision_utils/drawListOfPoints.h"
#include "vision_utils/hausdorff_distances.h"
#include "vision_utils/hausdorff_distances.h"
#include <vision_utils/img_path.h>
#include "vision_utils/nonNulPoints.h"
#include "vision_utils/redimContent.h"
#include "vision_utils/user_image_to_rgb.h"
// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   ROS_INFO(__VA_ARGS__)

///////////////////////////////////////////////////////////////////////

PlayzoneFind::Times::Times() {
  clear_times();
}

///////////////////////////////////////////////////////////////////////

void PlayzoneFind::Times::clear_times() {
  t01_after_thresh_image = vision_utils::Timer::NOTIME;
  t02_after_numeroteComponents = vision_utils::Timer::NOTIME;
  t03_after_compareComponents = vision_utils::Timer::NOTIME;
  t04_after_getCorners = vision_utils::Timer::NOTIME;
  t05_after_saving_images = vision_utils::Timer::NOTIME;
  t06_after_removeBorder2 = vision_utils::Timer::NOTIME;
  t07_after_calc_warp_matrix = vision_utils::Timer::NOTIME;
  t08_after_rectif_image = vision_utils::Timer::NOTIME;
  t_after_everything = vision_utils::Timer::NOTIME;
}

///////////////////////////////////////////////////////////////////////

std::string PlayzoneFind::Times::to_string() const {
  std::ostringstream ans;
  ans << (int) t01_after_thresh_image << ", ";
  ans << (int) t02_after_numeroteComponents << ", ";
  ans << (int) t03_after_compareComponents << ", ";
  ans << (int) t04_after_getCorners << ", ";
  ans << (int) t05_after_saving_images << ", ";
  ans << (int) t06_after_removeBorder2 << ", ";
  ans << (int) t07_after_calc_warp_matrix << ", ";
  ans << (int) t08_after_rectif_image << ", ";
  ans << (int) t_after_everything << " ms";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

PlayzoneFind::PlayzoneFind(const int BOARD_OUT_WIDTH_,
                           const int BOARD_OUT_HEIGHT_) :
  _current_status(NEVER_RUN),
  BOARD_OUT_WIDTH(BOARD_OUT_WIDTH_),
  BOARD_OUT_HEIGHT(BOARD_OUT_HEIGHT_)
{
  DEBUG_PRINT("PlayzoneFind:PlayzoneFind ctor\n");
  best_component_points = NULL;

  /* creating the frames */
  //rgb.create(1, 1);
  frame_buffer.create(1, 1);
  frameBW.create(1, 1);

#ifdef PZ_FIND_IMAGES
  frameOut.create(1, 1);
#endif // PZ_FIND_IMAGES

  /* init the other things */
  find_playzone_init();
  //corner_finder.findCorners_init();
  rectif_image_init();
  removeBorder_init();
}

////////////////////////////////////////////////////////////////////////////////

PlayzoneFind::~PlayzoneFind() {
}

////////////////////////////////////////////////////////////////////////////////

/* init the video writer */
#ifdef PZ_FIND_VIDEOS
cv::VideoWriter writer_components ("components.avi",codec, 10, cv::Size(640, 480));
#endif

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::set_input(const cv::Mat3b* new_frame) {
  DEBUG_PRINT("PlayzoneFind:set_input('%s')\n", vision_utils::infosImage(*new_frame).c_str());
  _frame = new_frame;
}

////////////////////////////////////////////////////////////////////////////////

const cv::Mat3b* PlayzoneFind::get_playzone() const {
  return &rectifiedImage;
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::display() const {
  cv::imshow("frame_thres", frame_thres);
  cv::Mat3b frame_thres2comps (frame_buffer.size(), cv::Vec3b(0, 0, 0));
  for (unsigned int comp_idx = 0; comp_idx < components_pts.size(); ++comp_idx)
    vision_utils::drawListOfPoints(frame_thres2comps, components_pts[comp_idx],
                                  vision_utils::color<cv::Vec3b>());
  vision_utils::drawListOfPoints(frame_thres2comps, *best_component_points,
                                cv::Vec3b(255, 255, 255));
  cv::imshow("frame_thres2comps", frame_thres2comps);
  cv::imshow("rectifiedImage", rectifiedImage);
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::get_corner_list(std::vector<cv::Point2i> & ans) const {
  ans.clear();
  ans = board_corners;
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::find_playzone_init() {
  DEBUG_PRINT("PlayzoneFind:find_playzone_init()\n");

  /* init image */
  //frame_thres.create(1, 1);
  //HSV.create(1, 1);
  //Value_comp.create(1, 1);

  /* init rand */
  srand((unsigned) time(0));

  /* define model_points with the non null pts */
  std::vector<std::string> model_image_filenames;
  model_image_filenames.push_back(vision_utils::IMG_DIR() + "board/shape_flat.png");
#ifdef USE_TWO_PATTERNS
  model_image_filenames.push_back(vision_utils::IMG_DIR() + "board/shape_twist.png");
#endif // USE_TWO_PATTERNS

  model_points.clear();
  for(std::vector<std::string>::const_iterator file = model_image_filenames.begin();
      file != model_image_filenames.end() ; ++file) {
    cv::Mat1b model_image = cv::imread(*file, CV_LOAD_IMAGE_GRAYSCALE);
    model_size = cv::Rect(0, 0, model_image.cols, model_image.rows);
#ifdef USE_CONTOURS
    std::vector<std::vector<cv::Point> > comps;
    cv::findContours(model_image, comps, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    model_points.push_back( comps.at(0) );
#else // USE_CONTOURS
    Component model_comp;
    vision_utils::nonNulPoints(model_image, model_comp);
    model_points.push_back( model_comp );
#endif
    DEBUG_PRINT("PlayzoneFind:MODEL : size:(%i, %i), size of model_points:%i\n",
           model_size.width, model_size.height, (int) model_points.back().size());
    model_image.release();
  } // end loop file

  resized_array.create(model_size.height, model_size.width);

  /* prepare matrix */
  warp_matrix.create(3, 3, CV_32FC1);
  board_corners.clear();
}

////////////////////////////////////////////////////////////////////////////////

bool PlayzoneFind::was_playzone_found() const {
  return (get_current_status() == SUCCESS_FOUND);
}

////////////////////////////////////////////////////////////////////////////////

PlayzoneFind::Times PlayzoneFind::get_current_times() const {
  return _current_times;
}

////////////////////////////////////////////////////////////////////////////////

PlayzoneFind::Status PlayzoneFind::get_current_status() const {
  return _current_status;
}

////////////////////////////////////////////////////////////////////////////////

std::string PlayzoneFind::get_current_status_as_string() const {
  switch (_current_status) {
    case NEVER_RUN:
      return "never run.";
      break;
    case SEARCHING:
      return "searching...";
      break;
    case SUCCESS_FOUND:
      return "success, found.";
      break;
    case FAILURE_NO_GOOD_COMP:
      return "failure, no good component.";
      break;
    case FAILURE_NO_CORNERS_FOUND:
      return "failure, no corners found.";
      break;
  }
  return "unknwon.";
}

////////////////////////////////////////////////////////////////////////////////

bool PlayzoneFind::find_playzone() {
  DEBUG_PRINT("PlayzoneFind:find_playzone()\n");
  find_playzone_part1();
  return find_playzone_part2();
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::find_playzone_part1() {
  DEBUG_PRINT("PlayzoneFind:find_playzone_part1()\n");

  //was_playzone_found() = true;
  _current_status = SEARCHING;

  /* acquire images */
  _frame->copyTo(frame_buffer);

  global_timer.reset();
  _current_times.clear_times();

  /* save images */
#ifdef SAVE_IMAGES
  cv::imwrite("010-frame_buffer.png", frame_buffer);
#endif

  /* compute the monochrome image */
  step_timer.reset();
  thresh_image();
  DEBUG_PRINT("Time for after thresh_image():%g ms", global_timer.time());
  _current_times.t01_after_thresh_image = step_timer.getTimeMilliseconds();

  /* save images */
#ifdef SAVE_IMAGES
  cv::imwrite("020-frame_thres.png", frame_thres);
#endif
}

////////////////////////////////////////////////////////////////////////////////

bool PlayzoneFind::find_playzone_part2(bool rectif_playzone /*= true*/) {
  DEBUG_PRINT("PlayzoneFind:find_playzone_part2()\n");

  /* find components */
  step_timer.reset();
  numeroteComponents();
  DEBUG_PRINT("Time for after numeroteComponents():%g ms", global_timer.time());
  _current_times.t02_after_numeroteComponents = step_timer.getTimeMilliseconds();

  /* find the best */
  step_timer.reset();
  compareComponents();
  DEBUG_PRINT("Time for after compareComponents():%g ms", global_timer.time());
  _current_times.t03_after_compareComponents = step_timer.getTimeMilliseconds();

  if (_current_status == FAILURE_NO_GOOD_COMP) {
    printf("PlayzoneFind:There was no good component. _current_status = FAILURE_NO_GOOD_COMP.\n");
    rectifiedImage.setTo(0);
    return false;
  }

  /* find corners */
  DEBUG_PRINT("PlayzoneFind:findCorners()\n");
  step_timer.reset();
  corner_finder.setComponent(best_component_points, best_component_bbox,
                             best_component_center);
  //corner_finder.set_M3_frame_size(frame_buffer.size());
  corner_finder.set_M3_frame_size(_frame->size());
  corner_finder.allow_all_methods();
  corner_finder.M20_allowed = false;
  corner_finder.findCorners();
  corner_finder.getCorners(board_corners);
  DEBUG_PRINT("Time for after getCorners():%g ms", global_timer.time());
  _current_times.t04_after_getCorners = step_timer.getTimeMilliseconds();

  if (!corner_finder.were_corners_found) {
    _current_status = FAILURE_NO_CORNERS_FOUND;
    printf("PlayzoneFind:The angles were not found. _current_status = FAILURE_NO_CORNERS_FOUND.\n");
    rectifiedImage.setTo(0);
    return false;
  }

  /* save images */
#ifdef PZ_FIND_IMAGES
  frame_buffer.copyTo(frameOut);
  int i = 0;

  // make the image brighter
  cv::add(frameOut, cv::Scalar::all(140), frameOut);

  for (Component::iterator it = board_corners.begin();
       it < board_corners.end(); ++it) {
    cv::circle(frameOut, *it, 3, CV_RGB(0,0,0), 2);
    std::ostringstream m;
    m << (i++);
    cv::putText(frameOut, m.str().c_str(), cv::Point(it->x + 10, it->y),
                cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0,0,0));
  }

  step_timer.reset();
  cv::imwrite("030-frameOut.png", frameOut);
  DEBUG_PRINT("Time for after saving images:%g ms", global_timer.time());
  _current_times.t05_after_saving_images = step_timer.getTimeMilliseconds();
  step_timer.reset();
#endif

  /* remove the border */
  step_timer.reset();
  removeBorder2();
  DEBUG_PRINT("Time for after removeBorder2():%g ms", global_timer.time());
  _current_times.t06_after_removeBorder2 = step_timer.getTimeMilliseconds();

  /* find the matrix */
  if (rectif_playzone) {
    step_timer.reset();
    calc_warp_matrix();
    DEBUG_PRINT("Time for after calc_warp_matrix():%g ms", global_timer.time());
    _current_times.t07_after_calc_warp_matrix = step_timer.getTimeMilliseconds();

    step_timer.reset();
    rectif_image();
    DEBUG_PRINT("Time for after rectif_image():%g ms", global_timer.time());
    _current_times.t08_after_rectif_image = step_timer.getTimeMilliseconds();

    /* save images 2 */
#ifdef SAVE_IMAGES
    cv::imwrite("140-rectifiedImage.png", rectifiedImage);
#endif
  } // end if rectif_playzone

  DEBUG_PRINT("Time for after everything:%g ms", global_timer.time());
  _current_times.t_after_everything = global_timer.getTimeMilliseconds();

  // we succeeded !
  _current_status = SUCCESS_FOUND;

  /*
     * draw nice images
     * for the paper
     */
#ifdef ARTICLE_METHODS_COMP
  /*
     * METHOD 15
     */
  std::vector<int> method_names;
  method_names.push_back(10);
  method_names.push_back(15);
  method_names.push_back(17);
  method_names.push_back(20);
  method_names.push_back(30);
  for(unsigned int method_it = 0 ; method_it < method_names.size() ; ++method_it) {
    int method_idx = method_names.at(method_it);
    DEBUG_PRINT("PlayzoneFind:Generating results for method %i...\n", method_idx);

    std::ostringstream method_str;
    method_str << "Method" << method_idx;
    std::string method_name = method_str.str();

    timer.reset();
    corner_finder.forbid_all_methods();
    if (method_idx == 10)
      corner_finder.M10_allowed = true;
    else if (method_idx == 15)
      corner_finder.M15_allowed = true;
    else if (method_idx == 17)
      corner_finder.M17_allowed = true;
    else if (method_idx == 20)
      corner_finder.M20_allowed = true;
    else if (method_idx == 30)
      corner_finder.M30_allowed = true;

    corner_finder.findCorners();
    corner_finder.getCorners(board_corners);
    calc_warp_matrix();
    rectif_image();
    timer.printTime(method_name.c_str());
    method_str.str("");
    method_str << method_name << ".png";
    cv::imwrite(method_str.str(), rectifiedImage);

    /* display */

    // convert frame to light grey
    cv::cvtColor(frame, frameBW, CV_RGB2GRAY);
    cv::cvtColor(frameBW, frameOut, CV_GRAY2RGB);
    cv::add(frameOut, CV_RGB(90, 90, 90), frameOut); // make the image more grey

    //////////////////////////////////////////////////////// M10
    if (method_idx == 10) {
      // histogram picture
      double max = 200;
      cv::Mat3b histo (frame_buffer.size() );
      histo.setTo(0);
      for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i) {
        int x_b = (int) (frame_buffer.cols * i / M10_NB_ANGULAR_SLICES );
        int x_f = (int) (frame_buffer.cols * (i+1) / M10_NB_ANGULAR_SLICES );
        int y_f = (int) (frame_buffer.rows
                         * (1.f - corner_finder.M10_biggestDistance_slice[i] / max) );
        cv::rectangle(histo, cv::Point(x_b, frame_buffer.rows), cv::Point(x_f, y_f), //
                      //vision_utils::color(i, M10_NB_ANGULAR_SLICES),
                      vision_utils::color_scalar<cv::Scalar>(i % 2), -1);
      }
      cv::imwrite("Method10-histo.png", histo);
      histo.release();

      for (int i = 0; i < M10_NB_ANGULAR_SLICES; ++i) {
        double r = corner_finder.M10_biggestDistance_slice[i];
        double ratio = 1.5; //1.0
        // draw the line
        cv::Point end;
        end.x = (int) (best_component_center.x + ratio * r *
                       cos(i * 2 * CV_PI / M10_NB_ANGULAR_SLICES) );
        end.y = (int) (best_component_center.y + ratio * r *
                       sin(i * 2 * CV_PI / M10_NB_ANGULAR_SLICES) );
        cv::line(frameOut, corner_finder.center_of_component, end,
                 vision_utils::color_scalar<cv::Scalar>(i%2));
        // draw the cross
        vision_utils::drawCross(frameOut,
                               corner_finder.M10_furthestPoint_slice[i], 5,
                               vision_utils::color_scalar<cv::Scalar>(i%2), 1);
        // write the index
        std::ostringstream m;
        m << i;
        cv::putText(frameOut, m.str(), end, cv::FONT_HERSHEY_DUPLEX, 1.0f,
                    vision_utils::color_scalar<cv::Scalar>(i%2) );
      } // end loop i
    }
    // //////////////////////////////////////////////////////// end M10

    //////////////////////////////////////////////////////// M30
    if (method_idx == 30) {
      // the contour
      cv::Mat3b best_component_img (frame_buffer.size() );
      best_component_img.setTo(0);
      vision_utils::drawListOfPoints(best_component_img, best_component_points,
                                    cv::Vec3b(255, 255, 255));

      // draw the simplified contours
      cv::Mat3b contours = best_component_img.clone();
      //    drawContours( Mat& image, const vector<vector<Point> >& contours,
      //                  int contourIdx, const Scalar& color
      cv::drawContours(contours, corner_finder.M30_contours, 0,
                       CV_RGB(255,0,255), 2);
      // draw a cross for each point of the contour
      for (Component::iterator it = corner_finder.M30_border_Points.begin();
           it < corner_finder.M30_border_Points.end(); ++it)
        vision_utils::drawCross(contours, *it, 25, CV_RGB(100,100,255), 2);
      // bbox
      //cv::rectangle(contour_frame, cv::Point(xMin, yMin), cv::Point(xMax, yMax), CV_RGB(255,0,0), 2);
      cv::imwrite("Method30-2-contour.png", contours);

      // the results
      cv::Mat3b illus = best_component_img.clone();
      for (int i = 0; i < 4; ++i)
        vision_utils::drawLine(illus, corner_finder.M30_average_equations[i].x,
                              corner_finder.M30_average_equations[i].y,
                              corner_finder.M30_average_equations[i].z, CV_RGB(255,0,0));
      for (unsigned int i = 0; i < board_corners.size(); ++i)
        vision_utils::drawCross(illus, board_corners.at(i), 40, CV_RGB(0, 255, 0), 8);
      cv::imwrite("Method30-3-results.png", illus);
    }
    // ///////////////////////////////////////////// end method 30

    for (unsigned int corner_idx = 0 ; corner_idx < board_corners.size() ; ++corner_idx) {
      // line between this corner and the next
      cv::line(frameOut,
               board_corners.at(corner_idx),
               board_corners.at( (corner_idx == board_corners.size() -1 ?
                                    0 : corner_idx + 1) ),
               CV_RGB(255, 0, 0), 2 );
      // circle for the position of this corner
      cv::circle(frameOut, board_corners.at(corner_idx), 3, CV_RGB(255,0,0), 2);
    }

    method_str.str("");
    method_str << method_name << "_res.png";
    cv::imwrite(method_str.str(), frameOut);
  } // end loop method_it

#endif
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::thresh_image() {
  DEBUG_PRINT("PlayzoneFind:thresh_image()\n");

  cv::cvtColor(frame_buffer, frameBW, CV_BGR2GRAY); // grayscale conversion
  DEBUG_PRINT("PlayzoneFind:frameBW:%s\n", vision_utils::infosImage(frameBW).c_str());

#ifdef USE_ADAPTATIVE_THRES
  cv::adaptiveThreshold(frameBW, frame_thres, 255,
                        //cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::ADAPTIVE_THRESH_MEAN_C,
                        cv::THRESH_BINARY_INV,
                        ADAPTATIVE_THRES_SIZE,
                        ADAPTATIVE_THRES_C);

#else // USE_ADAPTATIVE_THRES
  // higher => more white
  cv::threshold(frameBW, frame_thres, LIGHTNESS_THRES, 255, CV_THRESH_BINARY_INV);
  //    cv::imshow("frame_thres", frame_thres);
  //    cv::waitKey(0);

  //    /* second method : with a value thres */
  //    vision_utils::rgb2value(frame, HSV, Value_comp);
  //    // higher => more white
  //    cv::threshold(Value_comp, Value_comp, VALUE_THRES, 255, CV_THRESH_BINARY_INV);
  //    //    cv::imshow("Value_comp", Value_comp);
  //    //    cv::waitKey(0);


  //    /* combine these two methods */
  //    cv::bitwise_and(frame_thres, Value_comp, frame_thres);

#endif
  //cv::imshow("frame_thres", frame_thres); cv::waitKey(0);

#ifdef THRES_OPEN_IMAGE
  cv::imwrite("019-frame_thres_before_dilate.png", frame_thres);
  cv::erode(frame_thres,frame_thres,cv::Mat());
  //cv::dilate(frame_thres,frame_thres,cv::Mat());
#endif
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::numeroteComponents() {
  DEBUG_PRINT("PlayzoneFind:numeroteComponents()\n");

#ifdef USE_CONTOURS
  cv::findContours(frame_thres, components_pts, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

#else // USE_CONTOURS
  set.process_image(frame_thres);
  set.get_connected_components(frame_thres.cols, components_pts, boundingBoxes);
#endif // USE_CONTOURS
  nbComponents = components_pts.size();
  DEBUG_PRINT("PlayzoneFind:components:%i\n", nbComponents);

  /* display for the paper */
#if defined PZ_FIND_VIDEOS || defined PZ_FIND_IMAGES
  cv::Mat3b frame_thres2comps (frame_buffer.size(), cv::Vec3b(0, 0, 0));
  for (unsigned int comp_idx = 0; comp_idx < components_pts.size(); ++comp_idx)
    vision_utils::drawListOfPoints(frame_thres2comps, components_pts[comp_idx], vision_utils::color());
#ifdef PZ_FIND_IMAGES
  cv::imwrite("025-illus.png", illus);
#endif // PZ_FIND_IMAGES
#ifdef PZ_FIND_VIDEOS
  for (int var = 0; var < 10; ++var)
    writer_components.write(illus);
#endif // PZ_FIND_VIDEOS
#endif // PZ_FIND_IMAGES || PZ_FIND_VIDEOS
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::compareComponents() {
  DEBUG_PRINT("PlayzoneFind:compareComponents()\n");

  best_component_mark = ASSOC_MAX_DIST;
  best_component_index = -1;

  // init the pointers and go one element backward as there is
  // an increment at the beginning of each loop
#ifndef USE_CONTOURS
  std::vector<cv::Rect>::const_reverse_iterator bbox_curr = boundingBoxes.rbegin() - 1;
#endif // USE_CONTOURS
  std::vector<Component >::const_reverse_iterator points_curr = components_pts.rbegin() - 1;

  for (int i = nbComponents - 1; i >= 0; --i) {
    // decrement here
#ifndef USE_CONTOURS
    ++bbox_curr;
#endif // USE_CONTOURS
    ++points_curr;

#ifdef PZ_FIND_VIDEOS
    cv::Vec3b color = vision_utils::color();
    cv::Mat3b illus(frame_buffer.size());
    illus.setTo(0);
    vision_utils::drawListOfPoints( illus, *points_curr, color );
#endif

    //// shapes too little -> zap
#ifndef USE_CONTOURS
    if (bbox_curr->width < ASSOC_MIN_WIDTH
        || bbox_curr->height < ASSOC_MIN_HEIGHT) {
#ifdef PZ_FIND_VIDEOS
      // if it is just a few pixels, do nothing
      if (bbox_curr->width * bbox_curr->height < 50)
        continue;
      // write it on the video
      cv::putText( illus, "component too little",
                   cv::Point (10, frame_buffer.rows - 50),
                   cv::FONT_HERSHEY_DUPLEX, 1, cvScalarAll(255) );
      writer_components.write(illus);
#endif
      continue;
    }
#endif // USE_CONTOURS

    //// window two little => pass
    if (points_curr->size() < 70) {
      //DEBUG_PRINT("PlayzoneFind:Too few points in the component\n");
#ifdef PZ_FIND_VIDEOS
      // if it is just a few pixels, do nothing
      if (points_curr->size() < 50)
        continue;
      cv::putText( illus, "component too little",
                   cv::Point (10, frame_buffer.rows - 50),
                   cv::FONT_HERSHEY_DUPLEX, 1, cvScalarAll(255) );
      writer_components.write(illus);
#endif
      continue;
    }

    //// window two big => pass
    if (points_curr->size() > .25f * frame_thres.cols * frame_thres.rows) {
#ifdef PZ_FIND_VIDEOS
      cv::putText( illus, "component too big",
                   cv::Point (10, frame_buffer.rows - 50),
                   cv::FONT_HERSHEY_DUPLEX, 1, cvScalarAll(255) );
      for (int var = 0; var < 10; ++var)
        writer_components.write(illus);
#endif
      continue;
    }

    //// shapes too far of a square -> zap
#ifndef USE_CONTOURS
    double ratio = (bbox_curr->width)
        / (bbox_curr->height); // width / height
    if (ratio > 3 || ratio < 1 / 3.f) {
#ifdef PZ_FIND_VIDEOS
      cv::putText( illus, "shape too far from a square",
                   cv::Point (10, frame_buffer.rows - 50),
                   cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(255, 255, 255) );
      for (int var = 0; var < 10; ++var)
        writer_components.write(illus);
#endif
      continue;
    }
#endif

    /* resize the current component pts to the size of the model
           keeping the ratio */
    current_component_resized.clear();
#ifdef USE_CONTOURS
    vision_utils::redimContent_vector_without_repetition<Component>
        (*points_curr, current_component_resized,
         model_size, true);
#else
    vision_utils::redimContent_vector_without_repetition_given_src_bbox(
          *points_curr,
          current_component_resized,
          *bbox_curr,
          model_size,
          resized_array.data,
          false);
#endif // USE_CONTOURS

    /* compute the distance witht the model and keep if is the best */
    for (unsigned int model_idx = 0; model_idx < model_points.size(); ++model_idx) {

      double dist = vision_utils::D22_with_min<cv::Point, Component>
          (model_points[model_idx],
           current_component_resized,
           best_component_mark
           //INFINITY
           );
      if (dist < best_component_mark) {
        best_component_mark = dist;
        best_component_index = i;
      }

#ifdef USE_CONTOURS
      DEBUG_PRINT(
            "cpt #%i: size: %i, redimSize: %i, dist with %i=%g",
            i,
            (int) points_curr->size(), //
            (int) current_component_resized.size(),
            model_idx, dist
            );
#else // USE_CONTOURS
      DEBUG_PRINT(
            "cpt#%i: win:(%i, %i) -> (%i, %i), size: %i, redimSize: %i, dist with %i=%g\n",
            i,
            bbox_curr->x, bbox_curr->y,
            bbox_curr->x + bbox_curr->width - 1,
            bbox_curr->y + bbox_curr->height - 1,
            (int) points_curr->size(), //
            (int) current_component_resized.size(),
            model_idx, dist
            );
#endif // USE_CONTOURS

#ifdef PZ_FIND_VIDEOS
      /* write the note */
      std::ostringstream txt;
      txt << "mark : " << dist;
      cv::putText( illus, txt.str().c_str(),
                   cv::Point (10, frame_buffer.rows - 50),
                   cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(255, 255, 255) );
      for (int var = 0; var < 10; ++var)
        writer_components.write(illus);
#endif

    } // end loop model_idx

  } // end of loop on components

  /* have we found ? */
  if (best_component_index == -1) {
    printf("PlayzoneFind:best_component_index == -1, return\n");
    _current_status = FAILURE_NO_GOOD_COMP;
    return;
  }

  /* save the data of our best component in global variables */
  best_component_points = &components_pts.at(best_component_index);
#ifdef USE_CONTOURS
  best_component_bbox = vision_utils::boundingBox_vec
      <cv::Point2i, cv::Rect>
      (*best_component_points);
#else
  best_component_bbox = boundingBoxes[best_component_index];
#endif // USE_CONTOURS

  /* find the center */
  // -> first method => taking the barycenter.
  // problem : it will "pull" the center to the bottom of
  // the component, as the border is thicker
  //  best_component_center = vision_utils::barycenter( best_component_points );

  // -> second method : take the center of the bbox
  //    int xMin = best_component_bbox.x;
  //    int yMin = best_component_bbox.y;
  //    int xMax = xMin + best_component_bbox.width - 1;
  //    int yMax = yMin + best_component_bbox.height - 1;
  //    best_component_center.x = (xMin + xMax) / 2;
  //    best_component_center.y = (yMin + yMax) / 2;
  best_component_center.x = best_component_bbox.x + best_component_bbox.width / 2;
  best_component_center.y = best_component_bbox.y + best_component_bbox.height / 2;

  /* display for the paper */
#ifdef PZ_FIND_IMAGES
  cv::Mat3b illus (frame_buffer.size());
  illus.setTo(0);
  //vision_utils::drawRectangle(illus, *bb, CV_RGB(255,0,0));
  vision_utils::drawListOfPoints(illus, *best_component_points,
                                cv::Vec3b(255, 255, 255));
  cv::imwrite("026-illus.png", illus);
#endif
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::reordonnate_corners() {
  // compute center
  double xC = (board_corners[0].x + board_corners[1].x + board_corners[2].x
      + board_corners[3].x) / 4.f;
  double yC = (board_corners[0].y + board_corners[1].y + board_corners[2].y
      + board_corners[3].y) / 4.f;
  // compute angles
  double angles[4];
  for (int i = 0; i < 4; ++i) {
    angles[i] = atan2(board_corners[i].y - yC, board_corners[i].x - xC);
    if (angles[i] < 0)
      angles[i] += 2.f * CV_PI;
  }
  // find the order of the corners according to their angle with the center
  double angle_threshold = -1;
  int index_ordonnedCorner[4];
  for (int rank = 0; rank < 4; ++rank) {
    int toInsert = -1;
    double bestMinAngle = 10;
    for (int scannedCorner = 0; scannedCorner < 4; ++scannedCorner) {
      if (angles[scannedCorner] > angle_threshold
          && angles[scannedCorner] < bestMinAngle) {
        toInsert = scannedCorner;
        bestMinAngle = angles[scannedCorner];
      }
    }
    index_ordonnedCorner[rank] = toInsert;
    angle_threshold = bestMinAngle;
  }
  //cout << "angle:" << angles[0] << "-" << angles[1] << "-" << angles[2]
  //      << "-" << angles[3] << endl;
  //cout << "indices:" << index_ordonnedCorner[0] << "-"
  //      << index_ordonnedCorner[1] << "-" << index_ordonnedCorner[2] << "-"
  //      << index_ordonnedCorner[3] << endl;

  // update our corners
  cv::Point board_cornersCopy[4];
  for (int i = 0; i < 4; ++i)
    board_cornersCopy[i] = board_corners[index_ordonnedCorner[i]];
  for (int i = 0; i < 4; ++i)
    board_corners[i] = board_cornersCopy[i];
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::calc_warp_matrix() {
  DEBUG_PRINT("PlayzoneFind:calc_warp_matrix()\n");

  if (board_corners.size() != 4) { /* invalid data => restore the image */
    DEBUG_PRINT("PlayzoneFind:board_corners contains %i corners instead of 4 - "
           "taking default values !\n", (int) board_corners.size());
    board_corners.clear();
    int xC = (frame_buffer.cols - BOARD_OUT_WIDTH) / 2;
    int yC = (frame_buffer.rows - BOARD_OUT_HEIGHT) / 2;
    board_corners.push_back(cv::Point(xC + BOARD_OUT_WIDTH, yC));
    board_corners.push_back(cv::Point(xC, yC));
    board_corners.push_back(cv::Point(xC, yC + BOARD_OUT_HEIGHT));
    board_corners.push_back(cv::Point(xC + BOARD_OUT_WIDTH, yC + BOARD_OUT_HEIGHT));
  } else
    reordonnate_corners();

  /* calculate the matrix */
  cv::Point2f srcQuad[4], dstQuad[4];
  // source = the corners of the board that we found before
  // copy it in some cv::Point2f
  for (int i = 0; i < 4; ++i)
    srcQuad[i] = cv::Point2f(board_corners[i].x, board_corners[i].y);
  // destination = normalized square
  dstQuad[0] = cv::Point2f(0, 0);
  dstQuad[1] = cv::Point2f(BOARD_OUT_WIDTH, 0);
  dstQuad[2] = cv::Point2f(BOARD_OUT_WIDTH, BOARD_OUT_HEIGHT);
  dstQuad[3] = cv::Point2f(0, BOARD_OUT_HEIGHT);

  // calc the mat
  warp_matrix =  cv::getPerspectiveTransform(srcQuad, dstQuad);

  /* display input M30_border_Points */
  for (int i = 0; i < 4; ++i)
    DEBUG_PRINT("PlayzoneFind:#%i:(%f, %f)->(%f, %f) | \n", i, srcQuad[i].x, srcQuad[i].y, dstQuad[i].x , dstQuad[i].y);

  /* display mat */
  //      m.str("");
  //      m << "warp_matrix:";
  //      for (int row=0; row< warp_matrix->rows; ++row) {
  //          const float* ptr = (const float*)(warp_matrix->data.ptr + row
  //                  * warp_matrix->step);
  //          for (int col=0; col < warp_matrix->cols; ++col)
  //              m << *ptr++ << " ";
  //      }
  //      DEBUG_PRINT(m.str() );
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::rectif_image_init() {
  DEBUG_PRINT("PlayzoneFind:rectif_image_init()\n");

  // rows, cols
  rectifiedImage.create(BOARD_OUT_HEIGHT, BOARD_OUT_WIDTH);
  //frame_16bits.create(1, 1);
  //rectifiedImage_nonResized_16bits.create(1, 1);
  //rectifiedImage_nonResized.create(1, 1);
  //    rectifiedImage_monochrome.create(1, 1);
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::rectif_image() {
  DEBUG_PRINT("PlayzoneFind:rectif_image()\n");

  // no conversion seems needed
  cv::warpPerspective(frame_buffer, rectifiedImage,
                      warp_matrix, rectifiedImage.size());

  DEBUG_PRINT("PlayzoneFind:rectifiedImage:'%s'\n",
         vision_utils::infosImage(rectifiedImage).c_str());
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::removeBorder_init() {
  inverse_warp_matrix.create(1, 1);
  //rectifiedImage_BW.create(1, 1);
}

////////////////////////////////////////////////////////////////////////////////

void PlayzoneFind::removeBorder2() {
  DEBUG_PRINT("PlayzoneFind:removeBorder2()\n");
  cv::Point* A = &board_corners.at(2);
  cv::Point* B = &board_corners.at(3);
  cv::Point* C = &board_corners.at(0);
  cv::Point* D = &board_corners.at(1);
  cv::Point2f curr, curr2;

  /* compute the increments in x and y */
  float step_ty = 1.0 / std::max(hypot(D->y - A->y, D->x - A->x), // breakline
                                 hypot(C->y - B->y, C->x - B->x));
  float step_tx = 1.0 / std::max(hypot(B->y - A->y, B->x - A->x), // breakline
                                 hypot(D->y - C->y, D->x - C->x));
  //DEBUG_PRINT("PlayzoneFind:incr\n", step_ty);

  std::vector<float> left, right, up, down;

  for (float ty = 0; ty < 1; ty += step_ty) {
    float tx = 0;
    /* left */
    do {
      tx += step_tx;
      vision_utils::barycenter4<cv::Point> (tx, ty, *A, *B, *C, *D, curr);
    } while (frame_thres(curr.y, curr.x) != 0);
    left.push_back(tx);
    //cout << "Adding tx:" << tx << endl;
    /* right */
    tx = 1.0;
    do {
      tx -= step_tx;
      vision_utils::barycenter4<cv::Point> (tx, ty, *A, *B, *C, *D, curr2);
    } while (frame_thres(curr2.y, curr2.x) != 0);
    right.push_back(tx);
    //cout << "Adding tx:" << tx << endl;

  } // end for ty

  for (float tx = 0; tx < 1; tx += step_tx) {
    float ty = 0;
    /* up */
    do {
      ty += step_ty;
      vision_utils::barycenter4<cv::Point> (tx, ty, *A, *B, *C, *D, curr);
    } while (frame_thres(curr.y, curr.x) != 0);
    up.push_back(ty);
    //cout << "Adding tx:" << tx << endl;
    /* down */
    ty = 1.0;
    do {
      ty -= step_ty;
      vision_utils::barycenter4<cv::Point> (tx, ty, *A, *B, *C, *D, curr2);
    } while (frame_thres(curr2.y, curr2.x) != 0);
    down.push_back(ty);
    //cout << "Adding tx:" << tx << endl;

  } // end for ty

#ifdef PZ_FIND_IMAGES
  DEBUG_PRINT("PlayzoneFind:Display !\n");
  cv::Mat3b remove_border2_illus (frame_buffer.size());
  remove_border2_illus.setTo(0);
  cv::cvtColor(frameBW, remove_border2_illus, CV_GRAY2RGB);

  /* rainbow flag */
  for (float ty = 0; ty < 1; ty+= step_ty) {
    cv::Vec3b color = vision_utils::color(100 * ty, 100 );
    //        color[0] *= 2;
    //        color[1] *= 2;
    //        color[2] *= 2;
    for (float tx = 0; tx < 1; tx+= step_tx) {
      //color = color / 2;
      vision_utils::barycenter4(tx, ty, *A, *B, *C, *D, curr);
      remove_border2_illus(curr.y, curr.x) = color;
    } // end loop tx
  } // end loop ty
#endif // PZ_FIND_IMAGES


  /* take the medians */
  sort(left.begin(), left.end());
  sort(right.begin(), right.end());
  sort(up.begin(), up.end());
  sort(down.begin(), down.end());
  float l = left.at(left.size() / 2), r = right.at(right.size() / 2);
  float u = up.at(up.size() / 2), d = down.at(down.size() / 2);
  DEBUG_PRINT("PlayzoneFind:l:%f - r:%f | u:%f - d:%f\n", l, r, u, d);

  /* compute the "average" corners */
  cv::Point2f new_corners[4];
  vision_utils::barycenter4<cv::Point> (r, d, *A, *B, *C, *D, new_corners[0]);
  vision_utils::barycenter4<cv::Point> (l, d, *A, *B, *C, *D, new_corners[1]);
  vision_utils::barycenter4<cv::Point> (l, u, *A, *B, *C, *D, new_corners[2]);
  vision_utils::barycenter4<cv::Point> (r, u, *A, *B, *C, *D, new_corners[3]);

  /* save the new results */
  for (int i = 0; i < 4; ++i)
    board_corners[i] = new_corners[i];


#ifdef PZ_FIND_IMAGES
  // make the image brighter
  cv::add(remove_border2_illus, cv::Scalar::all(120), remove_border2_illus);

  for (int i = 0; i < 4; ++i) {
    cv::line(remove_border2_illus, new_corners[i], new_corners[(i+1)%4],
        //vision_utils::color_scalar<cv::Scalar>(i, 4),
        CV_RGB(100, 0, 0),
        2);
  }

  cv::imshow( "remove_border2_illus", remove_border2_illus );
  cv::waitKey(0);

  cv::imwrite("050-remove_border2_illus.png", remove_border2_illus);
#endif // PZ_FIND_IMAGES
}
