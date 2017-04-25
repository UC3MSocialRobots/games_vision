#ifndef FIND_PLAY_ZONE_SKILL_H
#define FIND_PLAY_ZONE_SKILL_H

/***************************************************************************//**
 * \class PlayzoneFindSkill
 *
 * \brief A skill to find the playzones ( thick black rectangles ), used in visual games
 *
 * It uses connected components.
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date 25/05/2009
 *
 *
 *******************************************************************************/

//! save the images in png files, to debug
//#define SAVE_IMAGES

//! draw the images for the article
//#define PZ_FIND_IMAGES
//! save the videos for the article
//#define PZ_FIND_VIDEOS
//! test all the methods
//#define ARTICLE_METHODS_COMP

///// Magguie imports
#include "vision_utils/timer.h"
#include "games_vision/corner_finder.h"
#include "vision_utils/disjoint_sets2.h"

#define GAMES_DATA_DIR   ros::package::getPath("games_vision") + "/data/"
#define PLAYZONE_DIR     GAMES_DATA_DIR + "playzone/"

#ifdef PZ_FIND_VIDEOS
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
int codec = //-1,
            CV_FOURCC('M','J','P','G'); //= motion-jpeg codec (does not work well)
//CV_FOURCC('D', 'I', 'V', 'X'); //= MPEG-4 codec
#endif // PZ_FIND_VIDEOS

class PlayzoneFind {
public:
  typedef CornerFinder::Component Component;

  //! the possible different status
  enum Status {
    NEVER_RUN,
    SEARCHING,
    SUCCESS_FOUND,
    FAILURE_NO_GOOD_COMP,
    FAILURE_NO_CORNERS_FOUND
  };

  /*! \class Times
        store all the times in a class for becnhmarking */
  class Times {
  public:
    double t01_after_thresh_image;
    double t02_after_numeroteComponents;
    double t03_after_compareComponents;
    double t04_after_getCorners;
    double t05_after_saving_images;
    double t06_after_removeBorder2;
    double t07_after_calc_warp_matrix;
    double t08_after_rectif_image;
    double t_after_everything;

    Times();
    void clear_times();

    std::string to_string() const;
  };


  //! the maximum distance between the best comp and the model
  const static double ASSOC_MAX_DIST =
    #ifdef USE_CONTOURS
      3;
#else
      2;
#endif
  //! the min width of the component representing the playzone (pixels)
  const static int ASSOC_MIN_WIDTH = 30;
  //! the min height of the component representing the playzone (pixels)
  const static int ASSOC_MIN_HEIGHT = 30;
  /*! the threshold in lightness for the detection of the black
        connected components. Higher -> more restrictive ? */
  const static int LIGHTNESS_THRES = 100; // 130
  /*! the threshold in value for the detection of the black
        connected components. Higher -> more restrictive ? */
  const static int VALUE_THRES = 100; // 140
  /*! the threshold value  is the weighted sum
    (i.e. cross-correlation with a Gaussian window) of
    a neighborhood of the point, minus C */
  const static int ADAPTATIVE_THRES_SIZE = 91;
  const static double ADAPTATIVE_THRES_C = -2.f;


  /** constructor */
  PlayzoneFind(const int BOARD_OUT_WIDTH_,
               const int BOARD_OUT_HEIGHT_);
  /** destructor */
  virtual ~PlayzoneFind();

  /////
  ///// general functions
  /////
  //! change the current image
  void set_input(const cv::Mat3b* new_frame);

  /*!
     * \brief   a routine to find the playzone
     * \arg stop_before_components true to stop the algorithm
            before counting connected components
     * \return true in case of success
     */
  virtual bool find_playzone();

  void display() const;

  //! \return true if the playzone was successfully found
  bool was_playzone_found() const ;

  /** return a copy the image of the playzone */
  const cv::Mat3b* get_playzone() const;

  //! \param ans where to put the corners we have found in the picture
  void get_corner_list(std::vector<cv::Point2i> & ans) const;

  //! \return the current status
  Status get_current_status() const ;

  //! \return the current status as a string
  std::string get_current_status_as_string() const ;

  inline int get_corner_successful_method_number() const {
    return corner_finder.successful_method_number;
  }

  //! \return the time needed for the current processing
  Times get_current_times() const ;

protected:
  void find_playzone_part1();
  bool find_playzone_part2(bool rectif_playzone = true);
  /*! init the detector of the board */
  void find_playzone_init();
  void thresh_image();
  //! \brief   find all the connected components
  void numeroteComponents();
  //! \brief   compare all the connected components to find the closest to a square
  void compareComponents();
  //! \brief   init the images containing the rectified image
  void rectif_image_init();
  //! \brief   rectify the image with the matrix we calculated before
  void rectif_image();
  /*! \brief   once you have calculated the corners of the zone,
        calc the matrix to distort the image */
  void calc_warp_matrix();
  /*! \brief   reordonate the corners of the board,
        according to their angle with the center */
  void reordonnate_corners();
  //! \brief   init the variables used in the removeBorder() method
  void removeBorder_init();
  /*! another method for removing the border,
      with only one warpPerspective */
  void removeBorder2();

  vision_utils::Timer global_timer;
  vision_utils::Timer step_timer;
  Status _current_status;

  //! the time needed for the current processing
  Times _current_times;

  //! a pointer to the processed image
  const cv::Mat3b* _frame;

  //! the buffer version of the acquired frame
  cv::Mat3b frame_buffer;
  //! the BW version of the acquired frame
  cv::Mat1b frameBW;
#ifdef PZ_FIND_IMAGES
  //! the output
  cv::Mat3b frameOut;
#endif // PZ_FIND_IMAGES

  cv::Mat1b frame_thres;

  int nbComponents;
  vision_utils::DisjointSets2 set;
  std::vector< Component > components_pts;
#ifndef USE_CONTOURS
  std::vector<cv::Rect> boundingBoxes;
#endif // USE_CONTOURS
  cv::Mat1b resized_array;

  ///// compare the components
  cv::Rect model_size;
  std::vector<Component> model_points;
  Component current_component_resized;
  float best_component_mark;

  int best_component_index;
  Component* best_component_points;
  cv::Rect best_component_bbox;
  Component board_corners;
  cv::Point best_component_center;

  ///// finding the corners
  CornerFinder corner_finder;

  ///// image rectification
  cv::Mat3b rectifiedImage;
  //! the desired width of the board, in pixels
  int BOARD_OUT_WIDTH;
  //! the desired height of the board, in pixels
  int BOARD_OUT_HEIGHT;
  cv::Mat warp_matrix;
  cv::Mat1f inverse_warp_matrix;
};

#endif

