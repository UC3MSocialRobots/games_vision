#ifndef PLAYZONE_TRACK_H
#define PLAYZONE_TRACK_H

#include "games_vision/playzone_find.h"
#include <opencv2/highgui/highgui.hpp>

//#define PZ_TRACK_VIDEOS

/*! \class  PlayzoneTrack
 *
 */
class PlayzoneTrack : public PlayzoneFind {
public:
    enum Mode { MODE_ONLY_TRACK,
                MODE_REPROJECT_IMAGE,
                MODE_REPROJECT_VIDEO};

    static const double SIZE_VARIATION_MIN_RATIO = 0.3;
    static const double SIZE_VARIATION_MAX_RATIO = 2.0;
    static const double ANGLE_VARIATION_MAX_DIST = 1.5;

    /*! constructor */
    PlayzoneTrack(int BOARD_OUT_WIDTH_,
                  int BOARD_OUT_HEIGHT_);
    ~PlayzoneTrack();

    void set_mode(const Mode mode,
                  const std::string & file_with_mode);

    //! change the current image
    //void set_input(cv::Mat3b* new_frame);

    bool find_playzone();

    //! true if the playzone was find in a succesful way
    bool is_tracking_OK;

    //! the final image, containing frame and the rectified model
    cv::Mat3b frame_with_model;

    /////
    ///// parameters
    /////
    Mode reproject_mode;
    std::string filename_with_reproject_mode;

protected:
    /////
    ///// the playzone finder
    /////
    void find_from_playzone();
    void find_with_tracking();
    cv::Point component_center;
    cv::Rect component_bbox;
    bool* region_growth_seen_points;
    //int xMin, xMax, yMin, yMax;


    //////
    ////// the reprojection
    //////
    void reproject_init();
    void reproject();
    cv::Mat model_warp_matrix;
    std::vector<cv::Point2f> source_pts;
    std::vector<cv::Point2f> dest_pts;

    // things from the model image
    cv::VideoCapture model_capture;
    cv::Mat3b model_image;

#ifdef PZ_TRACK_VIDEOS
    cv::VideoWriter writer_frame;
    cv::VideoWriter writer_frameOut;
    cv::VideoWriter writer_playzone_mask;
    cv::VideoWriter writer_frame_with_model;
#endif

};

#endif // PLAYZONE_TRACK_H
