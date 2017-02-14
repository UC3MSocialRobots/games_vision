#ifndef CORNERFINDER_H
#define CORNERFINDER_H


//! the number of slices around the center (int)
#define M10_NB_ANGULAR_SLICES       50
//! the number of neighbours necessary to be better to be declared a local max (int)
#define M1_HISTSIZE             4
//! the ration of the barycenter_of_two_points for the found corners with the center (0 < x < 1)
#define M1_BARY                 .95

//! the minimum ration between the best and the second best to declare the best as the the index to push (double, ~=1)
#define M15_DIFF_RATIO_THRES    1.05

//! the number of neighbours on the left for the computation of the average angle (int)
//! it is also the number of neighbours for the search of the local maximums
#define M17_HIST_SIZE           3
//! the minimum average angle of the point with its neighbours
//! so as not to be dismissed in the search of the local maximums (0 <= double <= PI)
#define M17_ANGLE_MIN           0.4

//! the maximum number of reverts to exit the loop (int)
#define M2_MAX_CONSEC_REVERTS   100

/// STD
#include <vector>

/// cv
#include <opencv2/core/core.hpp>

class CornerFinder {
public:
    typedef std::vector<cv::Point2i> Component;

    CornerFinder();
    ~CornerFinder();
    void setComponent(Component* c, cv::Rect & bbox, cv::Point2i & center);
    void set_M3_frame_size(cv::Size s);
    void findCorners();
    void getCorners(Component & rep);
    bool were_corners_found;
    int successful_method_number;

    void allow_all_methods();
    void forbid_all_methods();
    bool M10_allowed;
    bool M15_allowed;
    bool M17_allowed;
    bool M20_allowed;
    bool M30_allowed;

private:
    /////
    ///// data
    /////
    cv::Point2i center_of_component;
    Component* component;
    cv::Rect bounding_box;
    int xMin;
    int xMax;
    int yMin;
    int yMax;
    cv::Size M3_frame_size;

    /////
    ///// output
    /////
    Component found_corners;

    /////
    ///// specific data for each method
    /////
    void findCorners10();
    double* M10_biggestDistance_slice;
    cv::Point* M10_furthestPoint_slice;

    void findCorners15();
    double* M15_distance_diffs;

    void findCorners17();
    double* M17_average_angles;

    void findCorners20();
    double M20_mark;
    Component M20_quad;
    Component M20_quad_back_up;
    void findCorners20_eval();
    int M20_consec_reverts;

    void findCorners30();
    cv::Mat1b M30_bestComponent;
    std::vector< Component  > M30_contours;
    Component M30_border_Points;
    cv::Point3f M30_average_equations [4];

    void findCorners_init();
};

#endif // CORNERFINDER_H
