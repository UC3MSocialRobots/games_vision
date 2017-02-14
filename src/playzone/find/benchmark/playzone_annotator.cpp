#include "games_vision/playzone_annotator.h"

///////////////////////////////////////////////////////////////////////

void PlayzoneAnnotator::refresh_window_custom() {
    // draw a circle at each corner
    for (PlayzoneAnnotation::CornerList::const_iterator pt_it = corners.begin();
         pt_it != corners.end(); ++pt_it)
        cv::circle(_current_image_in_window, *pt_it, 5, CV_RGB(255, 0, 0), 2);
}

