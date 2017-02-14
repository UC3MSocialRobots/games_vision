#ifndef PLAYZONE_ANNOTATION_H
#define PLAYZONE_ANNOTATION_H

#include "vision_utils/XmlDocument.h"
// opencv
#include <opencv2/core/core.hpp>

/*! \class  PlayzoneAnnotation
 *  a class containing the data
 */
class PlayzoneAnnotation {
public:
    typedef cv::Point2i Corner;
    typedef std::vector<Corner> CornerList;

    /*! erase the list of corners */
    void clear_corners();

    /*! change the annotated corners */
    void add_corner(const Corner & new_corner);

    /*! return the error of estimation
        between the manually specified position and
        an algorithm estimated position */
    double distance_to_answer(const CornerList & computed_corners)
    const;

    //! \see XmlImagesReader::from_xml_node_custom()
    void from_xml_node_custom(const XmlDocument & doc, XmlDocument::Node* node);

    //! \see XmlImagesReader::to_xml_node_custom()
    void to_xml_node_custom(XmlDocument & doc, XmlDocument::Node* node) const;


protected:
    CornerList corners;
}; // end class PlayzoneAnnotation

#endif // PLAYZONE_ANNOTATION_H
