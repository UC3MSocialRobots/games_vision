#ifndef ANNOTATOR_PLAYZONE_H
#define ANNOTATOR_PLAYZONE_H

#include "vision_utils/xml_images_interface.h"
#include "games_vision/playzone_annotation.h"

////////////////////////////////////////////////////////////////////////////////

class PlayzoneAnnotator : public XmlImagesInterface, PlayzoneAnnotation {
public:
    //! \see XmlImagesReader::from_xml_node_custom()
    void from_xml_node_custom(const XmlDocument & doc, XmlDocument::Node* node) {
        PlayzoneAnnotation::from_xml_node_custom(doc, node);
    }

    //! \see XmlImagesReader::to_xml_node_custom()
    void to_xml_node_custom(XmlDocument & doc, XmlDocument::Node* node) const {
        PlayzoneAnnotation::to_xml_node_custom(doc, node);
    }

    //! \see XmlImagesInterface::action_at_left_click()
    virtual void action_at_left_click(int x, int y) {
        add_corner(cv::Point2i(x, y));
    }

    //! \see XmlImagesInterface::action_at_left_click()
    virtual void action_at_right_click(int, int) {
        clear_corners();
    }

    //! \see XmlImagesInterface::refresh_window_custom()
    virtual void refresh_window_custom();
private:
};

#endif // ANNOTATOR_PLAYZONE_H
