#include "games_vision/playzone_annotation.h"

/** test */
void test_annotation() {
    PlayzoneAnnotation annotation;

    annotation.clear_corners();
    for (int corner_idx = 0; corner_idx < 4; ++corner_idx)
        annotation.add_corner(PlayzoneAnnotation::Corner
                              (corner_idx, corner_idx * corner_idx));

    XmlDocument doc;
    XmlDocument::Node* node = doc.add_node(doc.root(), "annot", "");

    annotation.to_xml_node_custom(doc, node);
    ROS_WARN("doc:\n'%s'", doc.to_string().c_str());
    //doc.write_to_file("output_test_annotator_playzone.xml");
}

int main() {
    ROS_INFO("main()");
    test_annotation();

    return 0;
}

