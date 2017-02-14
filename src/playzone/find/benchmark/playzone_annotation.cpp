#include "games_vision/playzone_annotation.h"
#include "vision_utils/permutation.h"
#include "vision_utils/find_closest_points_brute_force.h"
#include <ros/ros.h>

///////////////////////////////////////////////////////////////////////

void PlayzoneAnnotation::clear_corners() {
    ROS_INFO("from_xml_node()");
    corners.clear();
}

///////////////////////////////////////////////////////////////////////

void PlayzoneAnnotation::add_corner(const Corner & new_corner) {
    ROS_INFO("add_corner()");
    corners.push_back(new_corner);
}

///////////////////////////////////////////////////////////////////////

double PlayzoneAnnotation::distance_to_answer(const CornerList & computed_corners)
const {
    ROS_INFO("distance_to_answer()");
    vision_utils::Permutation best_permut;
    return vision_utils::find_closest_points_brute_force(
                computed_corners, corners, best_permut);
}

///////////////////////////////////////////////////////////////////////

void PlayzoneAnnotation::
from_xml_node_custom(const vision_utils::XmlDocument & doc,
                     vision_utils::XmlDocument::Node* node) {
    ROS_INFO("from_xml_node()");
    // load the corners
    std::vector<vision_utils::XmlDocument::Node*> corner_nodes;
    doc.get_all_nodes_at_direction(node, "corner", corner_nodes);
    corners.clear();
    for (std::vector<vision_utils::XmlDocument::Node*>::const_iterator corner = corner_nodes.begin();
         corner != corner_nodes.end() ; ++corner) {
        double x = doc.get_node_attribute<double>(*corner, "x", -1);
        double y = doc.get_node_attribute<double>(*corner, "y", -1);
        corners.push_back(Corner(x, y));
    } // end loop corners
}

///////////////////////////////////////////////////////////////////////

void PlayzoneAnnotation::
to_xml_node_custom(vision_utils::XmlDocument & doc, vision_utils::XmlDocument::Node* node) const {
    ROS_INFO("to_xml_node()");
    // save the corners
    for (CornerList::const_iterator corner = corners.begin();
         corner != corners.end() ; ++corner) {
        vision_utils::XmlDocument::Node* corner_node = doc.add_node(node, "corner", "");
        doc.set_node_attribute(corner_node, "x", corner->x);
        doc.set_node_attribute(corner_node, "y", corner->y);
    } // end loop corners
}
