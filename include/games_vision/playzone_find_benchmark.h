#ifndef BENCHMARK_FIND_PLAYZONE_H
#define BENCHMARK_FIND_PLAYZONE_H

#include "vision_utils/xml_images_reader.h"
#include "games_vision/playzone_annotation.h"
#include "games_vision/playzone_find.h"

class PlayzoneFindBenchmark :
        public vision_utils::XmlImagesReader,
        public PlayzoneAnnotation{
public:
    enum Status {
        PROCESSING,
        DETECTION_POSITIVE,
        DETECTION_POSITIVE_WRONG_LOCATION,
        DETECTION_FALSE_NEGATIVE,
    };

    /*! the threshold in pixels for a succesful recognition.
      If the computed corners are in average further than this distance,
      the detection is considered as failed. */
    static const double MAX_DIST_PER_CORNER_FOR_SUCCESFUL_MATCHING = 15;

    /*! create an annotator from loading an xml file
      \param path_to_xml_file it should finish with /
      Example IMG_DIR "pz/"
      \param xml_filename Example foo.xml
      */
    PlayzoneFindBenchmark(const std::string & path_to_xml_file,
                          const std::string & xml_filename);

    //! make the processing
    void start();


    /*!
     //! the function that is called before
     \param input_image the image that can be modified
    */
    virtual void preprocessing_function(cv::Mat3b & input_image);

    //! \return the current status
    Status get_current_status() const ;

    //! \return the current status as a string
    std::string get_current_status_as_string() const ;

protected:

    //! \see XmlImagesReader::from_xml_node_custom()
    void from_xml_node_custom(const vision_utils::XmlDocument & doc, vision_utils::XmlDocument::Node* node) {
        PlayzoneAnnotation::from_xml_node_custom(doc, node);
    }

    //! \see XmlImagesReader::to_xml_node_custom()
    void to_xml_node_custom(vision_utils::XmlDocument & doc, vision_utils::XmlDocument::Node* node) const {
        PlayzoneAnnotation::to_xml_node_custom(doc, node);
    }

    void process_current_file();

    static const int PLAYZONE_OUT_WIDTH = 300;
    static const int PLAYZONE_OUT_HEIGHT = 300;
    PlayzoneFind _playzone_finder;
    Status _current_status;

    PlayzoneFind::Times _global_times;
    int _nb_files_processed;
    int _nb_files_successful;
};

#endif // BENCHMARK_FIND_PLAYZONE_H
