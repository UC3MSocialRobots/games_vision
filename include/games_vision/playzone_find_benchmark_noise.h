#ifndef PLAYZONE_FIND_BENCHMARK_NOISE_H
#define PLAYZONE_FIND_BENCHMARK_NOISE_H

#include "games_vision/playzone_find_benchmark.h"

#define NOISE_BENCHMARK
//#define LIGHT_BENCHMARK

/*! \class  PlayzoneFindBenchmarkNoise
 *
 */
class PlayzoneFindBenchmarkNoise : public PlayzoneFindBenchmark{
public:
    /*! create an annotator from loading an xml file
      \param path_to_xml_file it should finish with /
      Example IMG_DIR "pz/"
      \param xml_filename Example foo.xml
      */
    PlayzoneFindBenchmarkNoise(const std::string & path_to_xml_file,
                               const std::string & xml_filename);

    virtual void preprocessing_function(cv::Mat3b & input_image);

    //! make the processing
    void start();

private:
    //! the noise factor : 0 = no noise, 1 = random picture
    double _factor;
#ifdef NOISE_BENCHMARK
    static const double _factor_step = .02;
#endif // NOISE_BENCHMARK

#ifdef LIGHT_BENCHMARK
    static const double _factor_step = .02;
#endif // LIGHT_BENCHMARK
};

#endif // PLAYZONE_FIND_BENCHMARK_NOISE_H
