#include "games_vision/playzone_find_benchmark_noise.h"

////////////////////////////////////////////////////////////////////////////////

PlayzoneFindBenchmarkNoise::PlayzoneFindBenchmarkNoise(const std::string & path_to_xml_file,
                                                       const std::string & xml_filename) :
    PlayzoneFindBenchmark(path_to_xml_file, xml_filename) {
    ROS_INFO("ctor");
}

////////////////////////////////////////////////////////////////////////////////

inline void PlayzoneFindBenchmarkNoise::preprocessing_function(cv::Mat3b & input_image) {
    //input_image = cv::Vec3b(0, 0, 0);

#ifdef NOISE_BENCHMARK
    // another Mat constructor; allocates a matrix of the specified
    // size and type
    cv::Mat3b noise(input_image.size());

    // fills the matrix with normally distributed random values (randn());
    // there is also randu() for uniformly distributed random numbers.
    // Scalar replaces CvScalar, Scalar::all() replaces cvScalarAll().
    //cv::randn(noise, cv::Scalar::all(128), cv::Scalar::all(128));
    cv::randu(noise, cv::Scalar::all(0), cv::Scalar::all(255));

    // blur the noise a bit, kernel size is 3x3 and both sigma's
    // are set to 0.5
    //cv::GaussianBlur(noise, noise, Size(3, 3), 0.5, 0.5);

    // void addWeighted(const Mat& src1, double alpha, const Mat& src2, double beta, double gamma, Mat& dst)
    // The functions addWeighted calculate the weighted sum of two arrays as follows:
    // dst = saturate(src1 * alpha + src2 * beta + gamma)
    cv::addWeighted(input_image, 1 - _factor,
                    noise, _factor, 0,
                    input_image);
#endif // NOISE_BENCHMARK

#ifdef LIGHT_BENCHMARK
    // The parameters alpha and beta are often called the gain and bias parameters;
    // sometimes these parameters are said to control contrast and brightness respectively.
    double alpha = 1;
    double beta = -255 * _factor;

    // Do the operation new_image(i,j) = alpha*image(i,j) + beta
    for( int y = 0; y < input_image.rows; y++ ) {
        for( int x = 0; x < input_image.cols; x++ ) {
            for( int c = 0; c < 3; c++ ) {
                input_image.at<cv::Vec3b>(y,x)[c] =
                        cv::saturate_cast<uchar>
                        ( alpha*( input_image.at<cv::Vec3b>(y,x)[c] ) + beta );
            } // end loop c
        } // end loop x
    } // end loop y
#endif // LIGHT_BENCHMARK

    //cv::imshow("input_image", input_image); cv::waitKey(0);
}


////////////////////////////////////////////////////////////////////////////////

void PlayzoneFindBenchmarkNoise::start() {
    ROS_INFO("start()");

    // set the noise factor
    for (_factor = 0;
         _factor <= 1  + _factor_step;
         _factor += _factor_step) {
        // start the normal analysis
        PlayzoneFindBenchmark::start();

        ROS_WARN("_factor, success, average_time:  %f\t%f\t%g\t%s",
                    _factor,
                    100.f * _nb_files_successful / _nb_files_processed,
                    _global_times.t_after_everything,
                    _global_times.to_string().c_str()
                    );
    } // end loop _noise_factor
}
