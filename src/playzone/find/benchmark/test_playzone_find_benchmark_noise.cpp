#include "games_vision/playzone_find_benchmark_noise.h"
#include <vision_utils/img_path.h>


/** test */
int main() {
    ROS_INFO("main()");

    PlayzoneFindBenchmarkNoise bench(IMG_DIR "pz/",
                                     //"pz_1.xml"
                                     "pz.xml"
                                     );
    bench.start();

    return 0;
}

