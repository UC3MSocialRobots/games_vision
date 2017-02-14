#include "games_vision/playzone_find_benchmark.h"
#include <vision_utils/img_path.h>


/** tests */
int main() {
    ROS_INFO("main()");

    PlayzoneFindBenchmark bench(IMG_DIR "pz/", "pz.xml");
    bench.start();

    return 0;
}
