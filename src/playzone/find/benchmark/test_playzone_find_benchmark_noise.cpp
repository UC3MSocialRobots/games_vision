#include "games_vision/playzone_find_benchmark_noise.h"
#include "games_vision/playzone_find_skill.h"
#include <ros/ros.h>

int main() {
  ROS_INFO("main()");
  PlayzoneFindBenchmarkNoise bench(PLAYZONE_DIR,
                                   //"pz_1.xml"
                                   "pz.xml"
                                   );
  bench.start();
  return 0;
}

