#include "games_vision/playzone_find_benchmark.h"
#include "games_vision/playzone_find_skill.h"
#include <ros/ros.h>

int main() {
  ROS_INFO("main()");
  PlayzoneFindBenchmark bench(PLAYZONE_DIR, "pz.xml");
  bench.start();
  return 0;
}

