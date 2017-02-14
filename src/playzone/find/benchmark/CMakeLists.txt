add_library(playzone_annotation playzone_annotation.cpp)
target_link_libraries(playzone_annotation ${catkin_LIBRARIES}
  )

add_library(playzone_annotator playzone_annotator.cpp)
target_link_libraries(playzone_annotator ${catkin_LIBRARIES}
  playzone_find playzone_annotation)

add_library(playzone_find_benchmark playzone_find_benchmark.cpp)
target_link_libraries(playzone_find_benchmark ${catkin_LIBRARIES}
  playzone_find playzone_annotation)

add_library(playzone_find_benchmark_noise playzone_find_benchmark_noise.cpp)
target_link_libraries(playzone_find_benchmark_noise ${catkin_LIBRARIES}
  playzone_find_benchmark)

add_executable(test_playzone_annotation.exe test_playzone_annotation.cpp)
target_link_libraries(test_playzone_annotation.exe ${catkin_LIBRARIES}
  playzone_annotation)

add_executable(test_playzone_annotator.exe test_playzone_annotator.cpp)
target_link_libraries(test_playzone_annotator.exe ${catkin_LIBRARIES}
  playzone_annotator)

add_executable(test_playzone_find_benchmark.exe test_playzone_find_benchmark.cpp)
target_link_libraries(test_playzone_find_benchmark.exe ${catkin_LIBRARIES}
  playzone_find_benchmark)

add_executable(test_playzone_find_benchmark_noise.exe test_playzone_find_benchmark_noise.cpp)
target_link_libraries(test_playzone_find_benchmark_noise.exe ${catkin_LIBRARIES}
  playzone_find_benchmark_noise)
