add_library(corner_finder corner_finder.cpp)

add_library(playzone_find playzone_find.cpp)
target_link_libraries(playzone_find ${catkin_LIBRARIES}
  corner_finder)

add_library(playzone_find_skill playzone_find_skill.cpp)
target_link_libraries(playzone_find_skill ${catkin_LIBRARIES}
  playzone_find)

add_executable(launcher_playzone_find_skill.exe launcher_playzone_find_skill.cpp)
target_link_libraries(launcher_playzone_find_skill.exe ${catkin_LIBRARIES}
  playzone_find_skill)

add_subdirectory(benchmark)
