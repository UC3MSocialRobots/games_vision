add_library(playzone_track playzone_track.cpp)
target_link_libraries(playzone_track ${catkin_LIBRARIES}
  playzone_find)

add_executable(test_playzone_track.exe test_playzone_track.cpp)
target_link_libraries(test_playzone_track.exe ${catkin_LIBRARIES}
  playzone_track)

add_library(playzone_track_skill playzone_track_skill.cpp)
target_link_libraries(playzone_track_skill ${catkin_LIBRARIES}
  playzone_track)

add_executable(test_playzone_track_skill.exe test_playzone_track_skill.cpp)
target_link_libraries(test_playzone_track_skill.exe ${catkin_LIBRARIES}
  playzone_track_skill)
