add_executable(costmap_stage_editor.exe
    costmap_stage_editor.cpp)
target_link_libraries(costmap_stage_editor.exe
    ${catkin_LIBRARIES})
add_dependencies(costmap_stage_editor.exe
    ${people_msgs_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})

add_executable(costmap_watcher.exe
    costmap_watcher.cpp)
target_link_libraries(costmap_watcher.exe
    ${catkin_LIBRARIES})
add_dependencies(costmap_watcher.exe
    games_vision_generate_messages_cpp
    ${people_msgs_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})

add_executable(motion_video_recorder.exe motion_video_recorder.cpp)
target_link_libraries(motion_video_recorder.exe ${catkin_LIBRARIES})
add_dependencies(motion_video_recorder.exe
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})

add_executable(movement_listener.exe movement_listener.cpp)
target_link_libraries(movement_listener.exe ${catkin_LIBRARIES})
add_dependencies(movement_listener.exe
    games_vision_generate_messages_cpp
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})

