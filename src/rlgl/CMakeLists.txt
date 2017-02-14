add_executable(red_light_green_light_skill.exe
  red_light_green_light_skill.cpp

  )
target_link_libraries(red_light_green_light_skill.exe ${catkin_LIBRARIES}
  )
add_dependencies(red_light_green_light_skill.exe
                 ${people_msgs_EXPORTED_TARGETS})
