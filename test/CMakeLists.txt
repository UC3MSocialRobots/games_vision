catkin_add_gtest(     gtest_playzone_find gtest_playzone_find.cpp)
target_link_libraries(gtest_playzone_find ${catkin_LIBRARIES}
                      playzone_find)

catkin_add_gtest(     gtest_hangman  gtest_hangman.cpp)
target_link_libraries(gtest_hangman  ${catkin_LIBRARIES} CHangMan)

catkin_add_gtest(     gtest_playzone_sequential_user
                      gtest_playzone_sequential_user.cpp)
target_link_libraries(gtest_playzone_sequential_user ${catkin_LIBRARIES}
                      playzone_find_skill)
add_dependencies(     gtest_playzone_sequential_user
                      ${games_vision_EXPORTED_TARGETS})

catkin_add_gtest(     gtest_tic_tac_toe_skill gtest_tic_tac_toe_skill.cpp)
target_link_libraries(gtest_tic_tac_toe_skill ${catkin_LIBRARIES}
                      tic_tac_toe_skill playzone_find_skill)

catkin_add_gtest(     gtest_red_light_green_light gtest_red_light_green_light.cpp)
target_link_libraries(gtest_red_light_green_light ${catkin_LIBRARIES})
