add_executable(launcher_driving_wheel.exe launcher_driving_wheel.cpp )
target_link_libraries(launcher_driving_wheel.exe ${catkin_LIBRARIES})

add_executable(launcher_driving_game.exe launcher_driving_game.cpp )
target_link_libraries(launcher_driving_game.exe ${catkin_LIBRARIES})

add_executable(launcher_driving_receiver_base.exe
  launcher_driving_receiver_base.cpp )
target_link_libraries(launcher_driving_receiver_base.exe ${catkin_LIBRARIES})
