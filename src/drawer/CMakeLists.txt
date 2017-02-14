add_library(drawer drawer.cpp)
target_link_libraries(drawer ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_drawer.exe test_drawer.cpp)
target_link_libraries(test_drawer.exe ${catkin_LIBRARIES}
  drawer)
