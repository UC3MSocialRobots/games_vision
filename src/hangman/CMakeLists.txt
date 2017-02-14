add_library(CHangMan CHangMan.cpp HangmanSolver.cpp)
target_link_libraries(CHangMan ${catkin_LIBRARIES}
  curl)

add_executable(test_CHangMan.exe test_CHangMan.cpp)
target_link_libraries(test_CHangMan.exe ${catkin_LIBRARIES} CHangMan)

add_executable(launcher_hangman.exe launcher_hangman.cpp)
target_link_libraries(launcher_hangman.exe ${catkin_LIBRARIES}
  CHangMan)
