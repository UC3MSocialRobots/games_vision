add_library(CHangMan CHangMan.cpp HangmanSolver.cpp)
target_link_libraries(CHangMan ${catkin_LIBRARIES}
  curl)
add_dependencies(     CHangMan
                      ${games_vision_EXPORTED_TARGETS})

add_executable(launcher_hangman.exe launcher_hangman.cpp)
target_link_libraries(launcher_hangman.exe ${catkin_LIBRARIES}
  CHangMan)
