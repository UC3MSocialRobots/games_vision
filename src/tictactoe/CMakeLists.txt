add_library(tic_tac_toe_solver tic_tac_toe_solver.cpp
   ${games_vision_INCLUDE_DIR}/tic_tac_toe_solver.h)

add_executable(test_tic_tac_toe_solver.exe test_tic_tac_toe_solver.cpp)
target_link_libraries(test_tic_tac_toe_solver.exe ${catkin_LIBRARIES}
  tic_tac_toe_solver)

add_library(tic_tac_toe_skill tic_tac_toe_skill.cpp
  ${games_vision_INCLUDE_DIR}/tic_tac_toe_skill.h
  ${games_vision_INCLUDE_DIR}/playzone_sequential_user.h)
target_link_libraries(tic_tac_toe_skill ${catkin_LIBRARIES}
  tic_tac_toe_solver)

add_executable(launcher_tic_tac_toe_skill.exe launcher_tic_tac_toe_skill.cpp)
target_link_libraries(launcher_tic_tac_toe_skill.exe ${catkin_LIBRARIES}
  tic_tac_toe_skill)

add_executable(tic_tac_toe_ppl_skill.exe tic_tac_toe_ppl_skill.cpp
             ${games_vision_INCLUDE_DIR}/tic_tac_toe_ppl_skill.h)
target_link_libraries(tic_tac_toe_ppl_skill.exe ${catkin_LIBRARIES}
  tic_tac_toe_skill)
