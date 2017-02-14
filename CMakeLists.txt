### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(games_vision)
### 3 - Find other CMake/Catkin packages needed for build
SET(ROSDEPS  cvstage  image_transport  message_generation
             music2sound  nav_msgs  people_msgs  sensor_msgs  tf)
find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS ${ROSDEPS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
## Generate messages in the 'msg' folder
add_message_files( FILES AlertMessage.msg )
add_service_files( FILES GetPlayzone.srv )
generate_messages( DEPENDENCIES sensor_msgs )
### 5 - Invoke message/service/action generation (generate_messages())
# Nothing in this package
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ROSDEPS}
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
# get vision_utils includes from SVN
execute_process( # https://cmake.org/cmake/help/v3.0/command/execute_process.html
  COMMAND bash svn_vision_utils.bash
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${music2sound_INCLUDE_DIRS}
  /usr/include/ni/
  ${PCL_INCLUDE_DIRS}
)
SET(games_vision_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
ADD_SUBDIRECTORY(src)
### 8 - Tests to build (catkin_add_gtest())
ADD_SUBDIRECTORY(test)
### 9 - Install rules (install())
# Mark cpp header files for installation
INSTALL(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)
