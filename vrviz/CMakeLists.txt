cmake_minimum_required(VERSION 2.8.3)
project(vrviz)

set(PLATFORM 64)
set(SDL_REQUIRED_LIBRARIES SDL2)
add_definitions(-DLINUX -DPOSIX)
set(ARCH_TARGET linux64)
set(SHARED_SRC_DIR src/shared)


  # Better to use the prebuilt GNU preprocessor define __GNUC__,
  # kept for legacy reason with the sample code.
  add_definitions(-DGNUC)

  set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++14 -include ${SHARED_SRC_DIR}/compat.h")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  tf
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
  image_geometry
  pcl_conversions
  pcl_ros
  urdf
  openvr_library
  sdl2_library
  vulkan_library
)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLEW 1.11 REQUIRED)
FIND_PACKAGE(assimp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs roslib tf
)

include_directories(
  .
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIR}
  ${ASSIMP_INCLUDE_DIR}
  src
  ${SHARED_SRC_DIR}
)

file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
)

 add_executable(vrviz_gl
  ${SHARED_SRC_FILES}
                  src/vrviz_gl.cpp
                  src/openvr_gl.cpp
                  src/mesh.cpp
                  src/texture.cpp)
 target_link_libraries(vrviz_gl
  ${catkin_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
  assimp
)

#add_executable(vrviz_vk
#  ${SHARED_SRC_FILES}
#                 src/vrviz_gl.cpp
#                 src/openvr_vk.cpp)
# target_link_libraries(vrviz_vk
#  ${catkin_LIBRARIES}
#  ${CMAKE_DL_LIBS}
#  ${EXTRA_LIBS}
#)
#set_target_properties(vrviz_vk PROPERTIES COMPILE_FLAGS "-DUSE_VULKAN")

add_executable(marker_test src/marker_test.cpp)
target_link_libraries(marker_test ${catkin_LIBRARIES})

