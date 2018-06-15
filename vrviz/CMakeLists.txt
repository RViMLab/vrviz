cmake_minimum_required(VERSION 2.8.3)
project(vrviz)

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(OPENVR $ENV{HOME}/github/openvr)

set(PLATFORM 64)
set(SDL_REQUIRED_LIBRARIES SDL2)
add_definitions(-DLINUX -DPOSIX)
set(ARCH_TARGET linux64)
set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR}/samples/shared)
set(OPENVR_INCLUDE_DIR ${OPENVR}/headers)

# Check that the steamVR SDK is installed 
# (needed to prevent a segfault in OpenVR).
if(CMAKE_HOST_UNIX)
  find_file(OPENVRPATHS openvrpaths.vrpath PATHS $ENV{HOME}/.config/openvr "$ENV{HOME}/Library/Application Support/OpenVR/.openvr")
  if(${OPENVRPATHS} MATCHES OPENVRPATHS-NOTFOUND)
    message(FATAL_ERROR "${OPENVRPATHS} Please install SteamVR SDK to continue..")
  endif()
endif()

  # Better to use the prebuilt GNU preprocessor define __GNUC__,
  # kept for legacy reason with the sample code.
  add_definitions(-DGNUC)

  set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
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
  pcl_conversions
  pcl_ros
  urdf
)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLEW 1.11 REQUIRED)
FIND_PACKAGE(assimp REQUIRED)
FIND_PACKAGE(ImageMagick COMPONENTS Magick++ REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs roslib tf
)

set(SDL2_INCLUDE_DIR ${THIRDPARTY_DIR}/sdl2-2.0.3/include)

  foreach(lib ${SDL_REQUIRED_LIBRARIES})
    find_library(${lib}_LIBRARY
      NAMES
        ${lib}
      PATHS
        ${THIRDPARTY_DIR}/sdl2-2.0.3/bin
      PATH_SUFFIXES
        linux${PLATFORM}
        ${WINDOWS_PATH_SUFFIXES}
    )
    list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
  endforeach()
  
  
## OpenVR API path
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR}/bin
    ${OPENVR}/lib
  PATH_SUFFIXES
    osx32
    linux64
    ${WINDOWS_PATH_SUFFIXES}
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)


find_library(VULKAN_LIBRARY
  NAMES
    vulkan-1
    vulkan
  PATHS
    ${THIRDPARTY_DIR}/vulkan-1.0.49.0/lib
  PATH_SUFFIXES
    linux${PLATFORM}
    ${WINDOWS_PATH_SUFFIXES}
  NO_DEFAULT_PATH
)
set(VULKAN_INCLUDE_DIR ${THIRDPARTY_DIR}/vulkan-1.0.49.0/include)

include_directories(
  .
  include/${PROJECT_NAME}
  ${OPENVR}/samples
  ${catkin_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIR}
  ${VULKAN_INCLUDE_DIR}
  ${SDL2_INCLUDE_DIR}
  ${QT_INCLUDE_DIRS}
  ${OPENVR_INCLUDE_DIR}
  ${ASSIMP_INCLUDE_DIR}
  ${ImageMagick_INCLUDE_DIRS}
  src
)

file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
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
  ${SDL2_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
  ${ImageMagick_LIBRARIES}
  assimp
)

add_executable(vrviz_vk
  ${SHARED_SRC_FILES}
                 src/vrviz_gl.cpp
                 src/openvr_vk.cpp)
 target_link_libraries(vrviz_vk
  ${catkin_LIBRARIES}
  ${VULKAN_LIBRARY}
  ${SDL2_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
  ${ImageMagick_LIBRARIES}
)
set_target_properties(vrviz_vk PROPERTIES COMPILE_FLAGS "-DUSE_VULKAN")

add_executable(marker_test src/marker_test.cpp)
target_link_libraries(marker_test ${catkin_LIBRARIES})

