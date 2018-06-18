cmake_minimum_required(VERSION 2.8.3)
project(openvr_library)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

# find the 32 or 64 bit libraries
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "64")
else()
  set(ARCH "86")
endif()

if (WIN32)
  set(OPSYS "win")
elseif (UNIX)
  set(OPSYS "linux")
elseif (DARWIN)
  set(OPSYS "osx")
endif ()
message(STATUS "Detected OS: ${OPSYS}, architecture: ${ARCH}")


# Check that the steamVR SDK is installed 
# (needed to prevent a segfault in OpenVR).
if(CMAKE_HOST_UNIX)
  find_file(OPENVRPATHS openvrpaths.vrpath PATHS $ENV{HOME}/.config/openvr "$ENV{HOME}/Library/Application Support/OpenVR/.openvr")
  if(${OPENVRPATHS} MATCHES OPENVRPATHS-NOTFOUND)
    message(FATAL_ERROR "${OPENVRPATHS} Please install SteamVR SDK to continue..")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES openvr_api
)

###########
## Build ##
###########

include_directories(include/${PROJECT_NAME}/)

set(openvr_api_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${OPSYS}${ARCH}/libopenvr_api.so)

add_library(openvr_api SHARED ${openvr_api_LIBRARY})
add_custom_command(TARGET openvr_api POST_BUILD COMMAND cp ${openvr_api_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libopenvr_api.so)
set_target_properties(openvr_api PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

install(TARGETS openvr_api
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

