cmake_minimum_required(VERSION 2.8.3)
project(sdl2_library)

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

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES SDL2 SDL2-2.0
)

###########
## Build ##
###########

include_directories(include/${PROJECT_NAME}/)

set(SDL2_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${OPSYS}${ARCH}/libSDL2.so)
set(SDL2_20_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${OPSYS}${ARCH}/libSDL2-2.0.so.0)

add_library(SDL2 SHARED ${SDL2_LIBRARY})
add_custom_command(TARGET SDL2 POST_BUILD COMMAND cp ${SDL2_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libSDL2.so)
set_target_properties(SDL2 PROPERTIES LINKER_LANGUAGE CXX)

add_library(SDL2-2.0 SHARED ${SDL2_20_LIBRARY})
add_custom_command(TARGET SDL2-2.0 POST_BUILD COMMAND cp ${SDL2_20_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libSDL2-2.0.so)
set_target_properties(SDL2-2.0 PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

install(TARGETS SDL2 SDL2-2.0
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

