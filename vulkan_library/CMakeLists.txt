cmake_minimum_required(VERSION 2.8.3)
project(vulkan_library)

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
  LIBRARIES vulkan
)

###########
## Build ##
###########

include_directories(include/${PROJECT_NAME}/)

set(vulkan_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${OPSYS}${ARCH}/libvulkan.so)

add_library(vulkan SHARED ${vulkan_LIBRARY})
add_custom_command(TARGET vulkan POST_BUILD COMMAND cp ${vulkan_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libvulkan.so)
set_target_properties(vulkan PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############

install(TARGETS vulkan
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

