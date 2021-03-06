# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nxt_rviz_plugin)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rviz roscpp nxt_msgs tf)
include_directories(include ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(SOURCE_FILES src/nxt_ultrasonic_display.cpp
                 src/nxt_color_display.cpp 
                 src/init.cpp)


include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  add_library(${PROJECT_NAME} MODULE ${SOURCE_FILES})
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  add_library(${PROJECT_NAME} ${SOURCE_FILES})
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

find_package(PkgConfig)
pkg_check_modules(OGRE OGRE REQUIRED)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS OGRE wxWidgets
    CATKIN_DEPENDS rviz roscpp nxt_msgs tf
    INCLUDE_DIRS src
    LIBRARIES lib
)
