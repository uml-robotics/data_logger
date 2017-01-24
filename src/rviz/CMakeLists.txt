include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-std=c++0x")        ## Enable C++0x 
endif()

# QT
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

#pkg_check_modules(OGRE OGRE) 
#include_directories( ${OGRE_INCLUDE_DIRS} ) 
#link_directories( ${OGRE_LIBRARY_DIRS} ) 

add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  datalog_widget.h
  datalog_panel.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  datalog_widget.cpp
  datalog_panel.cpp
  ${MOC_FILES}
)

rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES
  DEFINE_SYMBOL THIS_PACKAGE_NAME="${PROJECT_NAME}")

#target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
