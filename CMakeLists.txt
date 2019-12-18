cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin_manager)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rviz pluginlib message_generation)

include_directories(include)

add_service_files(
	FILES
	PluginLoad.srv
	PluginUnload.srv
	PluginGetConfig.srv
	PluginSetConfig.srv
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	)

link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## Avoid defining "emit", "slots", etc because they can conflict with boost signals.
add_definitions(-DQT_NO_KEYWORDS)


catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

# Build Qt MOC files
if(rviz_QT_VERSION VERSION_LESS "5")
qt4_wrap_cpp(MOC_FILES
	include/${PROJECT_NAME}/plugin_manager.h
  )
else()
qt5_wrap_cpp(MOC_FILES
	include/${PROJECT_NAME}/plugin_manager.h
  )
endif()


add_library(${PROJECT_NAME} src/plugin_manager.cpp ${MOC_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ) 

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#install(TARGETS ${PROJECT_NAME}  
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


#install(FILES
#  plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
