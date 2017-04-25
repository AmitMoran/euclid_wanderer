cmake_minimum_required(VERSION 2.8.3)
project(realsense_wanderer)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
)

generate_dynamic_reconfigure_options(
	cfg/wanderer.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} /usr/include /usr/include/pcl-1.7 /usr/include/eigen3/	
)
###########
## Build ##
###########




add_library( 	realsense_wanderer_nodelet
  				src/wanderer_nodelet.cpp
)

## Specify libraries to link a library or executable target against


target_link_libraries( realsense_wanderer_nodelet 
						${catkin_LIBRARIES} 
						${roscpp_LIBRARIES} 						
)

add_dependencies(realsense_wanderer_nodelet ${PROJECT_NAME}_gencfg wanderer_gencfg)


# Install library
install(TARGETS realsense_wanderer_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
