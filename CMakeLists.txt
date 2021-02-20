cmake_minimum_required(VERSION 2.8.3)
project(onrobot_hex_ft_sensor)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system thread program_options)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES onrobot_hex_driver
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater geometry_msgs roscpp 
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(onrobot_hex_driver src/etherdaq_driver.cpp)
target_link_libraries(onrobot_hex_driver 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(etherdaq_node src/etherdaq_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(etherdaq_node onrobot_hex_driver)



#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS onrobot_hex_driver etherdaq_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
