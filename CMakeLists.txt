cmake_minimum_required(VERSION 2.8.3)
project(optoforce_etherdaq_driver)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(CURL REQUIRED)
find_package(TinyXML REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES etherdaq_driver
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater geometry_msgs roscpp 
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(etherdaq_driver src/etherdaq_driver.cpp)
target_link_libraries(etherdaq_driver 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${CURL_LIBRARIES} 
  ${TinyXML_LIBRARIES})

## Declare a cpp executable
add_executable(etherdaq_node src/etherdaq_node.cpp)
add_executable(etherdaq_subscriber src/etherdaq_subscriber.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(etherdaq_node etherdaq_driver)
target_link_libraries(etherdaq_subscriber ${catkin_LIBRARIES})



#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS etherdaq_driver etherdaq_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
