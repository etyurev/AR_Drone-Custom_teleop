cmake_minimum_required(VERSION 2.8.3)
project(depthDetection)





# Required catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

## OpenCV package
find_package( OpenCV REQUIRED)


#  INCLUDE_DIRS include
#  LIBRARIES P4GUI
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p4ll
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable

 add_executable(${PROJECT_NAME}_node src/depthDetect.cpp)
# Libraries

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${OpenCV_LIBRARIES}
 )

add_executable(${PROJECT_NAME}_sample src/sampleData.cpp
src/preProcess.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_sample
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${OpenCV_LIBRARIES}

 )

 add_executable(${PROJECT_NAME}_sort_data src/sort.cpp)
 ## Specify libraries to link a library or executable target against
  target_link_libraries(${PROJECT_NAME}_sort_data
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${OpenCV_LIBRARIES}
  )

  add_executable(${PROJECT_NAME}_train_classifier src/trainClassifier.cpp)
  ## Specify libraries to link a library or executable target against
   target_link_libraries(${PROJECT_NAME}_train_classifier
     ${catkin_LIBRARIES}
     ${OpenCV_LIBS}
     ${OpenCV_LIBRARIES}
   )

   add_executable(${PROJECT_NAME}_collect_data src/collect_data.cpp)
   ## Specify libraries to link a library or executable target against
    target_link_libraries(${PROJECT_NAME}_collect_data
      ${catkin_LIBRARIES}
      ${OpenCV_LIBS}
      ${OpenCV_LIBRARIES}
    )

    add_executable(${PROJECT_NAME}_test src/test.cpp
    src/HOG.cpp src/preProcess.cpp)
    ## Specify libraries to link a library or executable target against
     target_link_libraries(${PROJECT_NAME}_test
       ${catkin_LIBRARIES}
       ${OpenCV_LIBS}
       ${OpenCV_LIBRARIES}

     )
