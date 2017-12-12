cmake_minimum_required(VERSION 2.8.3)
project(modelling)

find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include( ${VTK_USE_FILE} )

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  image_transport
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES modelling
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros
  DEPENDS system_lib
)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})

## Declare a cpp executable
# add_executable(modelling_node src/modelling.cpp)
# target_link_libraries(modelling_node
#    ${catkin_LIBRARIES} ${PCL_LIBRARIES}
#  )
# add_executable(keypoint_node src/keypoint.cpp)
# target_link_libraries(keypoint_node
#    ${catkin_LIBRARIES} ${PCL_LIBRARIES}
#  )
# add_executable(calibration_node src/calibration.cpp src/properties.cpp)
# target_link_libraries(calibration_node
#    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
#  )
add_executable(experiment_node src/experiment.cpp src/properties.cpp src/extract_feature.cpp src/model.cpp)
target_link_libraries(experiment_node
   ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
 )

# add_executable(read_node src/pcl_read.cpp)
# target_link_libraries(read_node
#    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
#  )
# add_executable(write_node src/pcl_write.cpp)
# target_link_libraries(write_node
#    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
#  )
# add_executable(camera_calibration_node src/camera_calibration.cpp)
# target_link_libraries(camera_calibration_node
#    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
#  )

#add_executable(properties_node src/properties.cpp)
#target_link_libraries(properties_node
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
# )

add_executable(main_node src/main.cpp src/modelling.cpp src/properties.cpp src/extract_feature.cpp src/model.cpp src/model_builder.cpp src/recog.cpp)
target_link_libraries(main_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)

#add_executable(recog_node src/recog.cpp src/model.cpp)
#target_link_libraries(recog_node
#  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
#)
add_executable(pose_node src/pose_tracker.cpp )
target_link_libraries(pose_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
)

#add_executable(model_node src/model.cpp )
#target_link_libraries(model_node
#  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
#)


#add_executable(feature_node src/extract_feature.cpp src/properties.cpp)
#target_link_libraries(feature_node
#  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}
#)
