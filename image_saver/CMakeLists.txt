  cmake_minimum_required(VERSION 2.8.0)
  project(image_saver)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
  # Get the information about this package's buildtime dependencies
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nodelet
    sensor_msgs
    image_transport
    message_generation
  )

  FIND_PATH( OpenImageIO_INCLUDE_DIRS OpenImageIO/imageio.h
    /usr/include
  )

  FIND_LIBRARY( OpenImageIO_LIBRARIES OpenImageIO
    /usr/lib
  )

  # Declare that this catkin package's runtime dependencies
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp nodelet image_transport message_runtime
  )


  # message(FATAL_ERROR ${OpenImageIO_LIBRARIES})

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenImageIO_INCLUDE_DIRS}
  )

  add_library(${PROJECT_NAME}
    src/image_saver.cpp
    src/single_main.cpp
    src/single_nodelet.cpp
  )
  target_link_libraries(${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ${OpenImageIO_LIBRARIES}
  )
  add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${OpenImageIO_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

  add_executable(${PROJECT_NAME}_single_node 
  src/single_main.cpp
  )
  target_link_libraries(${PROJECT_NAME}_single_node ${PROJECT_NAME})
