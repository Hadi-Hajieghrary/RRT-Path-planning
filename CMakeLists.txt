cmake_minimum_required(VERSION 3.0)
project(RRT-Path-planning)


find_package(Eigen3 3.0.0 REQUIRED)
find_package(OpenCV 4.2 QUIET)
find_package(Boost 1.30 COMPONENTS program_options REQUIRED )

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )

add_executable(${PROJECT_NAME}
  main.cpp src/KDTree.cpp src/utility.cpp src/GridConstraint.cpp src/RRT.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${EIGEN3_LIBS}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  )