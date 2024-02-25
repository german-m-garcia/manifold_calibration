cmake_minimum_required(VERSION 3.0 )
project(manifold_calibration VERSION 0.1.0)

# catkin_simple
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS roscpp roslib laser_geometry rosbag )


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIRS}/include
)
# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )


link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# -------
# ManifoldCalibration Library
# -------

add_library( manifold_calibration 
    src/solver.cpp
    src/kinematics.cpp 
    src/conversions.cpp )

target_include_directories(manifold_calibration PUBLIC
"${PROJECT_SOURCE_DIR}/include"
)


add_executable(${PROJECT_NAME}_node src/calibration.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} 
   ${PCL_LIBRARIES}
   manifold_calibration
 )

