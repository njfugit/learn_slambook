include_directories("/usr/local/include/eigen3")
add_executable(coordinateTransform coordinateTransform.cpp)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
add_executable(Trajectory Trajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
target_link_libraries(Trajectory ${Pangolin_LIBRARIES})