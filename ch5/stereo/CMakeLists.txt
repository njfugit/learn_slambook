find_package(Pangolin REQUIRED)

add_executable(stereoVision stereoVision.cpp)
add_executable(stereo stereo.cpp)
add_executable(disparity test_disparity.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
target_link_libraries(stereo ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
target_link_libraries(disparity ${OpenCV_LIBS})