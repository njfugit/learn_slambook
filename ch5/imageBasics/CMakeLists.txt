project(imageBasics)

add_executable(imageBasics imageBasics.cpp)
add_executable(image image.cpp)
# 链接OpenCV库
target_link_libraries(image ${OpenCV_LIBS})
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
add_executable(undistort undistort.cpp)
# 链接OpenCV库
target_link_libraries(undistortImage ${OpenCV_LIBS})
target_link_libraries(undistort ${OpenCV_LIBS})