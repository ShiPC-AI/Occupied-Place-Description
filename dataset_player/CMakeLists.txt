add_executable(play_kitti play_kitti.cpp)
target_link_libraries(play_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(play_mulran play_mulran.cpp)
target_link_libraries(play_mulran ${catkin_LIBRARIES} ${PCL_LIBRARIES})