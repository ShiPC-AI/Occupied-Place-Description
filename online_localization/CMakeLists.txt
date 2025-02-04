add_executable(load_map load_map.cpp)
target_link_libraries(load_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(realtime_locate realtime_locate.cpp)
target_link_libraries(realtime_locate ${catkin_LIBRARIES} ${PCL_LIBRARIES})