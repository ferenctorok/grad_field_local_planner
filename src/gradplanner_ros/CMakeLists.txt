# adding the gradplanner_ros library as target to build:
add_library(gradplanner_ros
  ${INCLUDE_DIR}/gradplanner_ros/grad_field_local_planner.h
  ${CMAKE_CURRENT_LIST_DIR}/grad_field_local_planner.cpp
  ${CMAKE_CURRENT_LIST_DIR}/get_params.cpp
)

add_dependencies(gradplanner_ros
  gradplanner
)

target_link_libraries(gradplanner_ros
  gradplanner
  ${catkin_LIBRARIES}
)