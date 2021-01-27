# adding the gradplanner_ros library as target to build:
add_library(gradplanner_ros
  ${CMAKE_CURRENT_LIST_DIR}/grad_field_local_planner.cpp
)
add_dependencies(gradplanner_ros gradplanner)

# Specify libraries to link against the gradplanner_ros target library:
target_link_libraries(gradplanner_ros
  gradplanner
  ${catkin_LIBRARIES}
)