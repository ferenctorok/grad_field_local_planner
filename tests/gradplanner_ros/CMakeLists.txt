# tests of grad_field_local_planner.h:
CXXTEST_ADD_TEST(grad_field_local_planner_tests grad_field_local_planner_tests.cc
  ${CMAKE_CURRENT_LIST_DIR}/grad_field_local_planner_tests.h
)
add_dependencies(grad_field_local_planner_tests
  gradplanner_ros
)
target_link_libraries(grad_field_local_planner_tests
  gradplanner_ros
)