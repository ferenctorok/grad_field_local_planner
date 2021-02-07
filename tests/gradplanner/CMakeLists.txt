# tests of field_utils.h:
CXXTEST_ADD_TEST(field_utils_tests field_utils_tests.cc
  ${CMAKE_CURRENT_LIST_DIR}/field_utils_tests.h
)
add_dependencies(field_utils_tests
  gradplanner
)
target_link_libraries(field_utils_tests
  gradplanner
)

# tests of repulsive_field.h:
CXXTEST_ADD_TEST(repulsive_field_tests repulsive_field_tests.cc
  ${CMAKE_CURRENT_LIST_DIR}/repulsive_field_tests.h
)
add_dependencies(repulsive_field_tests
  gradplanner
)
target_link_libraries(repulsive_field_tests
  gradplanner
)

# tests of attractor_field.h:
CXXTEST_ADD_TEST(attractor_field_tests attractor_field_tests.cc
  ${CMAKE_CURRENT_LIST_DIR}/attractor_field_tests.h
)
add_dependencies(attractor_field_tests
  gradplanner
)
target_link_libraries(attractor_field_tests
  gradplanner
)

# Timing:
add_executable(gradplanner_timing
  gradplanner_timing.cpp
)
add_dependencies(gradplanner_timing
  gradplanner
)
target_link_libraries(gradplanner_timing
  gradplanner
)