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

# tests of utils.h:
CXXTEST_ADD_TEST(utils_tests utils_tests.cc
  ${CMAKE_CURRENT_LIST_DIR}/utils_tests.h
)
add_dependencies(utils_tests
  gradplanner
)
target_link_libraries(utils_tests
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

# tests of grad_field_controller.h:
CXXTEST_ADD_TEST(grad_field_controller_tests grad_field_controller_tests.cc
  ${CMAKE_CURRENT_LIST_DIR}/grad_field_controller_tests.h
)
add_dependencies(grad_field_controller_tests
  gradplanner
)
target_link_libraries(grad_field_controller_tests
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