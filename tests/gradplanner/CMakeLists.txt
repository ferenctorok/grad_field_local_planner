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