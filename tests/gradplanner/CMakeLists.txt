# tests of utils:
CXXTEST_ADD_TEST(utils_tests utils_tests.cc
  ${CMAKE_CURRENT_LIST_DIR}/utils_tests.h
)

target_link_libraries(utils_tests
  gradplanner
)