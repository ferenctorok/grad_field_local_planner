# Adding test suites to be executed with CTEST.
if (CXXTEST_FOUND)
  CXXTEST_ADD_TEST(main_test main_test.cc
  ${CMAKE_CURRENT_LIST_DIR}/test1.h)
endif()