#include <cxxtest/TestSuite.h>
#include <cmath>
#include <gradplanner/utils.h>

using namespace std;


/**
 * @brief Tests the GradFieldController class.
 */
class UtilsTests: public CxxTest::TestSuite
{
  public:
    /**
     * @brief Sets up the tests.
     */
    void setUp()
    {
      eps = 1e-4;
    }

    /**
     * @brief Tests the get_length() function.
     */
    void test_get_length()
    {
      double a[2] {1, 1};
      double l = gradplanner::get_length(a);
      TS_ASSERT_DELTA(l, sqrt(2), eps);
      
      l = gradplanner::get_length(2, 2);
      TS_ASSERT_DELTA(l, 2 * sqrt(2), eps);
    }

    /**
     * @brief Tests the sgn function.
     */
    void test_sgn()
    {
      TS_ASSERT_EQUALS(1, gradplanner::sgn<int >(13));
      TS_ASSERT_EQUALS(0, gradplanner::sgn<int >(0));
      TS_ASSERT_EQUALS(-1, gradplanner::sgn<double >(-0.1));
    }

    /**
     * @brief Tests the get_ang_diff() function.
     */
    void test_get_ang_diff()
    {
      double ang_diff = gradplanner::get_ang_diff(2, 1);
      TS_ASSERT_EQUALS(1, ang_diff);

      ang_diff = gradplanner::get_ang_diff(-2, -1);
      TS_ASSERT_EQUALS(-1, ang_diff);

      ang_diff = gradplanner::get_ang_diff(-2 / 3 * PI, 2 / 3 * PI);
      TS_ASSERT_DELTA(2 / 3 * PI, ang_diff, eps);

      ang_diff = gradplanner::get_ang_diff(2 / 3 * PI, -2 / 3 * PI);
      TS_ASSERT_DELTA(-2 / 3 * PI, ang_diff, eps);
    }

    private:
      double eps;
};