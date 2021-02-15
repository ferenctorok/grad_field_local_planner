#include <cxxtest/TestSuite.h>
#include <gradplanner_ros/grad_field_local_planner.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

/**
 * @brief Tests the RepulsiveField class.
 */
class GradFieldLocalPlannerTests : public CxxTest::TestSuite
{
  public:
    /**
     * @brief sets up the tests.
     */
    void setUp()
    {
      
    }
    void test_1()
    {
      TS_ASSERT_EQUALS(2, 2);
    }

  private:
    grad_field_local_planner::GradFieldPlannerROS planner;
};