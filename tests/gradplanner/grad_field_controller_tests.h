#include <cxxtest/TestSuite.h>
#include <cmath>
#include <gradplanner/grad_field_controller.h>


/**
 * @brief Tests the GradFieldController class.
 */
class GradFieldControllerTests: public CxxTest::TestSuite
{
  public:
    /**
     * @brief Sets up the tests.
     */
    void setUp()
    {
    }

    void test1()
    {
      TS_ASSERT_EQUALS(2, 2);
    }
  private:
    gradplanner::GradFieldController controller;
};