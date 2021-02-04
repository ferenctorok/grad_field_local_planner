#include <cxxtest/TestSuite.h>
#include <cmath>
#include <gradplanner/attractor_field.h>


/**
 * @brief Tests the AttractorField class.
 */
class AttractorFieldTests : public CxxTest::TestSuite
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
      double a = 2.89;
      int b = int(a);
      TS_ASSERT_EQUALS(b, 2);
    }
};