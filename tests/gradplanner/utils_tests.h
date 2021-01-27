#include <cxxtest/TestSuite.h>

#include <gradplanner/utils.h>


class MyTestSuite : public CxxTest::TestSuite
{
  public:
    /**
     * @brief sets up the tests.
     */
    void setUp()
    {
      x = 1;
      y = 2;
      value = 3;
      grad[0] = 1.1;
      grad[1] = -1.2;
      parent[0] = 4;
      parent[1] = 5;
    }

    /**
     * @brief Tests the constructors of the Pixel class.
     */
    void test_constr1()
    {
      // default constructor:
      p = gradplanner::Pixel();
      do_pixel_value_tests(p, 0, 0, 0,
        new double[2]{0, 0}, new unsigned int[2]{0, 0});

      // x, y provided:
      p = gradplanner::Pixel(x, y);
      do_pixel_value_tests(p, x, y, 0,
        new double[2]{0, 0}, new unsigned int[2]{0, 0});

      // x, y, value provided:
      p = gradplanner::Pixel(x, y, value);
      do_pixel_value_tests(p, x, y, value,
        new double[2]{0, 0}, new unsigned int[2]{0, 0});

      // x, y, value, grad provided:
      p = gradplanner::Pixel(x, y, value, grad);
      do_pixel_value_tests(p, x, y, value,
        grad, new unsigned int[2]{0, 0});

      // everything is provided:
      p = gradplanner::Pixel(x, y, value, grad, parent);
      do_pixel_value_tests(p, x, y, value, grad, parent);
    }

  private:
    gradplanner::Pixel p;
    unsigned int x, y, parent[2];
    int value;
    double grad[2];


    /**
     * @brief Compares the values of the members of the given pixel
     * to the provided values.
     */
    void do_pixel_value_tests(gradplanner::Pixel p, unsigned int x, unsigned int y,
      int value, double grad[2], unsigned int parent[2])
      {
        std::cout << "yoller2" << std::endl;
        TS_ASSERT_EQUALS(p.get_x(), x);
        TS_ASSERT_EQUALS(p.get_y(), y);
        TS_ASSERT_EQUALS(p.get_val(), value);
        for (int i=0; i < 2; i++)
        {
          TS_ASSERT_EQUALS(p.get_grad()[i], grad[i]);
          TS_ASSERT_EQUALS(p.get_parent()[i], parent[i]);
        }
      }
};