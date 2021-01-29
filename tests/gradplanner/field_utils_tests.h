#include <cxxtest/TestSuite.h>

#include <gradplanner/field_utils.h>
#include <gradplanner/utils.h>

#include <cmath>
using namespace std;


/**
 * @brief Compares the values of the members of the given pixel
 * to the provided values.
 */
void do_pixel_value_tests(gradplanner::Pixel p, unsigned int x, unsigned int y,
int value, double grad[2], unsigned int parent[2])
{
  TS_ASSERT_EQUALS(p.get_x(), x);
  TS_ASSERT_EQUALS(p.get_y(), y);
  TS_ASSERT_EQUALS(p.get_val(), value);
  for (int i=0; i < 2; i++)
  {
    TS_ASSERT_EQUALS(p.get_grad()[i], grad[i]);
    TS_ASSERT_EQUALS(p.get_parent()[i], parent[i]);
  }
}


class PixelTests : public CxxTest::TestSuite
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
      eps = 1e-5;
    }

    /**
     * @brief Tests the constructors of the Pixel class.
     */
    void test_Pixel_constr1()
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

    /**
     * @brief Tests the set methods of the Pixel class.
     */
    void test_Pixel_set_methods()
    {
      p = gradplanner::Pixel();

      // set_val()
      p.set_val(value);
      TS_ASSERT_EQUALS(p.get_val(), value);

      // set_grad()
      p.set_grad(grad);
      TS_ASSERT_EQUALS(p.get_grad()[0], grad[0]);
      TS_ASSERT_EQUALS(p.get_grad()[1], grad[1]);

      // set_parent()
      p.set_parent(parent);
      TS_ASSERT_EQUALS(p.get_parent()[0], parent[0]);
      TS_ASSERT_EQUALS(p.get_parent()[1], parent[1]);
    }

    /**
     * @brief Tests the scale_grad method of the Pixel class.
     */
    void test_Pixel_scale_grad()
    {
      p = gradplanner::Pixel(x, y, value, grad, parent);

      // without length parameter:
      p.scale_grad();
      double* out = p.get_grad();
      TS_ASSERT_DELTA(1, gradplanner::get_length(out), eps);
      TS_ASSERT_DELTA(out[0] / out[1], grad[0] / grad[1], eps);
      TS_ASSERT_EQUALS(signbit(out[0]), signbit(grad[0]));

      // with length parameter:
      p.scale_grad(3.2);
      out = p.get_grad();
      TS_ASSERT_DELTA(3.2, gradplanner::get_length(out), eps);
      TS_ASSERT_DELTA(out[0] / out[1], grad[0] / grad[1], eps);
      TS_ASSERT_EQUALS(signbit(out[0]), signbit(grad[0]));
    }

    /**
     * @brief Tests the normalize_grad method of the Pixel class.
     */
    void test_Pixel_normalize_grad()
    {
      p = gradplanner::Pixel(x, y, value, grad, parent);
      p.normalize_grad();
      double* out = p.get_grad();
      TS_ASSERT_DELTA(1, gradplanner::get_length(out), eps);
      TS_ASSERT_DELTA(out[0] / out[1], grad[0] / grad[1], eps);
      TS_ASSERT_EQUALS(signbit(out[0]), signbit(grad[0]));
    }

  private:
    gradplanner::Pixel p;
    unsigned int x, y, parent[2];
    int value;
    double grad[2];
    double eps;
};


class FieldTests : public CxxTest::TestSuite
{
  public:
    /**
     * @brief sets up the tests.
     */
    void setUp()
    {
      N = 10;
      M = 12;
      f = gradplanner::Field(N, M);
    }

    void test1()
    {
      TS_ASSERT_EQUALS(2, 2);
    }

    /**
     * @brief Tests the constructor of the Field class.
     */
    void test_Field_constr()
    {
      unsigned int* shape = f.get_shape();
      ETS_ASSERT_EQUALS(N, shape[0]);
      ETS_ASSERT_EQUALS(M, shape[1]);

      for (int i = 0; i < N; i ++)
        for (int j = 0; j < M; j ++)
          do_pixel_value_tests(*f.get_pix(i, j), i, j, 0,
           new double[2]{0, 0}, new unsigned int[2]{0, 0});
    }

  private:
    int N, M;
    gradplanner::Field f;
};