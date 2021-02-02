#include <cxxtest/TestSuite.h>

#include <gradplanner/field_utils.h>
#include <gradplanner/utils.h>

#include <cmath>
#include <queue>
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


/**
 * @class Tests the Pixel class.
 */
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


/**
 * @class Tests the Index class.
 */
class IndexTests : public CxxTest::TestSuite
{
  public:
    /**
    * @brief sets up the tests.
    */
    void setUp()
    {
      shape[0] = 5;
      shape[1] = 6;
    }

    /**
     * @brief tests the constructor.
     */
    void test_constr()
    {
      ind = gradplanner::Index(shape);
      TS_ASSERT_EQUALS(0, ind.get_x());
      TS_ASSERT_EQUALS(0, ind.get_y());
      
      ind = gradplanner::Index(shape, new int [2] {2, 3});
      TS_ASSERT_EQUALS(2, ind.get_x());
      TS_ASSERT_EQUALS(3, ind.get_y());
    }

    /**
     * @brief tests the is_valid() method.
     */
    void test_is_valid()
    {      
      ind = gradplanner::Index(shape, new int [2] {0, 0});
      TS_ASSERT_EQUALS(true, ind.is_valid());

      ind = gradplanner::Index(shape, new int [2] {4, 5});
      TS_ASSERT_EQUALS(true, ind.is_valid());

      ind = gradplanner::Index(shape, new int [2] {4, 6});
      TS_ASSERT_EQUALS(false, ind.is_valid());
    }

    /**
     * @brief Tests the assignment operator overloading.
     */
    void Test_assignment()
    {
      ind = new int [2] {2, 3};
      TS_ASSERT_EQUALS(2, ind.get_x());
      TS_ASSERT_EQUALS(3, ind.get_y());

      queue<gradplanner::Index > q;
      q.push(gradplanner::Index(ind));
      ind = new int [2] {4, 5};

      gradplanner::Index new_ind = q.front();
      TS_ASSERT_EQUALS(2, new_ind.get_x());
      TS_ASSERT_EQUALS(3, new_ind.get_y());
    }

    /**
     * @brief tests the + operator overloading.
     */
    void test_plus_operator()
    {
      gradplanner::Index ind1(shape, new int [2] {2, 3});
      gradplanner::Index ind2(shape, new int [2] {1, 1});

      ind = ind1 + ind2;
      
      TS_ASSERT_EQUALS(3, ind.get_x());
      TS_ASSERT_EQUALS(4, ind.get_y());
    }

  private:
    unsigned int shape[2] {0, 0};
    gradplanner::Index ind;
};


/**
 * @class Tests the Field class.
 */
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

      // indices:
      x = 2;
      y = 4;
      ind = f.get_indexer();
    }

    /**
     * @brief Tests the constructor of the Field class.
     */
    void test_constr()
    {
      unsigned int* shape = f.get_shape();
      TS_ASSERT_EQUALS(N, shape[0]);
      TS_ASSERT_EQUALS(M, shape[1]);

      for (int i = 0; i < N; i ++)
        for (int j = 0; j < M; j ++)
          do_pixel_value_tests(*f.get_pix(i, j), i, j, 0,
           new double[2]{0, 0}, new unsigned int[2]{0, 0});
    }

    /**
     * @brief Tests the get_pix method of the Field class.
     */
    void test_get_pix()
    {
      int a[2] {2, 3};
      ind = a;
      TS_ASSERT_EQUALS(f.get_pix(a[0], a[1]), f.get_pix(ind));
    }

    /**
     * @brief Tests the get_val method of the Field class.
     */
    void test_get_val()
    {
      int a[2] {2, 3};
      ind = a;
      f.set_val(ind, 3);
      TS_ASSERT_EQUALS(f.get_val(ind), 3);
      TS_ASSERT_EQUALS(f.get_val(a[0], a[1]), f.get_val(ind));
    }

    /**
     * @brief Tests the set_val method of the Field class.
     */
    void test_set_val()
    {
      f.set_val(3, 4, 5);
      TS_ASSERT_EQUALS(5, f.get_val(3, 4));
    }

    /**
     * @brief Tests the get_grad method of the Field class.
     */
    void test_get_grad()
    {
      int a[2] {2, 3};
      ind = a;
      f.set_grad(ind, new double [2] {2.3, -4.6});
      TS_ASSERT_EQUALS(f.get_grad(ind)[0], 2.3);
      TS_ASSERT_EQUALS(f.get_grad(ind)[1], -4.6);
      TS_ASSERT_EQUALS(f.get_grad(a[0], a[1])[0], f.get_grad(ind)[0]);
      TS_ASSERT_EQUALS(f.get_grad(a[0], a[1])[1], f.get_grad(ind)[1]);
    }

    /**
     * @brief Tests the set_grad method of the Field class.
     */
    void test_set_grad()
    {
      f.set_grad(3, 4, new double [2] {5.2, 6.3});
      TS_ASSERT_EQUALS(5.2, f.get_grad(3, 4)[0]);
      TS_ASSERT_EQUALS(6.3, f.get_grad(3, 4)[1]);
    }

    /**
     * @brief Tests the get_parent method of the Field class.
     */
    void test_get_parent()
    {
      int a[2] {2, 3};
      ind = a;
      f.set_parent(ind, new unsigned int [2] {4, 2});
      TS_ASSERT_EQUALS(f.get_parent(ind)[0], 4);
      TS_ASSERT_EQUALS(f.get_parent(ind)[1], 2);
      TS_ASSERT_EQUALS(f.get_parent(a[0], a[1])[0], f.get_parent(ind)[0]);
      TS_ASSERT_EQUALS(f.get_parent(a[0], a[1])[1], f.get_parent(ind)[1]);
    }

    /**
     * @brief Tests the set_parent method of the Field class.
     */
    void test_set_parent()
    {
      f.set_parent(3, 4, new unsigned int [2] {5, 6});
      TS_ASSERT_EQUALS(5, f.get_parent(3, 4)[0]);
      TS_ASSERT_EQUALS(6, f.get_parent(3, 4)[1]);
    }

  private:
    int N, M;
    gradplanner::Field f;
    unsigned int x, y;
    gradplanner::Index ind;
};