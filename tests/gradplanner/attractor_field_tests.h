#include <cxxtest/TestSuite.h>
#include <cmath>
#include <gradplanner/attractor_field.h>


/**
 * @brief Checks whether the 2 gradients are equal.
 * @param grad1 gradient1
 * @param b gradient2
 */
void compare_gradients(const double* grad1, double* grad2)
{
  TS_ASSERT_DELTA(grad1[0], grad2[0], 1e-4);
  TS_ASSERT_DELTA(grad1[1], grad2[1], 1e-4);
}

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
      size_x = 10;
      size_y = 12;

      occ_grid.resize(size_x);
      for (int i = 0; i < size_x; i ++)
      {
        occ_grid[i].resize(size_y);
        for (int j = 0; j < size_y; j ++)
          occ_grid[i][j] = false;
      }

      // adding frame obstacle to it.
      for (int i = 0; i < size_x; i ++)
      {
        occ_grid[i][0] = true;
        occ_grid[i][size_y - 1] = true;
      }

      for (int j = 0; j < size_y; j ++)
      {
        occ_grid[0][j] = true;
        occ_grid[size_x - 1][j] = true;
      }

      // adding an extra obstacle to it:
      occ_grid[5][7] = true;
    }

    /**
     * @brief Tests the constructor of the AttractorField class.
     */
    void test_constructor()
    {
      af = gradplanner::AttractorField(&occ_grid);
      TS_ASSERT_EQUALS(size_x, af.get_size_x());
      TS_ASSERT_EQUALS(size_y, af.get_size_y());

      gradplanner::AttractorField af2(af);
      TS_ASSERT_EQUALS(size_x, af2.get_size_x());
      TS_ASSERT_EQUALS(size_y, af2.get_size_y());
    }

    /**
     * @brief Tests the set_new_goal method.
     */
    void test_set_new_goal()
    {
      bool set_new_goal_success;
      af = gradplanner::AttractorField(&occ_grid);
      double goal1[2] {4.3, 7.7};
      double goal2[2] {5.6, 7.7};
      double goal3[2] {12.9, 5.7};
      double goal4[2] {1.9, 15.7};
      double goal5[2] {-0.4, 5.7};
      double goal6[2] {1.9, -2.6};

      // Assertions:
      set_new_goal_success = af.set_new_goal(goal1);
      TS_ASSERT_EQUALS(true, set_new_goal_success);

      set_new_goal_success = af.set_new_goal(goal2);
      TS_ASSERT_EQUALS(false, set_new_goal_success);

      set_new_goal_success = af.set_new_goal(goal3);
      TS_ASSERT_EQUALS(false, set_new_goal_success);

      set_new_goal_success = af.set_new_goal(goal4);
      TS_ASSERT_EQUALS(false, set_new_goal_success);

      set_new_goal_success = af.set_new_goal(goal5);
      TS_ASSERT_EQUALS(false, set_new_goal_success);

      set_new_goal_success = af.set_new_goal(goal6);
      TS_ASSERT_EQUALS(false, set_new_goal_success);
    }

    /**
     * @brief Tests the update_field method.
     */
    void test_update_field()
    {
      bool set_new_goal_success;
      af = gradplanner::AttractorField(&occ_grid);
      double goal[2] {5.3, 4.7};

      set_new_goal_success = af.set_new_goal(goal);
      af.update_field();

      // Assertions on values:
      TS_ASSERT_EQUALS(af.get_val(5, 4), -1);
      TS_ASSERT_EQUALS(af.get_val(4, 4), -2);
      TS_ASSERT_EQUALS(af.get_val(6, 4), -2);
      TS_ASSERT_EQUALS(af.get_val(5, 3), -2);
      TS_ASSERT_EQUALS(af.get_val(5, 5), -2);
      TS_ASSERT_EQUALS(af.get_val(6, 5), -3);
      TS_ASSERT_EQUALS(af.get_val(6, 3), -3);
      TS_ASSERT_EQUALS(af.get_val(4, 5), -3);
      TS_ASSERT_EQUALS(af.get_val(4, 3), -3);
      TS_ASSERT_EQUALS(af.get_val(8, 7), -7);
      TS_ASSERT_EQUALS(af.get_val(0, 7), 1);

      TS_ASSERT_EQUALS(af.get_val(5, 7), 1);
      TS_ASSERT_EQUALS(af.get_val(5, 6), -3);
      TS_ASSERT_EQUALS(af.get_val(6, 6), -4);
      TS_ASSERT_EQUALS(af.get_val(6, 7), -5);
      TS_ASSERT_EQUALS(af.get_val(6, 8), -6);
      TS_ASSERT_EQUALS(af.get_val(5, 8), -7);

      // Assertions on gradients:
      int aa = 1;
      compare_gradients(af.get_grad(5, 4), new double [2] {0, 0});
      compare_gradients(af.get_grad(4, 4), new double [2] {1, 0});
      compare_gradients(af.get_grad(6, 4), new double [2] {-1, 0});
      compare_gradients(af.get_grad(5, 3), new double [2] {0, 1});
      compare_gradients(af.get_grad(5, 5), new double [2] {0, -1});
      compare_gradients(af.get_grad(4, 3), new double [2] {std::sqrt(2) / 2, std::sqrt(2) / 2});
      compare_gradients(af.get_grad(8, 7), new double [2] {-std::sqrt(2) / 2, -std::sqrt(2) / 2});

      // A special case, where at some point the gradient would point
      // to an obstacle if it weren't prohibited. We check here this case.
      goal[0] = 4;
      goal[1] = 6;

      af.set_new_goal(goal);
      af.update_field();

      const double* grad = af.get_grad(6, 8);

      if (grad[0] == 0)
        compare_gradients(grad, new double [2] {0, -1});
      else
        compare_gradients(grad, new double [2] {-1, 0});
    }


  private:
  unsigned int size_x, size_y;
  vector<vector<bool >> occ_grid;
  gradplanner::AttractorField af;
};