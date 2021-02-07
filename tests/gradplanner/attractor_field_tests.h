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
     * @brief Tests the set_new_goal method of the AttractorField class.
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

  private:
  unsigned int size_x, size_y;
  vector<vector<bool >> occ_grid;
  gradplanner::AttractorField af;
};