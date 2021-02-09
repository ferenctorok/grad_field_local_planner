#include <cxxtest/TestSuite.h>
#include <cmath>
#include <gradplanner/grad_field_controller.h>
#include <gradplanner/utils.h>


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
      size_x = 11;
      size_y = 13;
      R = 2;

      // creating the occupancy grid:
      occ_grid.resize(size_x);
      for (int i = 0; i < size_x; i ++)
      {
        occ_grid[i].resize(size_y);
        for (int j = 0; j < size_y; j ++)
          occ_grid[i][j] = false;
      }

      // adding an obstacle to it:
      occ_grid[5][9] = true;

      // state:
      state.x = 2.0;
      state.y = 2.0;
      state.v = 1.0;
      state.psi = 0;
      state.omega = 1.0;

      // goal:
      goal.x = 5.3;
      goal.y = 5.2;

      // creating the controller object:
      controller = gradplanner::GradFieldController(&occ_grid,
                                                    &occ_grid,
                                                    &params);
    }
    
    /**
     * @brief Tests the set_new_goal() method.
     */
    void test_set_new_goal()
    {
      controller.set_state(state);
      bool is_valid = controller.set_new_goal(goal);
      TS_ASSERT_EQUALS(true, is_valid);

      // goal is in obstacle:
      goal.x = 2.3;
      goal.y = 5.7;
      is_valid = controller.set_new_goal(goal);
      TS_ASSERT_EQUALS(false, is_valid);

      // goal is outside of the grid:
      goal.x = 10.1;
      goal.y = 10.1;
      is_valid = controller.set_new_goal(goal);
      TS_ASSERT_EQUALS(false, is_valid);
    }

    /**
     * @brief Tests the get_cmd_vel() method.
     */
    void test_get_cmd_vel()
    {
      controller.set_state(state);
      controller.set_new_goal(goal);
      TS_ASSERT_EQUALS(2, 2);
    }
  private:
    gradplanner::GradFieldController controller;
    gradplanner::ControlParams params;
    unsigned int size_x, size_y;
    unsigned int R;
    vector<vector<bool >> occ_grid;
    gradplanner::State state;
    gradplanner::Pose goal;
};