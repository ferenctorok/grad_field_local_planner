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
      eps = 1e-4;
      size_x = 11;
      size_y = 13;

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
      state.omega = 0.0;

      // goal:
      goal.x = 5.3;
      goal.y = 5.2;

      // creating the controller object:
      controller = gradplanner::GradFieldController(&occ_grid,
                                                    &occ_grid,
                                                    &params);

      // setting the state:
      controller.set_state(state);
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
      
      // these tests should use the end_controller() //
      goal.x = state.x + 0.09;
      goal.y = state.y;
      goal.psi = 0.3;
      controller.set_new_goal(goal);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT_EQUALS(0, cmd_v);
      TS_ASSERT_EQUALS(0.24, cmd_omega);

      goal.psi = -0.3;
      controller.set_new_goal(goal);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT_EQUALS(0, cmd_v);
      TS_ASSERT_EQUALS(-0.24, cmd_omega);

      goal.x = state.x + 0.09;
      goal.y = state.y;
      goal.psi = 1;
      controller.set_new_goal(goal);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT_EQUALS(0, cmd_v);
      TS_ASSERT_DELTA(0.31415, cmd_omega, eps);

      goal.psi = -1;
      controller.set_new_goal(goal);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT_EQUALS(0, cmd_v);
      TS_ASSERT_DELTA(-0.31415, cmd_omega, eps);

      // Should be in direct mode //
      params.direct_mode.min_obst_dist = 0;
      controller = gradplanner::GradFieldController(&occ_grid,
                                                    &occ_grid,
                                                    &params);
      goal.x = state.x + 3;
      goal.y = state.y;
      state.psi = 0.1;
      controller.set_state(state);
      controller.set_new_goal(goal);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT_EQUALS(1.2, cmd_v);
      TS_ASSERT_DELTA(-0.08, cmd_omega, eps);

      state.psi = -0.1;
      controller.set_state(state);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT_EQUALS(1.2, cmd_v);
      TS_ASSERT_DELTA(0.08, cmd_omega, eps);

      state.psi = 1.2;
      controller.set_state(state);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT_EQUALS(0.8, cmd_v);
      TS_ASSERT_DELTA(-0.31415, cmd_omega, eps);

      state.psi = -1.2;
      controller.set_state(state);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT_EQUALS(0.8, cmd_v);
      TS_ASSERT_DELTA(0.31415, cmd_omega, eps);

      // should NOT be in direct mode //
      // goal is behind an obstacle:
      goal.x = state.x;
      goal.y = state.y + 4.2;
      state.psi = PI / 2 + 0.1;
      controller.set_state(state);
      controller.set_new_goal(goal);
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT(abs(cmd_omega + 0.08) > eps);

      // robot is too close to an obstacle:
      params.direct_mode.min_obst_dist = 3;
      controller = gradplanner::GradFieldController(&occ_grid,
                                                    &occ_grid,
                                                    &params);
      goal.x = state.x + 3;
      goal.y = state.y;
      state.psi = 0.1;
      controller.set_state(state);
      controller.set_new_goal(goal);
      occ_grid[5][7] = true;
      controller.get_cmd_vel(cmd_v, cmd_omega);
      TS_ASSERT(abs(cmd_omega + 0.08) > eps);

      // setting the occ_grid back:
      occ_grid[5][7] = false;

      // These should be in grad_mode //
      // setting the radius different from the default:
      params.general.R = 2;
      controller = gradplanner::GradFieldController(&occ_grid,
                                                    &occ_grid,
                                                    &params);

      // setting an extra obstacle.
      for (int i = 4; i <= 7; i ++)
        occ_grid[i][9] = true;

      goal.x = state.x;
      goal.y = state.y + 4.2;
      state.psi = 2.5533; // The desired orientation will be around 2.6533
      controller.set_state(state);
      controller.set_new_goal(goal);
      controller.get_cmd_vel(cmd_v, cmd_omega);

      // creting an attractor and repulsive fields for being able to test:
      gradplanner::RepulsiveField rf(&occ_grid, params.general.R);
      rf.update_field();
      gradplanner::AttractorField af(&occ_grid);
      af.set_new_goal(new double [2]{5.0, 10.2});
      af.update_field();

      vector<vector<double* >> grads_rep = rf.get_grads();
      vector<vector<double* >> grads_attr = af.get_grads();

      double* g0_r = grads_rep[5][6];
      double* g0_a = grads_attr[5][6];
      double* g1_r = grads_rep[4][6];
      double* g1_a = grads_attr[4][6];
      double* g2_r = grads_rep[5][7];
      double* g2_a = grads_attr[5][7];

      double dx = g0_r[0] + g0_a[0] + g1_r[0] + g1_a[0] + g2_r[0] + g2_a[0];
      double dy = g0_r[1] + g0_a[1] + g1_r[1] + g1_a[1] + g2_r[1] + g2_a[1];

      // des_orient should be around 2.6533 and hence ang_diff around -0.1
      double des_orient = std::atan2(dy, dx);
      double ang_diff = gradplanner::get_ang_diff(state.psi, des_orient);

      TS_ASSERT_EQUALS(1.2, cmd_v);
      TS_ASSERT_DELTA(-ang_diff * params.grad_mode.K, cmd_omega, eps);

    }

  private:
    gradplanner::GradFieldController controller;
    gradplanner::ControlParams params;
    unsigned int size_x, size_y;
    vector<vector<bool >> occ_grid;
    gradplanner::State state;
    gradplanner::Pose goal;
    double cmd_omega;
    double cmd_v;
    double eps;
};