#include <gradplanner/grad_field_controller.h>
#include <gradplanner/utils.h>

#include <cmath>

using namespace std;


namespace gradplanner
{
  GradFieldController::GradFieldController(vector<vector<bool >>* occ_grid_attr,
                                           vector<vector<bool >>* occ_grid_rep,
                                           ControlParams* params):
    occ_grid_attr(occ_grid_attr), occ_grid_rep(occ_grid_rep), params(params), 
    attractor(occ_grid_attr), repulsive(occ_grid_rep, params->general.R),
    goal_is_valid(false), goal_pos_reached(false), goal_ang_reached(false) {}

  void GradFieldController::set_state(const State& state)
  {
    this->state = state;
  }

  bool GradFieldController::set_new_goal(const Pose& goal)
  {
    this->goal = goal;
    goal_pos_reached = false;
    goal_ang_reached = false;

    // calculating the relative position of the goal, that is
    // the position of the goal in the potential field.
    goal_rel.x = goal.x - state.x + attractor.get_size_x() / 2;
    goal_rel.y = goal.y - state.y + attractor.get_size_y() / 2;

    goal_is_valid = attractor.set_new_goal(new double [2]
      {goal_rel.x, goal_rel.y});
    return goal_is_valid;
  }

  bool GradFieldController::get_cmd_vel(double& v_x, double& omega)
  {
    if (goal_is_valid && robot_is_free())
    {
      // update he gradient fields:
      attractor.update_field();
      repulsive.update_field();

      if (goal_is_reachable())
      {
        // decide which controller to be used:
        if (goal_pos_reached)
        {
          if (goal_ang_reached)
          {
            v_x = 0;
            omega = 0;
          }
          else end_controller(v_x, omega);
        }
        else
        {
          if (is_direct_mode())
            direct_controller(v_x, omega);
          else
            grad_controller(v_x, omega);
        }

        return true;
      }
    }

    return false;
  }

  bool GradFieldController::robot_is_free()
  {
    // the robot is always in the middle of the occupancy grid, 
    // which hence must have odd size in both directions.
    unsigned int x = (attractor.get_size_x() - 1) / 2;
    unsigned int y = (attractor.get_size_y() - 1) / 2;
    return ! (*occ_grid_rep)[x][y];
  }

  bool GradFieldController::goal_is_reachable()
  {
    // the robot is always in the middle of the occupancy grid, 
    // which hence must have odd size in both directions.
    unsigned int x = (attractor.get_size_x() - 1) / 2;
    unsigned int y = (attractor.get_size_y() - 1) / 2;
    return (attractor.get_val(x, y) < 0);
  }

  void GradFieldController::end_controller(double& v_x, double& omega)
  {
    // the translational velocity is in either case 0.
    v_x = 0;

    // proportional omega velocity based on the error:
    double ang_diff = get_ang_diff(state.psi, goal.psi);
    if (abs(ang_diff) > params->general.end_ang_tol)
      omega = get_ang_vel(ang_diff, params->end_mode.K);
    else
    {
      omega = 0;
      goal_ang_reached = true;
    }
  }

  void GradFieldController::direct_controller(double& v_x,
                                              double& omega)
  {

  }

  void GradFieldController::grad_controller(double& v_x,
                                            double& omega)
  {

  }

  bool GradFieldController::is_direct_mode()
  {

  }

  double GradFieldController::get_ang_vel(const double ang_diff,
                                          const double K)
  {
    double omega = - K * ang_diff;

    return omega;
  }
} // namespace gradplanner
