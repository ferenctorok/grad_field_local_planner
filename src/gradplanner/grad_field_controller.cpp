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
    goal_is_valid(false), goal_pos_reached(false), goal_ang_reached(false)
  {
    // Index where the robot is in the attractor field. (middle of the field.)
    int x_attr = (attractor.get_size_x() - 1) / 2;
    int y_attr = (attractor.get_size_y() - 1) / 2;
    rob_ind_attr = Index(new int [2] {x_attr, y_attr});

    // Index where the robot is in the repulsive field. (middle of the field.)
    int x_rep = (repulsive.get_size_x() - 1) / 2;
    int y_rep = (repulsive.get_size_y() - 1) / 2;
    rob_ind_attr = Index(new int [2] {x_rep, y_rep});

    set_from_params();
  }


  void GradFieldController::set_from_params()
  {
    // general:
    Ts = params->general.Ts;
    R = params->general.R;
    end_pos_tol = params->general.end_pos_tol;
    end_ang_tol = params->general.end_ang_tol;
    max_trans_vel = params->general.max_trans_vel;
    max_trans_acc = params->general.max_trans_acc;
    max_ang_vel = params->general.max_ang_vel;
    max_ang_acc = params->general.max_ang_acc;
    deceleration_radius = params->general.deceleration_radius;

    // grad mode:
    K_grad = params->grad_mode.K;
    boundary_error_grad = params->grad_mode.boundary_error;
    max_error_grad = params->grad_mode.max_error;

    // direct mode:
    min_obst_direct = params->direct_mode.min_obst_dist;
    K_direct = params->direct_mode.K;
    boundary_error_direct = params->direct_mode.boundary_error;
    max_error_direct = params->direct_mode.max_error;

    // end mode:
    K_end = params->end_mode.K;
  }


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
    return ! (*occ_grid_rep)[rob_ind_rep.get_x()][rob_ind_rep.get_y()];
  }


  bool GradFieldController::goal_is_reachable()
  {
    return (attractor.get_val(rob_ind_attr) < 0);
  }


  void GradFieldController::end_controller(double& v_x, double& omega)
  {
    // the translational velocity is in either case 0.
    v_x = 0;

    // proportional omega velocity based on the error:
    double ang_diff = get_ang_diff(state.psi, goal.psi);
    if (abs(ang_diff) > end_ang_tol)
      omega = get_ang_vel(ang_diff, K_end);
    else
    {
      omega = 0;
      goal_ang_reached = true;
    }
  }


  void GradFieldController::direct_controller(double& v_x,
                                              double& omega)
  {
    double des_orient = atan2(goal_rel.y, goal_rel.x);
    double ang_diff = get_ang_diff(state.psi, des_orient);

    // calculating the desired translational and angular velocities:
    v_x = get_trans_vel(ang_diff, boundary_error_direct,
                        max_error_direct);
    omega = get_ang_vel(ang_diff, K_direct);
  }


  void GradFieldController::grad_controller(double& v_x,
                                            double& omega)
  {
    
  }


  bool GradFieldController::is_direct_mode()
  {
    // Check whether the robot is far enough from the nearest obstacle:
    if ((0 < repulsive.get_val(rob_ind_rep)) &&
        (repulsive.get_val(rob_ind_rep) < min_obst_direct)) 
      return false;
    else
    {
      // carrying out raytracing to check whether the
      // path to the goal is free.
      double distance = sqrt(pow(goal_rel.x, 2) + pow(goal_rel.y, 2));
      double dx = goal_rel.x / distance;
      double dy = goal_rel.y / distance;
      double x = rob_ind_rep.get_x() + 0.5;
      double y = rob_ind_rep.get_y() + 0.5;
      
      for (int i = 1; i <= int(distance); i ++)
        if ((*occ_grid_rep)[int(x + i * dx)][int(y + i * dy)])
          return false;
    }

    return true;
  }


  double GradFieldController::get_ang_vel(const double ang_diff,
                                          const double K)
  {
    double omega = - K * ang_diff;
    double epsilon = (state.omega - state_old.omega) / Ts;

    if (abs(epsilon) > max_ang_acc)
      omega = state_old.omega + sgn<double >(epsilon) * max_ang_acc * Ts;

    if (abs(omega) > max_ang_vel)
      omega = sgn<double >(omega) * max_ang_vel;
    
    return omega;
  }


  double GradFieldController::get_trans_vel(const double ang_diff,
                                            const double boundary_error,
                                            const double max_error)
  {
    double v_x;
    
    if (abs(ang_diff) < boundary_error)
      v_x = max_trans_vel;
    else
    {
      if(abs(ang_diff) < max_error)
      {
        double ratio = (abs(ang_diff) - boundary_error) / (max_error - boundary_error);
        v_x = max_trans_vel * (1 - ratio);
      }
      else
        v_x = 0;
    }
      
    // saturating it with the max accelearation:
    double acc = (v_x - state_old.v) / Ts;
    if (abs(acc) > max_trans_acc)
      v_x = state.v + sgn<double >(acc) * max_trans_acc * Ts;

    return v_x;
  }
} // namespace gradplanner
