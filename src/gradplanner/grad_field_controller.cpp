#include <gradplanner/grad_field_controller.h>
#include <gradplanner/utils.h>

#include <cmath>

using namespace std;

#include <ros/ros.h>


namespace gradplanner
{
  GradFieldController::GradFieldController(vector<vector<bool >>* occ_grid_attr,
                                           vector<vector<bool >>* occ_grid_rep,
                                           ControlParams* params):
    occ_grid_attr(occ_grid_attr), occ_grid_rep(occ_grid_rep), params(params), 
    attractor(occ_grid_attr, params), repulsive(occ_grid_rep, params->general.R),
    goal_is_valid(false)
  {
    // setting up some member variables from the params:
    set_from_params();

    // calculating the deceleration distance:
    decel_distance = pow(max_trans_vel, 2) / (2 * decel_ratio * max_trans_acc);

    // initializing the difference between goal and state to be something bigger than the
    // tolerance for the case when goal_is_reached() is called first without setting state and goal.
    goal_rel.x = end_pos_tol * 2;
    goal_rel.y = end_pos_tol * 2;
    goal_rel.psi = end_ang_tol * 2;
  }


  void GradFieldController::set_from_params()
  {
    // general:
    cell_size = params->general.cell_size;
    Ts = params->general.Ts;
    R = params->general.R;
    end_pos_tol = params->general.end_pos_tol;
    end_ang_tol = params->general.end_ang_tol;
    max_trans_vel = params->general.max_trans_vel;
    max_trans_acc = params->general.max_trans_acc;
    max_ang_vel = params->general.max_ang_vel;
    max_ang_acc = params->general.max_ang_acc;
    decel_ratio = params->general.decel_ratio;

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


  bool GradFieldController::set_state(const State& state,
                                      const double origin_x_attr,
                                      const double origin_y_attr,
                                      const double origin_x_rep,
                                      const double origin_y_rep)
  {
    // state in the global frame:
    this->state = state;

    // goal relative to the state in the global frame:
    set_rel_goal_pose();

    // calculating the state in the normalized coordinate frame of
    // the attractor field.
    state_attr.x = (state.x - origin_x_attr) / cell_size;
    state_attr.y = (state.y - origin_y_attr) / cell_size;
    state_attr.psi = state.psi;

    rob_ind_attr = Index(new int [2] {int(state_attr.x), int(state_attr.y)});

    // calculating the state in the normalized coordinate frame of
    // the attractor field.
    state_rep.x = (state.x - origin_x_rep) / cell_size;
    state_rep.y = (state.y - origin_y_rep) / cell_size;
    state_rep.psi = state.psi;

    rob_ind_rep = Index(new int [2] {int(state_rep.x), int(state_rep.y)});

    return attractor.is_valid_index(rob_ind_attr) && 
           repulsive.is_valid_index(rob_ind_rep);
  }


  bool GradFieldController::set_new_goal(const Pose& goal,
                                         const double origin_x_attr,
                                         const double origin_y_attr)
  {
    // goal in the global frame:
    this->goal = goal;

    // goal relative to the state in the global frame.
    set_rel_goal_pose();

    // calculating the position of the goal in the attractor field
    goal_attr.x = (goal.x - origin_x_attr) / cell_size;
    goal_attr.y = (goal.y - origin_y_attr) / cell_size;

    // set the new goal to the attractor and return true if it is correct.
    goal_is_valid = attractor.set_new_goal(new double [2]
      {goal_attr.x, goal_attr.y});
    return goal_is_valid;
  }


  bool GradFieldController::robot_is_free()
  {
    return ! (*occ_grid_rep)[rob_ind_rep.get_x()][rob_ind_rep.get_y()];
  }


  bool GradFieldController::goal_is_reachable()
  {
    return (attractor.get_val(rob_ind_attr) < 0);
  }


  bool GradFieldController::goal_pos_reached()
  {
    double dist = get_length(goal_rel.x, goal_rel.y);
    return (dist < end_pos_tol);
  }


  bool GradFieldController::goal_ang_reached()
  {
    return (abs(goal_rel.psi) < end_ang_tol);
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
        if (goal_pos_reached())
        {
          if (goal_ang_reached())
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


  bool GradFieldController::goal_is_reached()
  {
    return (goal_pos_reached() && goal_ang_reached());
  }


  void GradFieldController::end_controller(double& v_x, double& omega)
  {
    // the translational velocity is in either case 0.
    v_x = 0;

    // proportional omega velocity based on the error:
    if (abs(goal_rel.psi) > end_ang_tol)
      omega = get_ang_vel(goal_rel.psi, K_end);
    else
      omega = 0;
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
    // We chose 2 neighboring Pixel next to the robot and sum up
    // their gradients together with the gradient of the Pixel where
    // the robot is. This gradient will be the desired orientation.
    // The choice of the 2 neighboring Pixels depends on the orientation of the robot.
    Index ind1_rep, ind1_attr;
    Index ind2_rep, ind2_attr;
    /*if (state.psi > 0)
    {
      ind1_rep = Index(new int [2] {rob_ind_rep.get_x(), rob_ind_rep.get_y() + 1});
      ind1_attr = Index(new int [2] {rob_ind_attr.get_x(), rob_ind_attr.get_y() + 1});
    }
    else
    {
      ind1_rep = Index(new int [2] {rob_ind_rep.get_x(), rob_ind_rep.get_y() - 1});
      ind1_attr = Index(new int [2] {rob_ind_attr.get_x(), rob_ind_attr.get_y() - 1});
    }
      
    if (state.psi < (PI / 2) && state.psi > (-PI / 2))
    {
      ind2_rep = Index(new int [2] {rob_ind_rep.get_x() + 1, rob_ind_rep.get_y()});
      ind2_attr = Index(new int [2] {rob_ind_attr.get_x() + 1, rob_ind_attr.get_y()});
    }
    else
    {
      ind2_rep = Index(new int [2] {rob_ind_rep.get_x() - 1, rob_ind_rep.get_y()});
      ind2_attr = Index(new int [2] {rob_ind_attr.get_x() - 1, rob_ind_attr.get_y()});
    }*/

    // we choose the neighbouring pixels based on the position of the robot.
    if ((state_attr.x - rob_ind_attr.get_x()) > 0.5)
    {
      ind2_rep = Index(new int [2] {rob_ind_rep.get_x() + 1, rob_ind_rep.get_y()});
      ind2_attr = Index(new int [2] {rob_ind_attr.get_x() + 1, rob_ind_attr.get_y()});
    }
    else
    {
      ind2_rep = Index(new int [2] {rob_ind_rep.get_x() - 1, rob_ind_rep.get_y()});
      ind2_attr = Index(new int [2] {rob_ind_attr.get_x() - 1, rob_ind_attr.get_y()});
    }

    if ((state_attr.y - rob_ind_attr.get_y()) > 0.5)
    {
      ind1_rep = Index(new int [2] {rob_ind_rep.get_x(), rob_ind_rep.get_y() + 1});
      ind1_attr = Index(new int [2] {rob_ind_attr.get_x(), rob_ind_attr.get_y() + 1});
    }
    else
    {
      ind1_rep = Index(new int [2] {rob_ind_rep.get_x(), rob_ind_rep.get_y() - 1});
      ind1_attr = Index(new int [2] {rob_ind_attr.get_x(), rob_ind_attr.get_y() - 1});
    }

    const double* g0_r = repulsive.get_grad(rob_ind_rep);
    const double* g0_a = attractor.get_grad(rob_ind_attr);
    const double* g1_r = repulsive.get_grad(ind1_rep);
    const double* g1_a = attractor.get_grad(ind1_attr);
    const double* g2_r = repulsive.get_grad(ind2_rep);
    const double* g2_a = attractor.get_grad(ind2_attr);

    // summing up the gradients and calculate its orientation:
    double dx = g0_r[0] + g0_a[0] + g1_r[0] + g1_a[0] + g2_r[0] + g2_a[0];
    double dy = g0_r[1] + g0_a[1] + g1_r[1] + g1_a[1] + g2_r[1] + g2_a[1];

    double des_orient = atan2(dy, dx);
    double ang_diff = get_ang_diff(state.psi, des_orient);

    // based on this, calculating the command velocities:
    v_x = get_trans_vel(ang_diff, boundary_error_grad,
                        max_error_grad);
    omega = get_ang_vel(ang_diff, K_grad);
  }


  bool GradFieldController::is_direct_mode()
  {
    // Check whether the robot is too close to an obstacle:
    if ((0 < repulsive.get_val(rob_ind_rep)) &&
        (repulsive.get_val(rob_ind_rep) < (min_obst_direct + 1))) 
      return false;
    else
    {
      // carrying out raytracing to check whether the
      // path to the goal is free.
      double distance = get_length(goal_rel.x, goal_rel.y);
      double dx = goal_rel.x / distance;
      double dy = goal_rel.y / distance;
      double x = state_rep.x;
      double y = state_rep.y;
      
      for (int i = 1; i <= int(distance); i ++)
        if ((*occ_grid_rep)[int(x + i * dx)][int(y + i * dy)])
          return false;
    }

    return true;
  }


  void GradFieldController::set_rel_goal_pose()
  {
    goal_rel.x = goal.x - state.x;
    goal_rel.y = goal.y - state.y;
    goal_rel.psi = get_ang_diff(state.psi, goal.psi);
  }


  double GradFieldController::get_ang_vel(const double ang_diff,
                                          const double K)
  {
    double omega = - K * ang_diff;
    double epsilon = (omega - state.omega) / Ts;

    if (abs(epsilon) > max_ang_acc)
      omega = state.omega + sgn<double >(epsilon) * max_ang_acc * Ts;

    if (abs(omega) > max_ang_vel)
      omega = sgn<double >(omega) * max_ang_vel;
    
    return omega;
  }


  double GradFieldController::get_trans_vel(const double ang_diff,
                                            const double boundary_error,
                                            const double max_error)
  {
    double v_x;
    double v_max;
    double dist_to_goal = get_length(goal_rel.x, goal_rel.y);

    // setting the max allowed velocity based on whether we should
    // already decelerate or not.
    if (dist_to_goal > decel_distance)
      v_max = max_trans_vel;
    else
      v_max = max_trans_vel * (dist_to_goal / decel_distance);
    
    // calculating the velocity based on the max allowed velocity
    // and the orientation error:
    if (abs(ang_diff) < boundary_error)
      v_x = v_max;
    else
    {
      if(abs(ang_diff) < max_error)
      {
        double ratio = (abs(ang_diff) - boundary_error) / (max_error - boundary_error);
        v_x = v_max * (1 - ratio);
      }
      else
        v_x = 0;
    }
      
    // saturating it with the max accelearation:
    double acc = (v_x - state.v) / Ts;
    if (abs(acc) > max_trans_acc)
      v_x = state.v + sgn<double >(acc) * max_trans_acc * Ts;

    return v_x;
  }
} // namespace gradplanner
