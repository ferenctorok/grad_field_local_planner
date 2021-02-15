#ifndef GRAD_FIELD_CONTROLLER_H
#define GRAD_FIELD_CONTROLLER_H

#include <gradplanner/attractor_field.h>
#include <gradplanner/repulsive_field.h>
#include <gradplanner/utils.h>


namespace gradplanner
{
  /**
   * @class GradFieldController is a high level controller for
   * a non-holonomic robot based on an attractor and a repulsive
   * gradient fields. It outputs 3 command velocities:
   *  v_x : translational in the x direction.
   *  v_y: translational in the y direction.
   *  omega: rotational around the z axis.
   */
  class GradFieldController
  {    
    public:
      /**
       * @brief Default Constructor of the GradFieldController class.
       */
      GradFieldController() {}

      /**
       * @brief Constructor of the of the GradFieldController class.
       * @param occ_grid_attr The occupancy grid for the attractor field.
       * @param occ_grid_attr The occupancy grid for the repulsive field.
       * @param params ControlParams object that contains every parameter
       * that the GradFieldController class needs.
       */
      GradFieldController(vector<vector<bool >>* occ_grid_attr,
                          vector<vector<bool >>* occ_grid_rep,
                          ControlParams* params);

      /**
       * @brief Default Destructor of the GradFieldController class.
       */
      ~GradFieldController() {}

      /**
       * @brief Sets the state of the robot.
       * @param state the State to set.
       * @param origin_x_attr The x coordinate of the origin of the attractor field.
       * @param origin_y_attr The y coordinate of the origin of the attractor field.
       */
      void set_state(const State& state,
                     const double origin_x_attr,
                     const double origin_y_attr);

      /**
       * @brief Sets a new goal pose for the robot.
       * @param goal the goal pose to set.
       * @param origin_x_attr The x coordinate of the origin of the attractor field.
       * @param origin_y_attr The y coordinate of the origin of the attractor field.
       * @return True if the given goal was valid.
       */
      bool set_new_goal(const Pose& goal,
                        const double origin_x_attr,
                        const double origin_y_attr);

      /**
       * @brief Checks if the robot is in free space.
       * @return True if the robot is in free space.
       */
      bool robot_is_free();

      /**
       * @brief Checks if the goal is reachable.
       * @return True if the goal is reachable.
       */
      bool goal_is_reachable();

      /**
       * @brief Gets the command velocity cmd_vel.
       * @param v_x Reference to the x translational velocity.
       * @param omega Reference ot the angular velocity around the z axis.
       * @return True if everything is ok. (goal is reachable,
       * robot is not in collision and the set goal is valid.)
       */
      bool get_cmd_vel(double& v_x, double& omega);

    private:
      vector<vector<bool >>* occ_grid_rep;  // occupancy grid.
      vector<vector<bool >>* occ_grid_attr;  // occupancy grid.
      ControlParams* params;  // ControlParams struct for storing parameters that the controller needs.
      AttractorField attractor; // The AttractorField object of the controller.
      RepulsiveField repulsive; // The RepulsiveField object of the controller.
      State state;  // Actual State in global frame
      Pose state_attr;  // Pose in the attractor field.
      Pose state_rep; // Pose in the repulsive field.
      Pose goal; // Goal pose in the global frame.
      Pose goal_rel; // Goal position relative to the robot in the global frame.
      Pose goal_attr;  // The goal position in the attractor potential field.
      bool goal_is_valid; // Flag to indicate that a valid goal is set for the Robot.
      Index rob_ind_attr;  // The index of the cell in the attractor field where the robot is.
      Index rob_ind_rep;   // The index of the cell in the attractor field where the robot is.
      double decel_distance; // The distance from the goal, where the robot should start decelerating.
      
      // params which are set up from the given params:
      // general:
      double cell_size; // The cell size of the grid.
      double Ts; // Time step in seconds.
      unsigned int R; // Effective radius of the RepulsiveField in grid step.
      double end_pos_tol; // Goal position tolerance in meters.
      double end_ang_tol; // Goal orientation tolerance in rad.
      double max_trans_vel; // Maximal translational velocity in m/s.
      double max_trans_acc; // Maximal translational acceleration in m/s²
      double max_ang_vel; // Maximal angular velocity in rad/s. Default:
      double max_ang_acc; // Maximal angular acceleration in rad/s².
      double decel_ratio; // During deceleration the robot uses this partition of its maximal acceleration.
      
      // grad mode:
      double K_grad; // Proportional control parameter. Defualt: 0.8 [-]
      double boundary_error_grad; // param for calculating translational command velocity in rad.
      double max_error_grad; // param for calculating translational command velocity in rad.
      
      // direct mode:
      double min_obst_direct; // If the robot is closer to an obstacle then this -> grad_mode control.
      double K_direct; // Proportional control parameter.
      double boundary_error_direct; // param for calculating translational command velocity in rad.
      double max_error_direct; // param for calculating translational command velocity in rad.

      // end mode:
      double K_end; // Proportional control parameter.

      /**
       * @brief sets up some member variables of the object based
       * on the given parameters.
       */
      void set_from_params();

      /**
       * @brief Checks whether the goal position is reached with the tolerance.
       * @return True if the goal position is reached.
       */
      bool goal_pos_reached();

      /**
       * @brief Checks whether the goal orientation is reached with the tolerance.
       * @return True if the goal orientation is reached.
       */
      bool goal_ang_reached();

      /**
       * @brief Checks whether the controller should be the
       * direct_controller().
       * @return True, if the goal is visible and the robot is
       * further away from the neares obstacle than a specified
       * threshold.
       */
      bool is_direct_mode();

      /**
       * @brief sets the relative goal position to the state.
       */
      void set_rel_goal_pose();

      /**
       * @brief The controller of the last period. It is used,
       * when the goal position is already reached, only the 
       * orientation of the robot has to be adjusted.
       * @param v_x Reference to the x translational velocity.
       * @param omega Reference ot the angular velocity around the z axis.
       */
      void end_controller(double& v_x, double& omega);

      /**
       * @brief The controller that is used if the goal is 
       * visible from the robot and the robot is further away from
       * an obstacle than a specified threshold. In this case the robot
       * tries to go directly towards the goal.
       * @param v_x Reference to the x translational velocity.
       * @param omega Reference ot the angular velocity around the z axis.
       */
      void direct_controller(double& v_x, double& omega);

      /**
       * @brief Controller for navigating in the gradient field.
       * It uses the attractor and the repulsive gradient fields.
       * @param v_x Reference to the x translational velocity.
       * @param omega Reference ot the angular velocity around the z axis.
       */
      void grad_controller(double& v_x, double& omega);

      /**
       * @brief Gets the angular velocity command based on the
       * orientation error and a proportional constant. It also
       * considers the angular velocity and acceleration constraints.
       * @param ang_diff The orientation error.
       * @param K The proportional constant.
       * @return The angular velocity command.
       */
      double get_ang_vel(const double ang_diff, const double K);

      /**
       * @brief Gets the translational velocity command. It considers
       * the maximal translational velocity and acceleration values
       * and also the orientation error of the robot.
       * If the orientation error is smaller than the boundary error value
       * this value, the robot tries to go as fast as possible. 
       * If the robot is between the boundary and the max errors,
       * the velocity is linearly decreesing with the error.
       * @param ang_diff The orientation error.
       * @param boundary_error The boundary error.
       * @param max_error The max error.
       * @return The translational velocity command.
       */
      double get_trans_vel(const double ang_diff,
                           const double boundary_error,
                           const double max_error);
  };
} // namespace gradplanner

#endif // GRAD_FIELD_CONTROLLER_H