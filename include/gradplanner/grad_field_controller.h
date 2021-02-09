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
       * @brief Copy constructor of the GradFieldController class.
       * @param other The other object to copy from.
       */
      GradFieldController(const GradFieldController& other);

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
       * @brief Copy assignment.
       * @param other The other object to copy from.
       */
      GradFieldController& operator=(const GradFieldController& other);

      /**
       * @brief Default Destructor of the GradFieldController class.
       */
      ~GradFieldController() {}

      /**
       * @brief Sets the state of the robot.
       * @param stae the State to set.
       */
      void set_state(const State& state);

      /**
       * @brief Sets a new goal pose for the robot.
       * @param goal the goal pose to set.
       * @return True if the given goal was valid.
       */
      bool set_new_goal(const Pose& goal);

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
      State state;  // Actual State.
      State state_old; // State in the previous timestep.
      Pose goal; // Goal pose.
      Pose goal_rel;  // The goal position in the potential field. (Relative position to the robot.)
      bool goal_is_valid; // Flag to indicate that a valid goal is set for the Robot.
      bool goal_pos_reached; // Flag to indicate whether the goal position was reached with the tolerance.
      bool goal_ang_reached; // Flag to indicate whether the goal orientation was reached with the tolerance.

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
       * @brief Checks whether the controller should be the
       * direct_controller().
       * @return True, if the goal is visible and the robot is
       * further away from the neares obstacle than a specified
       * threshold.
       */
      bool is_direct_mode();

      /**
       * @brief Gets the angular velocity command based on the
       * orientation error and a proportional constant. It also
       * considers the angular velocity and acceleration constraints.
       * @param ang_diff The orientation error.
       * @param K The proportional constant.
       * @return The angular velocity command.
       */
      double get_ang_vel(const double ang_diff, const double K);
  };
} // namespace gradplanner

#endif // GRAD_FIELD_CONTROLLER_H