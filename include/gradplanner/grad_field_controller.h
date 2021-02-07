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
       * @brief Sets the pose of the robot.
       * @param pose the Pose to set.
       */
      void set_pose(const Pose& pose);

      /**
       * @brief Sets a new goal pose for the robot.
       * @param goal the goal pose to set.
       * @return True if the given goal was valid.
       */
      bool set_new_goal(const Pose& goal);

      /**
       * @brief Gets the command velocity cmd_vel.
       * @param cmd_vel Reference to the cmd_vel.
       * @return True if everything is ok. (goal is reachable,
       * robot is not in collision and the set goal is valid.)
       */
      bool get_cmd_vel();

    private:
      vector<vector<bool >>* occ_grid_rep;  // occupancy grid.
      vector<vector<bool >>* occ_grid_attr;  // occupancy grid.
      ControlParams* params;  // ControlParams struct for storing parameters that the controller needs.
      AttractorField attractor; // The AttractorField object of the controller.
      RepulsiveField repulsive; // The RepulsiveField object of the controller.
      Pose pose;  // Actual Pose.
      Pose pose_old; // Pose in the previous timestep.
      Pose goal; // Goal pose.
      double goal_pos_rel[2];  // The goal position in the potential field. (Relative position to the robot.)
      bool goal_is_valid; // Flag to indicate that a valid goal is set for the Robot.

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
  };
} // namespace gradplanner

#endif // GRAD_FIELD_CONTROLLER_H