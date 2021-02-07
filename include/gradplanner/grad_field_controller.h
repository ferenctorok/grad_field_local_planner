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
       * @param occ_grid The occupancy grid.
       * @param params ControlParams object that contains every parameter
       * that the GradFieldController class needs.
       */
      GradFieldController(vector<vector<bool >>* occ_grid,
                          ControlParams* params);

      /**
       * @brief Default Destructor of the GradFieldController class.
       */
      ~GradFieldController() {}

      /**
       * @brief Copy assignment.
       * @param other The other object to copy from.
       */
      GradFieldController& operator=(const GradFieldController& other);

    private:
      vector<vector<bool >>* occ_grid;  // occupancy grid.
      ControlParams* params;  // ControlParams struct for storing parameters that the controller needs.
      Pose pose;  // Actual Pose.
      Pose pose_old; // Pose in the previous timestep.
  };
}

#endif // GRAD_FIELD_CONTROLLER_H