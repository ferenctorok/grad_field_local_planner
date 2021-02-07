#ifndef UTILS_H
#define UTILS_H

#include <cmath>


namespace gradplanner
{
  /**
   * @brief Returns the length of the vector v.
   * @param v The vector.
   * @return The length of the vector represented by v.
   */
  double get_length(double v[2]);

  /**
   * @struct Structure for storing parameters that a GradFieldController
   * controller would need.
   */
  struct ControlParams
  {
    /**
     * @struct Struct for storing some general params within the
     * ControlParams struct.
     */
    struct General
    {
      unsigned int R = 16; // Effective radius of the RepulsiveField in grid step. Default: 16 [grid cell]
      double end_pos_tol = 0.1; // Goal position tolerance in meters. Default: 0.1 [m]
      double end_ang_tol = 0.2; // Goal orientation tolerance in rad. Default: 0.2 [rad]
      double max_trans_vel = 4.0; // Maximal translational velocity in m/s. Default: 4 [m/s]
      double max_trans_acc = 2.0; // Maximal translational acceleration in m/s². Default: 2 [m/a²]
      double ang_trans_vel = 1.5708; // Maximal angular velocity in rad/s. Default: 1.5708 [rad/s]
      double ang_trans_acc = 3.1415; // Maximal angular acceleration in rad/s². Default: 3.1415 [rad/s²]
      double deceleration_radius = 1.0; // If the robot is inside this radius, it starts to decelerate. In metres. Default: 1 [m] 
    };
    General general;

    /**
     * @struct Struct for storing some params within the
     * ControlParams struct which are related to the "grad_mode"
     * of the GradFieldController.
     */
    struct GradMode
    {
      double K = 0.8; // Proportional control parameter. Defualt: 0.8 [-]
      double boundary_error = 0.2; // param for calculating translational command velocity in rad. Default: 0.2 [rad]
      double max_error = 1.0473; // param for calculating translational command velocity in rad. Default: 1.0473 [rad]
    };
    GradMode grad_mode;

    /**
     * @struct Struct for storing some params within the
     * ControlParams struct which are related to the "direct_mode"
     * of the GradFieldController.
     */
    struct DirectMode
    {
      double min_obst_dist = 6; // If the robot is closer to an obstacle then this -> grad_mode control. Default: 6 [grid cell]
      double K = 0.8; // Proportional control parameter. Defualt: 0.8 [-]
      double boundary_error = 0.2; // param for calculating translational command velocity in rad. Default: 0.2 [rad]
      double max_error = 1.0473; // param for calculating translational command velocity in rad. Default: 1.0473 [rad]
    };
    DirectMode direct_mode;

    /**
     * @struct Struct for storing some params within the
     * ControlParams struct which are related to the "end_mode"
     * of the GradFieldController.
     */
    struct EndMode
    {
      double K = 0.8; // Proportional control parameter. Defualt: 0.8 [-]
    };
    EndMode end_mode;
  };
} // namespace gradplanner

#endif // UTILS_H

