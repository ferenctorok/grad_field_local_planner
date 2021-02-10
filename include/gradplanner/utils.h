#ifndef UTILS_H
#define UTILS_H

#include <cmath>

const double PI = 3.141592;

namespace gradplanner
{
  /**
   * @brief Returns the length of the vector v.
   * @param v The vector.
   * @return The length of the vector represented by v.
   */
  double get_length(double v[2]);

  /**
   * @brief Returns the length of a vector given by coordinates.
   * @param x The first coordinate of the vector.
   * @param y The second coordinate of the vector.
   * @return The length of the vector.
   */
  double get_length(const double x, const double y);

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
      double Ts = 0.1; // Time step in seconds. Default: 0.1 [s]
      int R = 16; // Effective radius of the RepulsiveField in grid step. Default: 16 [grid cell]
      double end_pos_tol = 0.1; // Goal position tolerance in meters. Default: 0.1 [m]
      double end_ang_tol = 0.2; // Goal orientation tolerance in rad. Default: 0.2 [rad]
      double max_trans_vel = 4.0; // Maximal translational velocity in m/s. Default: 4 [m/s]
      double max_trans_acc = 2.0; // Maximal translational acceleration in m/s². Default: 2 [m/a²]
      double max_ang_vel = 1.5708; // Maximal angular velocity in rad/s. Default: 1.5708 [rad/s]
      double max_ang_acc = 3.1415; // Maximal angular acceleration in rad/s². Default: 3.1415 [rad/s²]
      double decel_ratio = 0.8; // During deceleration the robot uses this partition of its maximal acceleration. Default: 0.8
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
      double boundary_error = 0.5; // param for calculating translational command velocity in rad. Default: 0.5 [rad]
      double max_error = 2 / 3 * PI; // param for calculating translational command velocity in rad. Default: 2/3 PI [rad]
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

    /**
     * @struct Struct for storing some params within the
     * ControlParams struct, which are related to the AttractorField.
     */
    struct AttractorFieldParams
    {
      bool search_dir_8 = false; // whether to use 8 search directions during update. Default: false
    };
    AttractorFieldParams attractor;
  };

  /**
   * @struct Pose structure for storing an x, y, psi pose,
   * where psi is the orientation around the z axis.
   */
  struct Pose
  {
    double x = 0; // x position coordinate in metres. 
    double y = 0; // y position coordinate in metres. 
    double psi = 0; // Orientation around the z axis in radians. Has values between pi and -pi.
  };

  /**
   * @struct Structure to contain the state of the robot.
   * the state vector contains: [x, y, v, psi, omega]
   */
  struct State
  {
    double x = 0; // x position
    double y = 0; // y position
    double v = 0; // translational velocity
    double psi = 0; // orientation around the z axis.
    double omega = 0; // angular velocity around the z axis.
  };

  /**
   * @brief Implements the sign function as a template.
   * @param val Value of type T.
   * @return 1 if val > 0, 0 if val == 0 and -1 otherwise.
   */
  template <typename T>
  int sgn(const T& val)
  {
    if (val > 0) return 1;
    else if (val == 0) return 0;
    else return -1;
  }

  /**
   * @brief Gets the orientation difference between two angles. 
   * The returned orientation difference is always between -pi and pi.
   * @param real The angle that the robot has
   * @param desired The desired angle that the robot should have.
   * @return The orientation difference ang_diff = real - desired,
   * hence it always points from the desired to the real angle.
   */
  double get_ang_diff(const double real, const double desired);
} // namespace gradplanner

#endif // UTILS_H

