#ifndef GRAD_FIELD_CONTROLLER_H
#define GRAD_FIELD_CONTROLLER_H

#include <gradplanner/attractor_field.h>
#include <gradplanner/repulsive_field.h>


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
     * @brief Constructor of the  of the GradFieldController class.
     */
    GradFieldController(vector<vector<bool >>* occ_grid,
                        unsigned int R);

    /**
     * @brief Default Destructor of the GradFieldController class.
     */
    ~GradFieldController() {}
};

#endif // GRAD_FIELD_CONTROLLER_H