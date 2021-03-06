#ifndef ATTRACTOR_FIELD_h
#define ATTRACTOR_FIELD_h

/* gradplanner includes: */
#include <gradplanner/field_utils.h>
#include <gradplanner/utils.h>
#include <gradplanner/grad_field_base.h>

/* standarnd includes: */
#include <queue>

using namespace std;


namespace gradplanner
{
  /**
 * @class AttractorField class for creating the attractor field 
 * that "pulls" the robot in the direction of the goal. 
 * It inherits from the GradFieldBase class.
 */
  class AttractorField: public GradFieldBase
  {
    public:
      /**
       * @brief Default constructor of the AttractorField class. 
       */
      AttractorField() {}

      /**
       * @brief Copy Constructor of the AttractorField class.
       * @param other The other Object to copy.
       */
      AttractorField(const AttractorField& other);

      /**
       * @brief Constructor for the AttractorField class.
       * @param occ_grid Pointer to the occupancy grid. (The occ_grid is
       * true in the entries which are occupied.)
       */
      AttractorField(vector<vector<bool >>* occ_grid,
                     ControlParams* params=nullptr);

      /**
       * Defualt destructor of the AttractorField class.
       */
      ~AttractorField() {}

      /**
       * @brief Copy assignment.
       * @param other The other oject to copy.
       */
      AttractorField& operator=(const AttractorField& other);

      /**
       * @brief Sets a new goal.
       * @param new_goal The new goal to set.
       * @return Returns true if the goal was valid, that is, it
       * lies within the boundaries of the AttractorField.
       */
      bool set_new_goal(double new_goal[2]);

      /**
       * @brief updates the field values and gradients based on the occupancy grid.
       */
      void update_field();

    private:
      double goal[2]; // the goal position
      Index goal_ind; // The index of the Pixel in which the goal is.
      bool goal_is_valid, goal_is_free; // Flags used in set_new_goal() method.
      Index ind, new_ind, neighbour_ind, tester_ind; // Indices which are used in update_field().
      Pixel *pix, *new_pix; // Pixels corresponding to ind and new_ind used in update_field().
      bool grad_is_zero, neighbour_is_occupied; // Some flags used in update_field().
      double tester_ind_f[2]; // array used in update_field()
      ControlParams* params; // params
      vector<Index >* search_directions; // The search directions which are going to be used for expansion.

      /**
       * @brief Sets the value and the gradient of a Pixel.
       * The Index of the Pixel which's value will be set is "new_ind".
       * This Pixel has been reached by expanding the Pixel indexed with "ind".
       */
      void set_new_pixel();
  };
} // namespace gradplanner

#endif // ATTRACTOR_FIELD_h