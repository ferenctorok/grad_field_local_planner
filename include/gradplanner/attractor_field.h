#ifndef ATTRACTOR_FIELD_h
#define ATTRACTOR_FIELD_h

/* gradplanner includes: */
#include <gradplanner/field_utils.h>
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
      AttractorField(vector<vector<bool >>* occ_grid);

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
      double goal[2];
      Index goal_ind;

  };
} // namespace gradplanner

#endif // ATTRACTOR_FIELD_h