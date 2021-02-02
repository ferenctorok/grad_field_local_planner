#ifndef GRADIENT_FIELD_BASE_H
#define GRADIENT_FIELD_BASE_H

#include <gradplanner/field_utils.h>


namespace gradplanner
{
  /**
   * @class PotentialField
   * @brief Base class for gradient fields
   */
  class GradFieldBase 
  {
    public:
      /**
       * @brief Default constructor of the GradFieldBase class. 
       */
      GradFieldBase() {}

      /**
       * @brief Constructor for the GradFieldBase class.
       * @param costmap 2D ROS costmap about the enironment
       */
      GradFieldBase(vector<vector<bool >>* occ_grid);

      /**
       * Defualt destructor of the GradFieldBase class.
       */
      ~GradFieldBase() {}

      /**
       * @brief Returns the gridsize of the field in the x direction.
       * @return The gridsize in the x direction.
       */
      unsigned int get_size_x();

      /**
       * @brief Returns the gridsize of the field in the y direction.
       * @return The gridsize in the y direction.
       */
      unsigned int get_size_y();

      /**
       * @brief updates the gradient field.
       *        Pure virtual function, has to be defined in child class.
       */
      virtual void update_field() = 0;
    
    protected:
      vector<vector<bool >>* occ_grid;  // occupancy grid.
      unsigned int size_x, size_y;  // x and y size of the field
      gradplanner::Field field; // gradient field.
      vector<int *> search_directions4, search_directions8; // constants for searching directions.

      /**
       * @brief Re-initializes the field member of the class, that is,
       * sets every value to zero and then the values where an obstacle is to 1.
       */
      void re_init_field();
  };
} // namespace gradplanner

#endif // GRADIENT_FIELD_BASE_H