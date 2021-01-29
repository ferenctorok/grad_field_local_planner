#ifndef GRADIENT_FIELD_BASE_H
#define GRADIENT_FIELD_BASE_H

/* ROS related includes: */
// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

/* gradplanner includes: */
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
      GradFieldBase();

      /**
       * @brief Constructor for the GradFieldBase class.
       * @param costmap 2D ROS costmap about the enironment
       */
      GradFieldBase(vector<vector<bool >>* occ_grid);

      /**
       * Defualt destructor of the GradFieldBase class.
       */
      ~GradFieldBase();

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
      unsigned int size_x, size_y;
      gradplanner::Field field; // gradient field.
  };
} // namespace gradplanner

#endif // GRADIENT_FIELD_BASE_H