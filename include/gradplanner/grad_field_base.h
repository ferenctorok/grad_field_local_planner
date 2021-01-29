#ifndef GRADIENT_FIELD_BASE_H
#define GRADIENT_FIELD_BASE_H

/* ROS related includes: */
// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

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
      GradFieldBase(costmap_2d::Costmap2DROS* costmap);

      /**
       * Defualt destructor of the GradFieldBase class.
       */
      ~GradFieldBase();
    
    protected:
      costmap_2d::Costmap2DROS* costmap;
      gradplanner::Field field;

    private:
      /**
       * @brief Initializes the field member of the class GradFieldBase.
       *        Pure virtual function, has to be overriden in Child class.
       */
      virtual void init_field() = 0;
  };
} // namespace gradplanner

#endif // GRADIENT_FIELD_BASE_H