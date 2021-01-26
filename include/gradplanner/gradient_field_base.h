#ifndef GRADIENT_FIELD_BASE_H
#define GRADIENT_FIELD_BASE_H

/* standard includes */
// timing 
#include <chrono>

// IO
#include <iostream>
using namespace std;

/* ROS related includes: */
// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>


namespace gradplanner
{
  /**
   * @class PotentialField
   * @brief Base class for gradient fields
   */
  class GradientFieldBase
  {
    public:
      /**
       * @brief Default constructor of the GradientFieldBase class. 
       */
      GradientFieldBase();

      /**
       * @brief Constructor for the GradientFieldBase class.
       * @param costmap 2D ROS costmap about the enironment
       */
      GradientFieldBase(costmap_2d::Costmap2DROS* costmap);

      ~GradientFieldBase();
    
    protected:
      costmap_2d::Costmap2DROS* costmap_;
  };
} // namespace gradplanner

#endif // GRADIENT_FIELD_BASE_H