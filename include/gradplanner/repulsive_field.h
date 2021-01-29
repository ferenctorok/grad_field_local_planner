#ifndef REPULSIVE_FIELD_h
#define REPULSIVE_FIELD_h

/* ROS related includes: */
// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

/* gradplanner includes: */
#include <gradplanner/field_utils.h>
#include <gradplanner/grad_field_base.h>


namespace gradplanner
{
  class RepulsiveField: protected GradFieldBase
  {
    public:
      /**
       * @brief Default constructor of the GradFieldBase class. 
       */
      RepulsiveField();

      /**
       * @brief Constructor for the GradFieldBase class.
       * @param costmap 2D ROS costmap about the enironment
       * @param R The radius in grid step, in which the obstacles have an effect
       */
      RepulsiveField(vector<vector<bool >>* occ_grid,
                     unsigned int R);

      /**
       * Defualt destructor of the GradFieldBase class.
       */
      ~RepulsiveField();

    private:
      unsigned int R;

      /**
       * @brief Initializes the field member of the class RepulsiveField.
       */
      void init_field();

      /**
       * @brief updates the field values and gradients based on the costmap from ROS.
       */
      void update_field();
  };
}

#endif // REPULSIVE_FIELD_h