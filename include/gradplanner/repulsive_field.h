#ifndef REPULSIVE_FIELD_h
#define REPULSIVE_FIELD_h

/* ROS related includes: */
// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

/* gradplanner includes: */
#include <gradplanner/field_utils.h>
#include <gradplanner/grad_field_base.h>

/* standarnd includes: */
#include <queue>

using namespace std;


namespace gradplanner
{
  class RepulsiveField: public GradFieldBase
  {
    public:
      /**
       * @brief Default constructor of the GradFieldBase class. 
       */
      RepulsiveField() {}

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
      ~RepulsiveField() {}

      /**
       * @brief updates the field values and gradients based on the occupancy grid.
       */
      void update_field();

      /**
       * @brief Gets the R member of the class. 
       * @return The radius around obstacles which they infulence.
       */
      unsigned int get_R();

    private:
      unsigned int R;

      /**
       * @brief Initializes the queue of indices to expand, that is
       * it fills it up with the indices of the occupied grid points.
       * @param q Reference to the queue.
       */
      void init_queue(queue<unsigned int* >& q);
  };
}

#endif // REPULSIVE_FIELD_h