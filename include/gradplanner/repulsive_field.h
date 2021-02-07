#ifndef REPULSIVE_FIELD_h
#define REPULSIVE_FIELD_h

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
       * @brief Default constructor of the RepulsiveField class. 
       */
      RepulsiveField() {}

      /**
       * @brief Copy Constructor of the RepulsiveField class.
       * @param other The other Object to copy.
       */
      RepulsiveField(const RepulsiveField& other);

      /**
       * @brief Constructor for the RepulsiveField class.
       * @param occ_grid Pointer to the occupancy grid. (The occ_grid is
       * true in the entries which are occupied.)
       * @param R The radius in grid step, in which the obstacles have an effect
       */
      RepulsiveField(vector<vector<bool >>* occ_grid,
                     unsigned int R);

      /**
       * Defualt destructor of the RepulsiveField class.
       */
      ~RepulsiveField() {}

      /**
       * @brief Copy assignment.
       * @param other The other oject to copy.
       */
      RepulsiveField& operator=(const RepulsiveField& other);

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
      unsigned int R; // The effective radius of an obstacle in number of pixels.
      Index ind, new_ind, neighbour_ind; // Indices which are used in update_field().
      Pixel *pix, *new_pix; // Pixels corresponding to ind and new_ind used in update_field().
      Index ind0, ind1, ind2, ind3; // Indices which are used in the is_special_case() method.

      /**
       * @brief Initializes the queue of indices to expand, that is
       * it fills it up with the indices of the occupied grid points.
       * @param q Reference to the queue.
       */
      void init_queue(queue<Index >& q);

      /**
       * @brief Sets the value, parent and the gradient of a Pixel.
       * The Index of the Pixel which's value will be set is "new_ind".
       * This Pixel has been reached by expanding the Pixel indexed with "ind".
       */
      void set_new_pixel();

      /**
       * @brief Returns true, if the current Pixel "new_pix" lies on a special
       * place, where we would like to return zero grad, no matter what. This is 
       * the case, when the Pixel has 1-1 obstacles on 2 opposite side of it.
       * @return true if the currecnt Pixel "new_pix" is special in this sense.
       */
      bool is_special_case();
  };
}

#endif // REPULSIVE_FIELD_h