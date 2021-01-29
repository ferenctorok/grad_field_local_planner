#include <gradplanner/repulsive_field.h>


namespace gradplanner
{
  RepulsiveField::RepulsiveField(vector<vector<bool >>* occ_grid,
                                 unsigned int R):
    GradFieldBase(occ_grid), R(R) {}


  void RepulsiveField::update_field()
  {

  }
}