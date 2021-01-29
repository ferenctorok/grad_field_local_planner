#include <gradplanner/grad_field_base.h>


namespace gradplanner
{
  GradFieldBase::GradFieldBase(vector<vector<bool >>* occ_grid):
    occ_grid(occ_grid)
    {
      size_x = occ_grid->size() / sizeof((*occ_grid)[0]);
      size_x = (*occ_grid)[0].size() / sizeof(bool);
      // initializing the field:
      field = Field(size_x, size_y);
    }

  unsigned int GradFieldBase::get_size_x() {return size_x;}

  unsigned int GradFieldBase::get_size_y() {return size_y;}
} //namespace gradplanner