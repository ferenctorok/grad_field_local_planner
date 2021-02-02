#include <gradplanner/grad_field_base.h>


namespace gradplanner
{
  GradFieldBase::GradFieldBase(vector<vector<bool >>* occ_grid):
    occ_grid(occ_grid)
    {
      size_x = occ_grid->size();
      size_y = (*occ_grid)[0].size();
      // initializing the field:
      field = Field(size_x, size_y);
    }

  unsigned int GradFieldBase::get_size_x() {return size_x;}

  unsigned int GradFieldBase::get_size_y() {return size_y;}

  void GradFieldBase::re_init_field()
  {
    for (unsigned int i = 0; i < size_x; i ++)
    {
      for (unsigned int j = 0; j < size_y; j ++)
        if ((*occ_grid)[i][j]) field.set_val(i, j, 1);
        else field.set_val(i, j, 0);
    }
  }
} //namespace gradplanner