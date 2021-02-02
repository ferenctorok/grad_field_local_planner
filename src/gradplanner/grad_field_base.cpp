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

      // Constants for searching. (Representing the directions to the neighboring
      // cells.)
      search_directions4.push_back(Index(field.get_shape(), new int [2] {1, 0}));
      search_directions4.push_back(Index(field.get_shape(), new int [2] {0, 1}));
      search_directions4.push_back(Index(field.get_shape(), new int [2] {-1, 0}));
      search_directions4.push_back(Index(field.get_shape(), new int [2] {0, -1}));

      search_directions8.push_back(Index(field.get_shape(), new int [2] {1, 0}));
      search_directions8.push_back(Index(field.get_shape(), new int [2] {1, 1}));
      search_directions8.push_back(Index(field.get_shape(), new int [2] {0, 1}));
      search_directions8.push_back(Index(field.get_shape(), new int [2] {-1, 1}));
      search_directions8.push_back(Index(field.get_shape(), new int [2] {-1, 0}));
      search_directions8.push_back(Index(field.get_shape(), new int [2] {-1, -1}));
      search_directions8.push_back(Index(field.get_shape(), new int [2] {0, -1}));
      search_directions8.push_back(Index(field.get_shape(), new int [2] {1, -1}));
    }

  unsigned int GradFieldBase::get_size_x() {return size_x;}

  unsigned int GradFieldBase::get_size_y() {return size_y;}

  void GradFieldBase::re_init_field()
  {
    for (int i = 0; i < size_x; i ++)
    {
      for (int j = 0; j < size_y; j ++)
        if ((*occ_grid)[i][j])
        {
          field.set_val(i, j, 1);
          field.set_parent(i, j, new unsigned int [2] {i, j});
        }
        else field.set_val(i, j, 0);
    }
  }
} //namespace gradplanner