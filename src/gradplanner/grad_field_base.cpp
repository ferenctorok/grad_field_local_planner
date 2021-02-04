#include <gradplanner/grad_field_base.h>


namespace gradplanner
{
  GradFieldBase::GradFieldBase(const GradFieldBase& other):
  occ_grid(other.occ_grid), size_x(other.size_x),
  size_y(other.size_y), field(other.field),
  search_directions4(other.search_directions4),
  search_directions8(other.search_directions8) {}

  GradFieldBase::GradFieldBase(vector<vector<bool >>* occ_grid):
    occ_grid(occ_grid)
    {
      size_x = occ_grid->size();
      size_y = (*occ_grid)[0].size();
      // initializing the field:
      field = Field(size_x, size_y);

      // Constants for searching. (Representing the directions to the neighboring
      // cells.)
      search_directions4.push_back(Index(new int [2] {1, 0}));
      search_directions4.push_back(Index(new int [2] {0, 1}));
      search_directions4.push_back(Index(new int [2] {-1, 0}));
      search_directions4.push_back(Index(new int [2] {0, -1}));

      search_directions8.push_back(Index(new int [2] {1, 0}));
      search_directions8.push_back(Index(new int [2] {1, 1}));
      search_directions8.push_back(Index(new int [2] {0, 1}));
      search_directions8.push_back(Index(new int [2] {-1, 1}));
      search_directions8.push_back(Index(new int [2] {-1, 0}));
      search_directions8.push_back(Index(new int [2] {-1, -1}));
      search_directions8.push_back(Index(new int [2] {0, -1}));
      search_directions8.push_back(Index(new int [2] {1, -1}));
    }

  GradFieldBase& GradFieldBase::operator=(const GradFieldBase& other)
  {
    occ_grid = other.occ_grid;
    size_x = other.size_x;
    size_y = other.size_y;
    field = other.field;
    search_directions4 = other.search_directions4;
    search_directions8 = other.search_directions8;

    return *this;
  }

  unsigned int GradFieldBase::get_size_x()
    {return size_x;}

  unsigned int GradFieldBase::get_size_y()
    {return size_y;}

  const int GradFieldBase::get_val(Index ind)
    {return field.get_val(ind);}

  const int GradFieldBase::get_val(unsigned int x, unsigned int y)
    {return field.get_val(x, y);}

  const double* GradFieldBase::get_grad(Index ind)
    {return field.get_grad(ind);}

  const double* GradFieldBase::get_grad(unsigned int x, unsigned int y)
    {return field.get_grad(x, y);}

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

  vector<vector<int >> GradFieldBase::get_values()
  {
    vector<vector<int >> out;
    out.resize(size_x);
    for(int i = 0; i < size_x; i ++)
    {
      out[i].resize(size_y);
      for(int j = 0; j < size_y; j ++)
        out[i][j] = field.get_val(i, j);
    }

    return out;
  }

  const vector<vector<double* >> GradFieldBase::get_grads()
  {
    vector<vector<double* >> out;
    out.resize(size_x);
    for(int i = 0; i < size_x; i ++)
    {
      out[i].resize(size_y);
      for(int j = 0; j < size_y; j ++)
        out[i][j] = field.get_grad(i, j);
    }

    return out;
  }
} //namespace gradplanner