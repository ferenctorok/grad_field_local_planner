#include <gradplanner/repulsive_field.h>
#include <iostream>
#include <cmath>


namespace gradplanner
{
  RepulsiveField::RepulsiveField(const RepulsiveField& other):
    GradFieldBase(other), R(other.R) {}

  RepulsiveField::RepulsiveField(vector<vector<bool >>* occ_grid,
                                 unsigned int R):
    GradFieldBase(occ_grid), R(R) {}

  RepulsiveField& RepulsiveField::operator=(const RepulsiveField& other)
  {
    this->GradFieldBase::operator=(other);
    R = other.R;
  }

  unsigned int RepulsiveField::get_R() {return R;}

  void RepulsiveField::update_field()
  {
    // re-initialize the field:
    re_init_field();

    // Creating and intializing the queue.
    queue<Index > q;
    init_queue(q);

    while (!q.empty())
    {
      ind = q.front();

      for (auto &direction: search_directions_8)
      {
        new_ind = ind + direction;
        if (field.is_valid_index(new_ind))
          if (field.get_val(new_ind) == 0)
          {
            set_new_pixel();
            // if the pixel's distance from an obstacle is smaller the R,
            // it is still expandable. (The obstacle has value 1, so an obstacle
            // at distance D has got a value D+1.)
            if ((field.get_val(new_ind) - 1) < R)
              q.push(Index(new_ind));
          }
      }
      
      q.pop();
    }
  }

  void RepulsiveField::init_queue(queue<Index >& q)
  {
    for (int i = 0; i < size_x; i ++)
    {
      for (int j = 0; j < size_y; j ++)
        if ((*occ_grid)[i][j])
          q.push(Index(new int [2] {i, j}));
    }
  }

  void RepulsiveField::set_new_pixel()
  {
    pix = field.get_pix(ind);
    new_pix = field.get_pix(new_ind);

    // setting the value and parent of the new pixel:
    new_pix->set_val(pix->get_val() + 1);
    new_pix->set_parent(pix->get_parent());

    // setting the gradient of the new pixel //
    // zeroing out the gradient initially:
    double* new_grad = new_pix->get_grad();
    new_grad[0] = 0;
    new_grad[1] = 0;

    // if it is a special case, return the zero grad.
    // What a special case is, is detailed in the function description.
    if (is_special_case()) return ;

    // if the pixel has the value 0 or 1 it also has to have a gradient of 0.
    if (new_pix->get_val() <= 1) return ;

    // else calculate the gradient of the new pixel from its neighbour:
    // The grad of a pixel is the sum of directions from a neighbour to this
    // pixel, if the neighbour is closer to an obstacle then this pixel. (Has smaller value)
    for (auto &direction: search_directions_8)
    {
      neighbour_ind = new_ind + direction;
      if (field.is_valid_index(neighbour_ind))
        if ((field.get_val(neighbour_ind) != 0) &&
           (field.get_val(neighbour_ind) < field.get_val(new_ind)))
        {
          new_grad[0] -= direction.get_x();
          new_grad[1] -= direction.get_y();
        }
    }

    // scale the length of the gradient. The length should
    // decrease linearly from the edge of an obstacle.
    // It has got length 1 at the boarder of an obstacle,
    // that is, in pixels with value 2.
    // double scale = (1 - (new_pix->get_val() - 2) / R);
    double scale = sqrt((1 - pow((new_pix->get_val() - 2) / R, 2)));
    int val = new_pix->get_val();
    new_pix->scale_grad(scale);
  }

  bool RepulsiveField::is_special_case()
  {
    if (new_pix->get_val() == 2)
    {
      // creating the neigboring pixel indices:
      ind0 = new_ind + search_directions_4[0];
      ind1 = new_ind + search_directions_4[1];
      ind2 = new_ind + search_directions_4[2];
      ind3 = new_ind + search_directions_4[3];

      // checking if the 2-2 opposite pixels are occupied.
      if (field.is_valid_index(ind0) && (field.get_val(ind0) == 1) &&
          field.is_valid_index(ind2) && (field.get_val(ind2) == 1))
        return true;
      else if (field.is_valid_index(ind1) && (field.get_val(ind1) == 1) &&
               field.is_valid_index(ind3) && (field.get_val(ind3) == 1))
        return true;
    }

    return false;
  }
}