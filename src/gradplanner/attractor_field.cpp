#include <gradplanner/attractor_field.h>


namespace gradplanner
{
  AttractorField::AttractorField(const AttractorField& other):
    GradFieldBase(other)
  {
    goal[0] = other.goal[0];
    goal[1] = other.goal[1];
    goal_ind = other.goal_ind;
    params = other.params;

    // chosing how many search directions to use.
    if ((params != nullptr) && params->attractor_params.search_dir_8)
      search_directions = &search_directions_8;
    else
      search_directions = &search_directions_4;
  }

  AttractorField::AttractorField(vector<vector<bool >>* occ_grid,
                                 ControlParams* params):
    GradFieldBase(occ_grid), goal{0.0, 0.0},
    goal_ind(new int [2] {0, 0}), params(params)
  {
    // chosing how many search directions to use.
    if ((params != nullptr) && params->attractor_params.search_dir_8)
      search_directions = &search_directions_8;
    else
      search_directions = &search_directions_4;
  }

  AttractorField& AttractorField::operator=(const AttractorField& other)
  {
    this->GradFieldBase::operator=(other);
    goal[0] = other.goal[0];
    goal[1] = other.goal[1];
    goal_ind = other.goal_ind;
    params = params;

    // chosing how many search directions to use.
    if ((params != nullptr) && params->attractor_params.search_dir_8)
      search_directions = &search_directions_8;
    else
      search_directions = &search_directions_4;

    return *this;
  }

  bool AttractorField::set_new_goal(double new_goal[2])
  {
    goal[0] = new_goal[0];
    goal[1] = new_goal[1];
    goal_ind = Index(new int [2] {int(goal[0]), int(goal[1])});

    if (field.is_valid_index(goal_ind))
    {
      goal_is_free = ! (*occ_grid)[goal_ind.get_x()][goal_ind.get_y()];
      if (goal_is_free)
        return true;
    }
    
    return false;
  }

  void AttractorField::update_field()
  {
    // re-initialize the field:
    re_init_field();

    // Creating and intializing the queue.
    queue<Index > q;
    q.push(Index(goal_ind));

    // setting the value of the pixel where the goal is to -1:
    field.set_val(goal_ind, -1);

    while (!q.empty())
    {
      ind = q.front();

      for (auto &direction: *search_directions)
      {
        new_ind = ind + direction;
        if (field.is_valid_index(new_ind))
          if (field.get_val(new_ind) == 0)
          {
            set_new_pixel();
            q.push(Index(new_ind));
          }
      }

      q.pop();
    }
  }

  void AttractorField::set_new_pixel()
  {
    pix = field.get_pix(ind);
    new_pix = field.get_pix(new_ind);

    // setting the value:
    new_pix->set_val(pix->get_val() - 1);

    // setting the gradient of the new pixel //
    // zeroing out the gradient initially:
    double* new_grad = new_pix->get_grad();
    new_grad[0] = 0;
    new_grad[1] = 0;

    // The gradient can be obtained by summing up the directions
    // in which the neighbouring pixel is closer to the goal,
    // that is, it has got a bigger value.
    for (auto &direction: search_directions_8)
    {
      neighbour_ind = new_ind + direction;
      if (field.is_valid_index(neighbour_ind))
        if ((field.get_val(neighbour_ind) < 0) &&
           (field.get_val(neighbour_ind) > field.get_val(new_ind)))
        {
          new_grad[0] += direction.get_x();
          new_grad[1] += direction.get_y();
        }
    }

    new_pix->normalize_grad();

    /*If the sum is zero, then this pixel is a saddle point. For
    this not to cause any trouble, the gradient is set to point
    to any of the neighboring cells which are closer to the goal.
    Also, it is possible that the resulting grad points to
    a neighboring occupied Pixel. Then it is also set to point
    to one of the neighbouring not occuied cells wich is closer
    to the goal.*/

    // checking if grad is zero:
    grad_is_zero = ((new_grad[0] == 0) && (new_grad[1] == 0));

    // checking if the neighboring cell to which the gradient 
    // points is occupied.
    tester_ind_f[0] = new_ind.get_x() + 0.5 + new_grad[0];
    tester_ind_f[1] = new_ind.get_y() + 0.5 + new_grad[1];
    tester_ind = Index(new int [2] {int(tester_ind_f[0]), int(tester_ind_f[1])});
    neighbour_is_occupied = (field.get_val(tester_ind) == 1);

    // If any of the aboves is true, we have to change new_grad:
    if (grad_is_zero || neighbour_is_occupied)
    {
      for (auto &direction: search_directions_8)
      {
        neighbour_ind = new_ind + direction;
        if (field.is_valid_index(neighbour_ind))
          if ((field.get_val(neighbour_ind) < 0) &&
            (field.get_val(neighbour_ind) > field.get_val(new_ind)))
          {
            new_grad[0] = direction.get_x();
            new_grad[1] = direction.get_y();
            break;
          }
      }

      new_pix->normalize_grad();
    }
  }

  
} // namespace gradplanner