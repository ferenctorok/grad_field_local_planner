#include <gradplanner/attractor_field.h>


namespace gradplanner
{
  AttractorField::AttractorField(const AttractorField& other):
    GradFieldBase(other)
    {
      goal[0] = other.goal[0];
      goal[1] = other.goal[1];
      goal_ind = other.goal_ind;
    }

  AttractorField::AttractorField(vector<vector<bool >>* occ_grid):
    GradFieldBase(occ_grid), goal{0.0, 0.0}, goal_ind(new int [2] {0, 0}) {}

  AttractorField& AttractorField::operator=(const AttractorField& other)
  {
    this->GradFieldBase::operator=(other);
    goal[0] = other.goal[0];
    goal[1] = other.goal[1];
    goal_ind = other.goal_ind;

    return *this;
  }

  bool AttractorField::set_new_goal(double new_goal[2])
  {
    goal[0] = new_goal[0];
    goal[1] = new_goal[1];
    goal_ind = Index(new int [2] {int(goal[0]), int(goal[1])});

    return field.is_valid_index(goal_ind);
  }

  void AttractorField::update_field()
  {
    

  }

  
} // namespace gradplanner