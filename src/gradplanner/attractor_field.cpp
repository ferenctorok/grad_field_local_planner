#include <gradplanner/attractor_field.h>


namespace gradplanner
{
  AttractorField::AttractorField(const AttractorField& other):
    GradFieldBase(other) {}

  AttractorField::AttractorField(vector<vector<bool >>* occ_grid):
    GradFieldBase(occ_grid) {}

  AttractorField& AttractorField::operator=(const AttractorField& other)
  {
    this->GradFieldBase::operator=(other);
  }

  

  
} // namespace gradplanner