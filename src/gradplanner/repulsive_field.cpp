#include <gradplanner/repulsive_field.h>


namespace gradplanner
{
  RepulsiveField::RepulsiveField(costmap_2d::Costmap2DROS* costmap,
                                 unsigned int R):
    GradFieldBase(costmap)
  {
    this->R = R;
    init_field();
  }


  void RepulsiveField::init_field()
  {
    field = Field(2 * R + 1, 2 * R + 1);
  }
}