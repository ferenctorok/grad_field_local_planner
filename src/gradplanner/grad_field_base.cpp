#include <gradplanner/grad_field_base.h>


namespace gradplanner
{
  GradFieldBase::GradFieldBase(costmap_2d::Costmap2DROS* costmap):
    costmap(costmap) {}
} //namespace gradplanner