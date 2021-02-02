#include <gradplanner/repulsive_field.h>
#include <iostream>


namespace gradplanner
{
  RepulsiveField::RepulsiveField(vector<vector<bool >>* occ_grid,
                                 unsigned int R):
    GradFieldBase(occ_grid), R(R) {}

  unsigned int RepulsiveField::get_R() {return R;}


  void RepulsiveField::update_field()
  {
    // re-initialize the field:
    re_init_field();

    // Creating and intializing the queue.
    queue<unsigned int* > q;
    init_queue(q);

    while (!q.empty())
    {
      
    }
  }


  void RepulsiveField::init_queue(queue<unsigned int* >& q)
  {
    for (unsigned int i = 0; i < size_x; i ++)
    {
      for (unsigned int j = 0; j < size_y; j ++)
        if ((*occ_grid)[i][j])
          q.push(new unsigned int [2] {i, j});
    }
  }
}