#include <gradplanner/field_utils.h>
#include <assert.h>

using namespace std;

namespace gradplanner
{
  Index::Index():
    ind{0, 0} {}

  Index::Index(const Index& other)
  {
    ind[0] = other.ind[0];
    ind[1] = other.ind[1];
  }

  Index::Index(int ind[2])
    {
      if (ind != nullptr)
      {
        this->ind[0] = ind[0];
        this->ind[1] = ind[1];
      }
    }

  int Index::get_x() {return ind[0];}

  int Index::get_y() {return ind[1];}

  Index& Index::operator=(int new_ind[2])
  {
    this->ind[0] = new_ind[0];
    this->ind[1] = new_ind[1];

    return *this;
  }

  Index& Index::operator=(const Index& other)
  {
    ind[0] = other.ind[0];
    ind[1] = other.ind[1];
    return *this;
  }

  Index operator+(const Index& a, const Index& b)
  {
    int x, y;
    x = a.ind[0] + b.ind[0];
    y = a.ind[1] + b.ind[1];
    return Index(new int [2] {x, y});
  }
} // namespace gradplanner