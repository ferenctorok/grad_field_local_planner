#include <gradplanner/field_utils.h>
#include <assert.h>

using namespace std;

namespace gradplanner
{
  Index::Index(unsigned int shape[2], unsigned int ind[2]):
    shape(shape)
    {
      if (ind != nullptr) this->ind = ind;
      else this->ind = new unsigned int [2] {0, 0};
    }

  unsigned int Index::get_x() {return ind[0];}

  unsigned int Index::get_y() {return ind[1];}

  void Index::operator=(unsigned int new_ind[2])
    {ind = new_ind;}

  Index operator+(const Index& a, const Index& b)
  {
    // Asserting that the indices are indices of a field of same size:
    assert((a.shape[0] == b.shape[0]));
    assert((a.shape[1] == b.shape[1]));

    unsigned int x, y;
    x = a.ind[0] + b.ind[0];
    y = a.ind[1] + b.ind[1];
    return Index(a.shape, new unsigned int [2] {x, y});
  }

  bool Index::is_valid()
  {
    return ((ind[0] < shape[0]) && (ind[0] >= 0)) &&
           ((ind[1] < shape[1]) && (ind[1] >= 0));
  }
} // namespace gradplanner