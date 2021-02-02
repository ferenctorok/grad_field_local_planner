#include <gradplanner/field_utils.h>
#include <assert.h>

using namespace std;

namespace gradplanner
{
  Index::Index()
  {
    this->shape = new unsigned int [2] {0, 0};
    this->ind = new int [2] {0, 0};
  }

  Index::Index(const Index& other):
  shape(shape)
  {
    this->ind = new int [2];
    this->ind[0] = other.ind[0];
    this->ind[1] = other.ind[1];
  }

  Index::Index(unsigned int shape[2], int ind[2]):
    shape(shape)
    {
      this->ind = new int [2] {0, 0};
      if (ind != nullptr)
      {
        this->ind[0] = ind[0];
        this->ind[1] = ind[1];
      }
    }

  int Index::get_x() {return ind[0];}

  int Index::get_y() {return ind[1];}

  void Index::operator=(int new_ind[2])
  {
    ind[0] = new_ind[0];
    ind[1] = new_ind[1];
  }

  Index& Index::operator=(Index other)
  {
    shape = other.shape;
    ind[0] = other.ind[0];
    ind[1] = other.ind[1];
    return *this;
  }

  Index operator+(const Index& a, const Index& b)
  {
    // Asserting that the indices are indices of a field of same size:
    assert((a.shape[0] == b.shape[0]));
    assert((a.shape[1] == b.shape[1]));

    int x, y;
    x = a.ind[0] + b.ind[0];
    y = a.ind[1] + b.ind[1];
    return Index(a.shape, new int [2] {x, y});
  }

  bool Index::is_valid()
  {
    return ((ind[0] < shape[0]) && (ind[0] >= 0)) &&
           ((ind[1] < shape[1]) && (ind[1] >= 0));
  }
} // namespace gradplanner