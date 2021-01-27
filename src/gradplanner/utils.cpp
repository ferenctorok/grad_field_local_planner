#include <gradplanner/utils.h>

#include <cmath>
using namespace std;

namespace gradplanner
{
  double get_length(double v[2])
  {
    return sqrt(pow(v[0], 2) + pow(v[1], 2));
  }
}