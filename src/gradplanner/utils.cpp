#include <gradplanner/utils.h>

#include <cmath>
using namespace std;

namespace gradplanner
{
  double get_length(double v[2])
  {
    return sqrt(pow(v[0], 2) + pow(v[1], 2));
  }

  template <typename T>
  int sgn(const T& val)
  {
    if (val > 0) return 1;
    else if (val == 0) return 0;
    else return -1;
  }

  double get_ang_diff(const double real, const double desired)
  {
    double ang_diff = real - desired;
    if (abs(ang_diff) > PI)
      ang_diff -= sgn<double >(ang_diff) * 2 * PI;
    
    return ang_diff;
  }
}