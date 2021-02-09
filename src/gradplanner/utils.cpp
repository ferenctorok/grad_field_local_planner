#include <gradplanner/utils.h>

#include <cmath>
using namespace std;

namespace gradplanner
{
  double get_length(double v[2])
  {
    return sqrt(pow(v[0], 2) + pow(v[1], 2));
  }

  double get_ang_diff(const double real, const double desired)
  {
    double ang_diff = real - desired;
    if (abs(ang_diff) > PI)
      if (ang_diff > 0) 
        ang_diff -= 2 * PI;
      else
        ang_diff += 2 * PI;
    
    return ang_diff;
  }
}