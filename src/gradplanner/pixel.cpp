#include <gradplanner/utils.h>

#include <cmath>

using namespace std;

namespace gradplanner
{
  Pixel::Pixel():
  x(0), y(0), value(0), grad{0, 0}, parent{0, 0} {}


  Pixel::Pixel(unsigned int x, unsigned int y, int value):
  x(x), y(y), value(value), grad{0, 0}, parent{0, 0} {}


  Pixel::Pixel(unsigned int x, unsigned int y, int value, double grad[2]):
  x(x), y(y), value(value), grad{grad[0], grad[1]}, parent{0, 0} {}


  Pixel::Pixel(unsigned int x, unsigned int y, int value, double grad[2], unsigned int parent[2]):
  x(x), y(y), value(value), grad{grad[0], grad[1]}, parent{parent[0], parent[1]} {}


  unsigned int Pixel::get_x() const {return x;}
  unsigned int Pixel::get_y() const {return y;}
  int Pixel::get_val() const {return value;}
  const double* Pixel::get_grad() const {return grad;}
  const unsigned int* Pixel::get_parent() const {return parent;}
  void Pixel::set_val(int value) {this->value = value;}


  void Pixel::set_grad(double grad[2])
  {
    this->grad[0] = grad[0];
    this->grad[1] = grad[1];
  }

  
  void Pixel::set_parent(unsigned int parent[2])
  {
    this->parent[0] = parent[0];
    this->parent[1] = parent[1];
  }


  void Pixel::normalize_grad() {this->scale_grad();}


  void Pixel::scale_grad(double L)
  {
    double length = sqrt(pow(grad[0], 2) + pow(grad[1], 2));
    if (length < 1e-5) grad[0], grad[1] = 0.0, 0.0;
    else
    {
      grad[0] *= L / length;
      grad[1] *= L / length;
    }
  }
} // namespace gradplanner