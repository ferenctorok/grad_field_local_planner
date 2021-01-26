#include <gradplanner/utils.h>


using namespace std;

namespace gradplanner
{
  Pixel::Pixel():
  x(0), y(0), value(0), grad({0, 0}), parent({0, 0}) {}


  Pixel::Pixel(unsigned int x, unsigned int y, int value):
  x(x), y(y), value(value), grad({0, 0}), parent({0, 0}) {}


  Pixel::Pixel(unsigned int x, unsigned int y, int value, double grad[2]):
  x(x), y(y), value(value), parent({0, 0})
  {
    this->grad[0] = grad[0];
    this->grad[1] = grad[1];
  }


  Pixel::Pixel(unsigned int x, unsigned int y, int value, double grad[2], unsigned int parent[2]):
  x(x), y(y), value(value)
  {
    this->grad[0] = grad[0];
    this->grad[1] = grad[1];
    this->parent[0] = parent[0];
    this->parent[1] = parent[1];
  }


  Pixel::~Pixel() {delete grad, parent;}


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




} // namespace gradplanner