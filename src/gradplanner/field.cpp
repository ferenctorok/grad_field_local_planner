#include <gradplanner/field_utils.h>

#include <vector>
#include <iostream>
using namespace std;


namespace gradplanner
{
  Field::Field(unsigned int N, unsigned int M)
  {
    this->N = N;
    this->M = M;
    this->shape[0] = N;
    this->shape[1] = M;

    // Initializing the data:
    data.resize(N);
    for (int i = 0; i < N; i++)
      {
        data[i].resize(M);
        for (int j = 0; j < M; j++)
          data[i][j] = Pixel(i, j);
      }
  }

  Field::Field(const Field& other):
  N(other.N), M(other.M)
  {
    this->shape[0] = other.shape[0];
    this->shape[1] = other.shape[1];

    // copying the data:
    this->data.resize(this->N);
    for (int i = 0; i < this->N; i++)
      {
        this->data[i].resize(this->M);
        for (int j = 0; j < this->M; j++)
          this->data[i][j] = other.data[i][j];
      }
  }

  Field& Field::operator=(const Field& other)
  {
    this->N = other.N;
    this->M = other.M;
    this->shape[0] = other.shape[0];
    this->shape[1] = other.shape[1];

    // copying the data:
    this->data.resize(this->N);
    for (int i = 0; i < this->N; i++)
      {
        this->data[i].resize(this->M);
        for (int j = 0; j < this->M; j++)
          this->data[i][j] = other.data[i][j];
      }

    return *this;
  }

  bool Field::is_valid_index(Index ind)
  {
    return ((ind.get_x() >= 0) && (ind.get_x() < N) &&
            (ind.get_y() >= 0) && (ind.get_y() < M));
  }

  unsigned int* Field::get_shape()
    {return shape;}

  // get_pix()
  Pixel* Field::get_pix(unsigned int x, unsigned int y)
    {return &data[x][y];}

  Pixel* Field::get_pix(Index ind)
    {return &data[ind.get_x()][ind.get_y()];}


  // get_val()
  int Field::get_val(unsigned int x, unsigned int y)
    {return data[x][y].get_val();}

  int Field::get_val(Index ind)
    {return data[ind.get_x()][ind.get_y()].get_val();}


  // set_val()
  void Field::set_val(unsigned int x, unsigned int y, int val)
    {data[x][y].set_val(val);}

  void Field::set_val(Index ind, int val)
    {data[ind.get_x()][ind.get_y()].set_val(val);}


  // get_grad()
  double* Field::get_grad(unsigned int x, unsigned int y)
    {return data[x][y].get_grad();}

  double* Field::get_grad(Index ind)
    {return data[ind.get_x()][ind.get_y()].get_grad();}


  // set_grad()
  void Field::set_grad(unsigned int x, unsigned int y, double grad[2])
    {data[x][y].set_grad(grad);}

  void Field::set_grad(Index ind, double grad[2])
    {data[ind.get_x()][ind.get_y()].set_grad(grad);}


  // get_parent()
  unsigned int* Field::get_parent(unsigned int x, unsigned int y)
    {return data[x][y].get_parent();}

  unsigned int* Field::get_parent(Index ind)
    {return data[ind.get_x()][ind.get_y()].get_parent();}


  // set_parent()
  void Field::set_parent(unsigned int x, unsigned int y, unsigned int parent[2])
    {data[x][y].set_parent(parent);}

  void Field::set_parent(Index ind, unsigned int parent[2])
    {data[ind.get_x()][ind.get_y()].set_parent(parent);}
} // namespace gradplanner