#ifndef FIELD_UTILS_H
#define FIELD_UTILS_H

#include <vector>
using namespace std;


namespace gradplanner
{
  /**
   * @class Pixel class that implements the functionalities of a gridpoint in a gradient field. 
   */
  class Pixel
  {
    public:
      /**
       * @brief Default constructor of the Pixel class.
       */
      Pixel();

      /**
       * @brief Contstructor of the Pixel class.
       * @param x The x position of the pixel
       * @param y The y position of the pixel
       * @param value The value of the pixel
       */
      Pixel(unsigned int x, unsigned int y, int value=0);

      /**
       * @brief Contstructor of the Pixel class with gradient.
       * @param x The x position of the pixel
       * @param y The y position of the pixel
       * @param value The value of the pixel
       * @param grad The gradient of the pixel
       */
      Pixel(unsigned int x, unsigned int y, int value, double grad[2]);

      /**
       * @brief Contstructor of the Pixel class with parent.
       * @param x The x position of the pixel
       * @param y The y position of the pixel
       * @param value The value of the pixel
       * @param grad The gradient of the pixel
       * @param parent The parent of the pixel. It has got a meaning if the pixel is part of a repulsive
       * field, in which case it is important to know, which obstacle pixel has got an influence on that pixel.
       */
      Pixel(unsigned int x, unsigned int y, int value, double grad[2], unsigned int parent[2]);

      /**
       * @brief Destructor of the Pixel class.
       */
      ~Pixel() {}

      /**
       * @brief Get the x member of a Pixel.
       */
      unsigned int get_x();

      /**
       * @brief Get the y member of a Pixel.
       */
      unsigned int get_y();

      /**
       * @brief Get the value member of a Pixel.
       */
      int get_val();

      /**
       * @brief Sets the value of the Pixel.
       * @param value The value to be set.
       */
      void set_val(int value);

      /**
       * @brief Get the gradient member of a Pixel.
       */
      double* get_grad();

      /**
       * @brief Sets the gradient of the Pixel.
       * @param grad The gradient to be set.
       */
      void set_grad(double grad[2]);

      /**
       * @brief Get the gradient member of a Pixel.
       */
      unsigned int* get_parent();

      /**
       * @brief Sets the parent of the Pixel.
       * @param parent The parent to be set.
       */
      void set_parent(unsigned int parent[2]);

      /**
       * @brief Normalizes the gradient of the Pixel.
       */
      void normalize_grad();

      /**
       * @brief Scales the gradient of the Pixel to a specified length.
       * @param L The length the gradient has to be scaled.
       */
      void scale_grad(double L=1.0);

    protected:
      unsigned int x, y;
      int value;
      double grad[2];
      unsigned int parent[2];
  };

  /**
   * @class Field class that is a 2D container for Pixels. 
   */
  class Field
  {
    public:
      /**
       * @brief Default constructor of the Field class.
       */
      Field() {}

      /**
       * @brief Constructor of the Field class.
       * @param N size of the field in the x direction. (first coordinate.)
       * @param M size of the field in the y direction. (second coordinate.)
       */
      Field(unsigned int N, unsigned int M);

      /**
       * @brief Default destructor of the Field class.
       */
      ~Field() {}

      /**
       * @brief Returns the shape of the field as an array.
       */
      unsigned int* get_shape();

      /**
       * @brief Returns the Pixel pointer at the given index. 
       * @param x x direction.
       * @param y y direction.
       */
      Pixel* get_pix(unsigned int x, unsigned int y);

      /**
       * @brief Get the value of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       */
      int get_val(unsigned int x, unsigned int y);

      /**
       * @brief Set the value of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @param val The value to be set.
       */
      void set_val(unsigned int x, unsigned int y, int val);

      /**
       * @brief Get the gradient of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       */
      double* get_grad(unsigned int x, unsigned int y);

      /**
       * @brief Set the gradient of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @param grad The value to be set.
       */
      void set_grad(unsigned int x, unsigned int y, double grad[2]);

      /**
       * @brief Get the gradient of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       */
      unsigned int* get_parent(unsigned int x, unsigned int y);

      /**
       * @brief Set the gradient of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @param parent The value to be set.
       */
      void set_parent(unsigned int x, unsigned int y, unsigned int parent[2]);

    private:
      unsigned int N;
      unsigned int M;
      vector<vector<Pixel* >> data;
      vector<int > data2;
  };
} // namespace gradplanner

#endif // FIELD_UTILS_H