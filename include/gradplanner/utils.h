#ifndef UTILS_H
#define UTILS_H


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
      unsigned int get_x() const;

      /**
       * @brief Get the y member of a Pixel.
       */
      unsigned int get_y() const;

      /**
       * @brief Get the value member of a Pixel.
       */
      int get_val() const;

      /**
       * @brief Sets the value of the Pixel.
       * @param value The value to be set.
       */
      void set_val(int value);

      /**
       * @brief Get the gradient member of a Pixel.
       */
      const double* get_grad() const;

      /**
       * @brief Sets the gradient of the Pixel.
       * @param grad The gradient to be set.
       */
      void set_grad(double grad[2]);

      /**
       * @brief Get the gradient member of a Pixel.
       */
      const unsigned int* get_parent() const;

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
} // namespace gradplanner

#endif // UTILS_H