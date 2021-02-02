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
       * @return the x position of the Pixel.
       */
      unsigned int get_x();

      /**
       * @brief Get the y member of a Pixel.
       * @return the y position of the Pixel.
       */
      unsigned int get_y();

      /**
       * @brief Get the value member of a Pixel.
       * @return the value of the Pixel.
       */
      int get_val();

      /**
       * @brief Sets the value of the Pixel.
       * @param value The value to be set.
       */
      void set_val(int value);

      /**
       * @brief Get the gradient member of a Pixel.
       * @return the gradient of the Pixel.
       */
      double* get_grad();

      /**
       * @brief Sets the gradient of the Pixel.
       * @param grad The gradient to be set.
       */
      void set_grad(double grad[2]);

      /**
       * @brief Get the gradient member of a Pixel.
       * @return the parent of the Pixel.
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
   * @class Index class, that enables convinient indexing of a Field and 
   * manipulation of the indices.
   */
  class Index
  {
    public:
      /**
       * @brief Default constructor of the Index class.
       */
      Index();

      /**
       * @brief Copy constructor.
       * @param other Other Index to copy.
       */
      Index(const Index& other);

      /**
       * @brief Constructor of the Index class.
       * @param shape Necessary, array containing the size of the field
       *              in the x and y directions.
       * @param ind 2 member pointer, the indices.
       */
      Index(unsigned int shape[2], int ind[2]=nullptr);

      /**
       * @brief Default destructor of the Index class.
       */
      ~Index() {}

      /**
       * @brief returns the x coordinate of the Index object.
       */
      int get_x();

      /**
       * @brief returns the y coordinate of the Index object.
       */
      int get_y();

      /**
       * @brief Copy assignment operator:
       * @param other Reference to another Index object to copy.
       */
      Index& operator=(Index other);

      /**
       * @brief Assignment operator overloading.
       * @param new_ind New indices to set for the object.
       */
      void operator=(int new_ind[2]);

      /**
       * @brief Addition operation overloading.
       * @param ind2 the other index which is added to it.
       */
      friend Index operator+(const Index& a, const Index& b);

      /**
       * @brief Returns true, if the index is valid, that is the index
       * points within the boundaries of the field.
       */
      bool is_valid();


    private:
      unsigned int* shape;  // size of the field.
      int* ind;  // The indices.
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
       * @brief Gets an indexer object for this field:
       * @return Indexer object with set shape that fits to this field.
       */
      Index get_indexer();

      /**
       * @brief Returns the shape of the field as an array.
       * @return the shape of the Field.
       */
      unsigned int* get_shape();

      /**
       * @brief Returns the Pixel pointer at the given index. 
       * @param x x direction.
       * @param y y direction.
       * @return The pointer to the Pixel object at the given position.
       */
      Pixel* get_pix(unsigned int x, unsigned int y);

      /**
       * @brief Returns the Pixel pointer at the given index. 
       * @param ind the Index object.
       * @return The pointer to the Pixel object at the given position.
       */
      Pixel* get_pix(Index ind);

      /**
       * @brief Get the value of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @return The value of the Pixel object at the given position.
       */
      int get_val(unsigned int x, unsigned int y);

      /**
       * @brief Returns the Pixel pointer at the given index. 
       * @param ind the Index object.
       * @return The value of the Pixel at that index.
       */
      int get_val(Index ind);

      /**
       * @brief Sets the value of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @param val The value to be set.
       */
      void set_val(unsigned int x, unsigned int y, int val);

      /**
       * @brief Sets the value of the pixel at the given place.
       * @param ind the Index object.
       * @param val the value to be set.
       */
      void set_val(Index ind, int val);

      /**
       * @brief Get the gradient of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @return The gradient of the Pixel object at the given position.
       */
      double* get_grad(unsigned int x, unsigned int y);

      /**
       * @brief Returns the Pixel pointer at the given index. 
       * @param ind the Index object.
       * @return The gradient of the Pixel at that index.
       */
      double* get_grad(Index ind);

      /**
       * @brief Sets the gradient of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @param grad The gradient to be set.
       */
      void set_grad(unsigned int x, unsigned int y, double grad[2]);

      /**
       * @brief Sets the value of the pixel at the given place.
       * @param ind the Index object.
       * @param grad The gradient to be set.
       */
      void set_grad(Index ind, double grad[2]);

      /**
       * @brief Get the gradient of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @return The parent of the Pixel object at the given position.
       */
      unsigned int* get_parent(unsigned int x, unsigned int y);

      /**
       * @brief Returns the Pixel pointer at the given index. 
       * @param ind the Index object.
       * @return The parent of the Pixel at that index.
       */
      unsigned int* get_parent(Index ind);

      /**
       * @brief Set the gradient of the pixel at the given place.
       * @param x The x coordinate of the pixel.
       * @param y The y coordinate of the pixel.
       * @param parent The parent to be set.
       */
      void set_parent(unsigned int x, unsigned int y, unsigned int parent[2]);

      /**
       * @brief Sets the value of the pixel at the given place.
       * @param ind the Index object.
       * @param parent The parent to be set.
       */
      void set_parent(Index ind, unsigned int parent[2]);

    private:
      unsigned int N;
      unsigned int M;
      vector<vector<Pixel* >> data;
  };


  
} // namespace gradplanner

#endif // FIELD_UTILS_H