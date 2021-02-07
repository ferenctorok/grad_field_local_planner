#include <cxxtest/TestSuite.h>
#include <cmath>
#include <gradplanner/repulsive_field.h>



/**
 * @brief Checks whether every values in the two matrices are equal.
 * @param a martix 1 to compare.
 * @param b matrix 2 to compare.
 */
void check_values_in_matrix(vector<vector<int >>* a,
                            vector<vector<int >>* b)
{
  int size_x = a->size();
  int size_y = (*a)[0].size();

  for (int i = 0; i < size_x; i ++)
    for (int j = 0; j < size_y; j ++)
      TS_ASSERT_EQUALS((*a)[i][j], (*b)[i][j]);
}

/**
 * @brief Checks whether the 2 gradients are equal.
 * @param grad1 gradient1
 * @param b gradient2
 */
void compare_gradients(double* grad1, double* grad2)
{
  TS_ASSERT_DELTA(grad1[0], grad2[0], 1e-4);
  TS_ASSERT_DELTA(grad1[1], grad2[1], 1e-4);
}

/**
 * @brief Tests the RepulsiveField class.
 */
class RepulsiveFieldTests : public CxxTest::TestSuite
{
  public:
    /**
     * @brief Sets up the tests.
     */
    void setUp()
    {
      size_x = 10;
      size_y = 12;
      R = 2;

      occ_grid.resize(size_x);
      for (int i = 0; i < size_x; i ++)
      {
        occ_grid[i].resize(size_y);
        for (int j = 0; j < size_y; j ++)
          occ_grid[i][j] = false;
      }

      // adding frame obstacle to it.
      for (int i = 0; i < size_x; i ++)
      {
        occ_grid[i][0] = true;
        occ_grid[i][size_y - 1] = true;
      }

      for (int j = 0; j < size_y; j ++)
      {
        occ_grid[0][j] = true;
        occ_grid[size_x - 1][j] = true;
      }

      // this should be the result after growing the repulsive field.
      res_etalon.resize(size_x);
      for (int i = 0; i < size_x; i ++)
      {
        res_etalon[i].resize(size_y);
        for (int j = 0; j < size_y; j ++)
          res_etalon[i][j] = 0;
      }

      // 1:
      for (int i = 0; i < size_x; i ++)
      {
        res_etalon[i][0] = 1;
        res_etalon[i][size_y - 1] = 1;
      }

      for (int j = 0; j < size_y; j ++)
      {
        res_etalon[0][j] = 1;
        res_etalon[size_x - 1][j] = 1;
      }

      // 2:
      for (int i = 1; i < (size_x - 1); i ++)
      {
        res_etalon[i][1] = 2;
        res_etalon[i][size_y - 2] = 2;
      }

      for (int j = 1; j < (size_y - 1); j ++)
      {
        res_etalon[1][j] = 2;
        res_etalon[size_x - 2][j] = 2;
      }

      // 3:
      for (int i = 2; i < (size_x - 2); i ++)
      {
        res_etalon[i][2] = 3;
        res_etalon[i][size_y - 3] = 3;
      }

      for (int j = 2; j < (size_y - 2); j ++)
      {
        res_etalon[2][j] = 3;
        res_etalon[size_x - 3][j] = 3;
      }
    }

    /**
     * @brief Tests the constructor of the RepulsiveField class.
     */
    void test_constructor()
    {
      rf = gradplanner::RepulsiveField(&occ_grid, R);
      TS_ASSERT_EQUALS(R, rf.get_R());
      TS_ASSERT_EQUALS(size_x, rf.get_size_x());
      TS_ASSERT_EQUALS(size_y, rf.get_size_y());

      gradplanner::RepulsiveField rf2(rf);
      TS_ASSERT_EQUALS(R, rf2.get_R());
      TS_ASSERT_EQUALS(size_x, rf2.get_size_x());
      TS_ASSERT_EQUALS(size_y, rf2.get_size_y());
    }

    /**
     * @brief Tests the update_field() method.
     */
    void test_update_field()
    {
      rf = gradplanner::RepulsiveField(&occ_grid, R);
      rf.update_field();

      // checking whether the values are match with the expected values:
      vector<vector<int >> values = rf.get_values();
      check_values_in_matrix(&res_etalon, &values);

      // check some gradients:
      vector<vector<double *>> grads = rf.get_grads();

      compare_gradients(grads[1][5], new double [2] {1, 0});
      compare_gradients(grads[5][1], new double [2] {0, 1});
      compare_gradients(grads[8][5], new double [2] {-1, 0});
      compare_gradients(grads[5][10], new double [2] {0, -1});

      compare_gradients(grads[1][1], new double [2] {std::sqrt(2) / 2, std::sqrt(2) / 2});
      compare_gradients(grads[1][10], new double [2] {std::sqrt(2) / 2, -std::sqrt(2) / 2});
      compare_gradients(grads[7][2], new double [2] {-std::sqrt(2) / 2, std::sqrt(2) / 2});
      compare_gradients(grads[7][9], new double [2] {-std::sqrt(2) / 2, -std::sqrt(2) / 2});

      for (int i = 0; i < size_x; i ++)
        for (int j = 0; j < size_y; j ++)
          if (values[i][j] == 0)
            compare_gradients(grads[i][j], new double [2] {0, 0});
      
      // placing an other obsticle in the occupancy grid:
      occ_grid[7][8] = true;
      rf.update_field();

      // check some values:
      values = rf.get_values();
      TS_ASSERT_EQUALS(2, values[8][8]);
      TS_ASSERT_EQUALS(2, values[7][7]);
      TS_ASSERT_EQUALS(2, values[6][8]);
      TS_ASSERT_EQUALS(2, values[7][9]);
      TS_ASSERT_EQUALS(2, values[7][10]);

      TS_ASSERT_EQUALS(3, values[7][6]);
      TS_ASSERT_EQUALS(3, values[5][8]);
      TS_ASSERT_EQUALS(3, values[5][6]);

      TS_ASSERT_EQUALS(0, values[4][4]);

      // checking some grads:

      compare_gradients(grads[8][8], new double [2] {0, 0});
      compare_gradients(grads[7][7], new double [2] {0, -1});
      compare_gradients(grads[6][8], new double [2] {-1, 0});
      compare_gradients(grads[7][9], new double [2] {0, 1});

      for (int i = 0; i < size_x; i ++)
        for (int j = 0; j < size_y; j ++)
          if (values[i][j] == 0)
            compare_gradients(grads[i][j], new double [2] {0, 0});
    }

  private:
    unsigned int size_x, size_y;
    unsigned int R;
    vector<vector<bool >> occ_grid;
    vector<vector<int >> res_etalon;
    gradplanner::RepulsiveField rf;
};