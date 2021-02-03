#include <cxxtest/TestSuite.h>

#include <gradplanner/repulsive_field.h>


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
      R = 4;

      occ_grid.resize(size_x);
      for (int i = 0; i < size_x; i ++)
      {
        occ_grid[i].resize(size_y);
        for (int j = 0; j < size_y; j ++)
          occ_grid[i][j] = false;
      }

      // adding frame and an obstacle to it.
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

      occ_grid[5][5] = true;
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
    }

    /**
     * @brief Tests the re_init_field() method.
     */
    void test_re_init_field()
    {
      TS_ASSERT_EQUALS(2, 2);
    }

    /**
     * @brief Tests the update_field() method.
     */
    void test_update_field()
    {
      rf = gradplanner::RepulsiveField(&occ_grid, R);
      rf.update_field();
    }

  private:
    unsigned int size_x, size_y;
    unsigned int R;
    vector<vector<bool >> occ_grid;
    gradplanner::RepulsiveField rf;
};