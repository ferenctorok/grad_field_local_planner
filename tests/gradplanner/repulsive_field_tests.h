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

      // creating the occuancy grid:
      for (int i = 0; i < size_x; i ++)
      {
        occ_grid.push_back(vector<bool > ());
        for (int j = 0; j < size_y; j ++)
          occ_grid[i].push_back(false);
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
     * @brief Testing the contstructor.
     */
    void test_constructor()
    {
      TS_ASSERT_EQUALS(2, 2);
    }

  private:
    uint size_x, size_y;
    vector<vector<bool >> occ_grid;
};