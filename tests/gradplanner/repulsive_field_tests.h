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
     * @brief Tests the constructor of the RepulsiveField class.
     */
    void test_constructor()
    {
      rf = gradplanner::RepulsiveField(&occ_grid, R);
      TS_ASSERT_EQUALS(R, rf.get_R());
      cout << "shape: " << occ_grid.size() / sizeof(occ_grid[0]) << endl;
      TS_ASSERT_EQUALS(size_x, rf.get_size_x());
      TS_ASSERT_EQUALS(size_y, rf.get_size_y());
    }

  private:
    unsigned int size_x, size_y;
    unsigned int R;
    vector<vector<bool >> occ_grid;
    gradplanner::RepulsiveField rf;
};