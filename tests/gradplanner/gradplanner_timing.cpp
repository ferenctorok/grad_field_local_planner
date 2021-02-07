#include <iostream>
#include <chrono>
using namespace std::chrono;

/* Gradplanner inlcudes */
#include <gradplanner/attractor_field.h>
#include <gradplanner/repulsive_field.h>


/**
 * @brief Returns an occupancy grid to measure execution time
 * of the RepulsiveField.update_field() method. It is only written
 * so that the main() is not filled with the code of creating an 
 * occupancy grid with some obstacles.
 * @return The occupancy grid with some obstacles.
 */
vector<vector<bool >> get_occ_grid_rep();


int main()
{
  /* Timing the AttractorField.update_field() method. */
  int num_of_measurements = 100;
  unsigned int size_x = 64;
  unsigned int size_y = 64;
  double goal[2] {24.5, 37.3};
  vector<vector<bool >> occ_grid;

  occ_grid.resize(size_x);
  for (int i = 0; i < size_x; i ++)
      {
        occ_grid[i].resize(size_y);
        for (int j = 0; j < size_y; j ++)
          occ_grid[i][j] = false;
      }

  gradplanner::AttractorField af(&occ_grid);
  af.set_new_goal(goal);

  // timing:
  auto start_time = high_resolution_clock::now();
  auto end_time = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(end_time - start_time);
  double average_duration = 0;

  for (int i = 0; i < num_of_measurements; i ++)
  {
    start_time = high_resolution_clock::now();
    af.update_field();
    end_time = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end_time - start_time);
    average_duration += (duration.count() / num_of_measurements); 
  }

  std::cout << "--- AttractorField ---" << std::endl;
  std::cout << "Grid size: " << size_x << " x " << size_y << std::endl;
  std::cout << "Average update_field() execution time from " << num_of_measurements << " runs: ";
  std::cout << average_duration << " us" << std::endl;

  
  /* Timing the AttractorField.update_field() method. */
  occ_grid = get_occ_grid_rep();
  unsigned int R = 16;
  gradplanner::RepulsiveField rf(&occ_grid, R);

  // timing:
  for (int i = 0; i < num_of_measurements; i ++)
  {
    start_time = high_resolution_clock::now();
    rf.update_field();
    end_time = high_resolution_clock::now();
    duration = duration_cast<microseconds>(end_time - start_time);
    average_duration += (duration.count() / num_of_measurements); 
  }

  std::cout << "--- RepulsiveField ---" << std::endl;
  std::cout << "Grid size: " << size_x << " x " << size_y << std::endl;
  std::cout << "Radius: " << R << std::endl;
  std::cout << "Average update_field() execution time from " << num_of_measurements << " runs: ";
  std::cout << average_duration << " us" << std::endl;
  
  return 0;
}


vector<vector<bool >> get_occ_grid_rep()
{
  unsigned int size_x = 64;
  unsigned int size_y = 64;
  vector<vector<bool >> occ_grid;

  occ_grid.resize(size_x);
  for (int i = 0; i < size_x; i ++)
      {
        occ_grid[i].resize(size_y);
        for (int j = 0; j < size_y; j ++)
          occ_grid[i][j] = false;
      }

  // side walls:
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
  
  // obstacle 1:
  for (int i = 5; i < 13; i ++)
    for (int j = 5; j < 13; j ++)
      occ_grid[i][j] = true;

  // obstacle 2:
  for (int i = 8; i < 14; i ++)
    for (int j = 34; j < 40; j ++)
      occ_grid[i][j] = true;

  // obstacle 3:
  for (int i = 50; i < 52; i ++)
    for (int j = 3; j < 35; j ++)
      occ_grid[i][j] = true;

  // obstacle 4:
  for (int i = 14; i < 30; i ++)
    for (int j = 20; j < 30; j ++)
      occ_grid[i][j] = true;

  // obstacle 5:
  for (int i = 50; i < 60; i ++)
    for (int j = 40; j < 42; j ++)
      occ_grid[i][j] = true;

  return occ_grid;
}