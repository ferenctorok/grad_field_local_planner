#include <iostream>
#include <chrono>
using namespace std::chrono;

/* Gradplanner inlcudes */
#include <gradplanner/attractor_field.h>
#include <gradplanner/repulsive_field.h>


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

  return 0;
}