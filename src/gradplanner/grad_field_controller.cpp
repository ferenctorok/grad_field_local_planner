#include <gradplanner/grad_field_controller.h>
#include <gradplanner/utils.h>

#include <cmath>

using namespace std;


namespace gradplanner
{
  GradFieldController::GradFieldController(const GradFieldController& other):
    occ_grid_attr(other.occ_grid_attr), occ_grid_rep(other.occ_grid_rep),
    params(other.params), goal_is_valid(other.goal_is_valid),
    attractor(other.attractor), repulsive(other.repulsive),
    pose(other.pose), pose_old(other.pose_old), goal(other.goal)
  {
    goal_pos_rel[0] = other.goal_pos_rel[0];
    goal_pos_rel[1] = other.goal_pos_rel[1];
  }

  GradFieldController::GradFieldController(vector<vector<bool >>* occ_grid_attr,
                                           vector<vector<bool >>* occ_grid_rep,
                                           ControlParams* params):
    occ_grid_attr(occ_grid_attr), occ_grid_rep(occ_grid_rep),
    params(params), goal_is_valid(false),
    attractor(occ_grid_attr), repulsive(occ_grid_rep, params->general.R) {}

  GradFieldController& GradFieldController::operator=(const GradFieldController& other)
  {
    occ_grid_attr = other.occ_grid_attr;
    occ_grid_rep = other.occ_grid_rep;
    params = other.params;
    goal_is_valid = other.goal_is_valid;
    attractor = other.attractor;
    repulsive = other.repulsive;
    pose = other.pose;
    pose_old = other.pose_old;
    goal = other.goal;
    goal_pos_rel[0] = other.goal_pos_rel[0];
    goal_pos_rel[1] = other.goal_pos_rel[1];
  }

  void GradFieldController::set_pose(const Pose& pose)
  {
    this->pose = pose;
  }

  bool GradFieldController::set_new_goal(const Pose& goal)
  {
    this->goal = goal;

    // calculating the relative position of the goal, that is
    // the position of the goal in the potential field.
    goal_pos_rel[0] = goal.x - pose.x + attractor.get_size_x() / 2;
    goal_pos_rel[1] = goal.y - pose.y + attractor.get_size_y() / 2;

    goal_is_valid = attractor.set_new_goal(goal_pos_rel);
    return goal_is_valid;
  }

  bool GradFieldController::get_cmd_vel()
  {
    if (goal_is_valid && robot_is_free())
    {

      if (goal_is_reachable())
      {
        return true;
      }
    }

    return false;
  }

  bool GradFieldController::robot_is_free()
  {
    // the robot is always in the middle of the occupancy grid, 
    // which hence must have odd size in both directions.
    unsigned int x = (attractor.get_size_x() - 1) / 2;
    unsigned int y = (attractor.get_size_y() - 1) / 2;
    return ! (*occ_grid_rep)[x][y];
  }

  bool GradFieldController::goal_is_reachable()
  {
    // the robot is always in the middle of the occupancy grid, 
    // which hence must have odd size in both directions.
    unsigned int x = (attractor.get_size_x() - 1) / 2;
    unsigned int y = (attractor.get_size_y() - 1) / 2;
    return (attractor.get_val(x, y) < 0);
  }
} // namespace gradplanner
