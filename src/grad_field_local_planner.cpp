# include <grad_field_local_planner/grad_field_local_planner.h>

#include <pluginlib/class_list_macros.h>

// Exporting the class as a plugin
PLUGINLIB_EXPORT_CLASS(grad_field_local_planner::GradFieldPlannerROS,
                       nav_core::BaseLocalPlanner)


namespace grad_field_local_planner
{
  GradFieldPlannerROS::GradFieldPlannerROS()
  {

  }


  GradFieldPlannerROS::~GradFieldPlannerROS()
  {

  }


  void GradFieldPlannerROS::initialize(std::string name,
                                  tf2_ros::Buffer* tf,
                                  costmap_2d::Costmap2DROS* costmap_ros)
  {

  }


  bool GradFieldPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
  {
    return true;
  }


  bool GradFieldPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    return true;
  }


  bool GradFieldPlannerROS::isGoalReached()
  {
    return true;
  }
} // namespace grad_field_local_planner
