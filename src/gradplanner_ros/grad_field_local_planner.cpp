# include <gradplanner_ros/grad_field_local_planner.h>

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
    ROS_INFO("--- PUBLISHED CMD_VEL ---");
    cmd_vel.linear.x = 1.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    return true;
  }


  bool GradFieldPlannerROS::isGoalReached()
  {
    return false;
  }
} // namespace grad_field_local_planner
