# include <gradplanner_ros/grad_field_local_planner.h>

#include <pluginlib/class_list_macros.h>

// Exporting the class as a plugin
PLUGINLIB_EXPORT_CLASS(grad_field_local_planner::GradFieldPlannerROS,
                       nav_core::BaseLocalPlanner)


namespace grad_field_local_planner
{
  GradFieldPlannerROS::GradFieldPlannerROS():
  costmap_ros(NULL), tf_buffer(NULL), initialized(false) {}


  GradFieldPlannerROS::GradFieldPlannerROS(std::string name,
                                           tf2_ros::Buffer* tf_buffer,
                                           costmap_2d::Costmap2DROS* costmap_ros):
  costmap_ros(NULL), tf_buffer(NULL), initialized(false)
  {
    initialize(name, tf_buffer, costmap_ros);
  }


  void GradFieldPlannerROS::initialize(std::string name,
                                       tf2_ros::Buffer* tf_buffer_,
                                       costmap_2d::Costmap2DROS* costmap_ros_)
  {
    if (! initialized)
    {
      costmap_ros = costmap_ros_;
      tf_buffer = tf_buffer_;

      // create ROS nodehandler and create subscribers and publishers:
      ros::NodeHandle nh;
      amcl_sub = nh.subscribe("amcl_pose", 100, &GradFieldPlannerROS::amclCallback, this);


      initialized = true;
    }      
    
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


  void GradFieldPlannerROS::setup_from_param_sever()
  {

  }


  void GradFieldPlannerROS::amclCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;
    state.psi = msg->pose.pose.orientation.w;
  }

} // namespace grad_field_local_planner
