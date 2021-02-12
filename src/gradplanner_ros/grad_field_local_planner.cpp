# include <gradplanner_ros/grad_field_local_planner.h>

#include <pluginlib/class_list_macros.h>

using namespace std;

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
      costmap = costmap_ros->getCostmap();
      tf_buffer = tf_buffer_;

      // setting up some params mostly based on the parameter server:
      getParams();

      // create ROS node handler:
      ros::NodeHandle nh;

      // subscribers and publishers:
      amcl_sub = nh.subscribe("amcl_pose", 100,
        &GradFieldPlannerROS::amclCallback, this);

      costmap_2d::NO_INFORMATION;
      initialized = true;
    }      
    
  }


  bool GradFieldPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
  {
    return true;
  }


  bool GradFieldPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    ROS_INFO("--- PUBLISHED CMD_VEL ---\n");
    cmd_vel.linear.x = 0.0;
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


  void GradFieldPlannerROS::amclCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;
    state.psi = getYaw(msg);

    ROS_INFO_STREAM("x: " << state.x);
    ROS_INFO_STREAM("y: " << state.y);
    ROS_INFO_STREAM("psi: " << state.psi);

    unsigned int mx, my;
    costmap->worldToMap(state.x, state.y, mx, my);
    ROS_INFO_STREAM("attractor_size: " << size_x_attr);
    ROS_INFO_STREAM("mx: " << mx);
    ROS_INFO_STREAM("my: " << my);
  }


  double GradFieldPlannerROS::getYaw(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
	{

		double q[4];
		q[0]= msg->pose.pose.orientation.x;
		q[1]= msg->pose.pose.orientation.y;
		q[2]= msg->pose.pose.orientation.z;
		q[3]= msg->pose.pose.orientation.w;

		double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
		double t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);  

		return std::atan2(t3, t4);

	}

} // namespace grad_field_local_planner
