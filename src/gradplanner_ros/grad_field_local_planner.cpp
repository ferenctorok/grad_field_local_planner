/* Gradplanner includes */
# include <gradplanner_ros/grad_field_local_planner.h>

/* ROS includes */
#include <pluginlib/class_list_macros.h>

/* std includes */
#include <queue>

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
      lay_costmap = costmap_ros->getLayeredCostmap();
      layers = lay_costmap->getPlugins();
      tf_buffer = tf_buffer_;

      // setting up some params mostly based on the parameter server:
      getParams();

      // create ROS node handler:
      ros::NodeHandle nh;

      // subscribers and publishers:
      amcl_sub = nh.subscribe("amcl_pose", 100,
        &GradFieldPlannerROS::amclCallback, this);

      // initializing the occupancy grids:
      initOccGrid(size_x_attr, size_y_attr, occ_grid_attr);
      initOccGrid(size_x_rep, size_y_rep, occ_grid_rep);

      occ_grid_attr.resize(size_x_attr);
      for(int i = 0; i < size_x_attr; i ++)
      {
        occ_grid_attr[i].resize(size_y_attr);
        for(int j = 0; j < size_x_attr; j ++)
          occ_grid_attr[i][j] = false;
      }
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

    int mx, my;
    costmap->worldToMapEnforceBounds(state.x, state.y, mx, my);
    ROS_INFO_STREAM("attractor_size: " << size_x_attr);
    ROS_INFO_STREAM("mx: " << mx);
    ROS_INFO_STREAM("my: " << my);
    ROS_INFO_STREAM("val origin: " << costmap->getCost(29, 29));

    ROS_INFO_STREAM("origin_x: " << costmap->getOriginX());
    ROS_INFO_STREAM("origin_y: " << costmap->getOriginY());
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


  void GradFieldPlannerROS::initOccGrid(const unsigned int size_x, const unsigned int size_y,
                                        vector<vector<bool >>& occ_grid)
  {
    occ_grid.resize(size_x);
      for(int i = 0; i < size_x; i ++)
      {
        occ_grid[i].resize(size_y);
        for(int j = 0; j < size_y; j ++)
          occ_grid[i][j] = false;
      }
  }


  void GradFieldPlannerROS::updateOccGrids()
  {
    queue<int* > q;

    // filling up the attractor field:
    for (int i = 0; i < size_x_attr; i ++)
      for (int j = 0; j < size_y_attr; j ++)
      {
        if (costmap->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
        {
          occ_grid_attr[i][j] == true;
          q.push(new int [2] {i, j});
        }
        else
          occ_grid_attr[i][j] == false;
      }

    // inflating the obstacles with the robot radius:
    int x_min, x_max, y_min, y_max;
    int* ind;
    while(! q.empty())
    {
      ind = q.front();
      q.pop();

      x_min = ind[0] - safety_R;
      if (x_min < 0) x_min = 0;
      x_max = ind[0] + safety_R;
      if (x_max >= size_x_attr) x_max = size_x_attr - 1;

      y_min = ind[1] - safety_R;
      if (y_min < 0) y_min = 0;
      y_max = ind[1] + safety_R;
      if (y_max >= size_y_attr) y_max = size_y_attr - 1;

      for (int i = x_min; i <= x_max; i ++)
        for (int j = y_min; j <= y_max; j ++)
          occ_grid_attr[i][j] = true;
    }

    // copying the relevant part to the repulsive occupancy grid:
    int x_diff = (size_x_attr - size_x_rep) / 2;
    int y_diff = (size_y_attr - size_y_rep) / 2;

    for (int i = 0; i < size_x_rep; i ++)
      for (int j = 0; j < size_y_rep; j ++)
        occ_grid_rep[i][j] = occ_grid_attr[i + x_diff][j + y_diff];
  }

} // namespace grad_field_local_planner
