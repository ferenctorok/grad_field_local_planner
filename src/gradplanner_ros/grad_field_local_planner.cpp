/* Gradplanner includes */
# include <gradplanner_ros/grad_field_local_planner.h>

/* ROS includes */
#include <pluginlib/class_list_macros.h>

/* std includes */
#include <queue>
#include <cmath>

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

      ROS_INFO_STREAM("Costmap name: ");
      // create ROS node handler:
      ros::NodeHandle nh;

      // subscribers and publishers:
      amcl_sub = nh.subscribe("amcl_pose", 100,
        &GradFieldPlannerROS::amclCallback, this);

      // initializing the occupancy grids:
      initOccGrid(size_x_attr, size_y_attr, occ_grid_attr);
      initOccGrid(size_x_rep, size_y_rep, occ_grid_rep);

      // initializing the controller:
      controller = gradplanner::GradFieldController(&occ_grid_attr,
                                                    &occ_grid_rep,
                                                    &params);

      initialized = true;
    }      
  }


  bool GradFieldPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
  {
    return true;
  }


  bool GradFieldPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if (initialized)
    {
      // updating the state and the origin of the attractor field:
      geometry_msgs::PoseStamped rob_pose;
      costmap_ros->getRobotPose(rob_pose);
      state.x = rob_pose.pose.position.x;
      state.y = rob_pose.pose.position.y;
      state.psi = 0;

      origin_x_attr = costmap->getOriginX();
      origin_y_attr = costmap->getOriginX();

      ROS_INFO_STREAM("x: " << state.x);
      ROS_INFO_STREAM("y: " << state.y);
      ROS_INFO_STREAM("psi: " << state.psi);

      ROS_INFO_STREAM("origin_x: " << origin_x_attr);
      ROS_INFO_STREAM("origin_y: " << origin_y_attr);



      // updating the occupancy grids based on the actual costmap from ROS:
      updateOccGrids();

      // updating the controller state with the most recent available:
      if (controller.set_state(state, origin_x_attr, origin_y_attr))
      {
        // calculating the new goal:
        goal_attr.x = state.x + 1.5;
        goal_attr.y = state.y;
        goal_attr.psi = 0;

        // setting the new goal to the controller and control:
        if (controller.set_new_goal(goal_attr, origin_x_attr, origin_y_attr))
        {
          double v_x, omega;
          if (controller.get_cmd_vel(v_x, omega))
          {
            cmd_vel.linear.x = v_x;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = omega;
            ROS_INFO("Set velocity commands!");
            ROS_INFO_STREAM("v_x: " << v_x);
            ROS_INFO_STREAM("omega: " << omega);
          }
          else
          {
            ROS_WARN("Cmd_vel calculation was unsuccessful!");
            return false;
          }
        }
        else
        {
          ROS_WARN("Setting the goal was unsuccessful!");
          ROS_INFO_STREAM("robot is free: " << controller.robot_is_free());
          ROS_INFO_STREAM("goal is reachable: " << controller.goal_is_reachable());
          return false;
        }
      }
      else
      {
        ROS_WARN("Setting the state to the controller was unsuccessful!");

        ROS_INFO_STREAM("x: " << state.x);
        ROS_INFO_STREAM("y: " << state.y);
        ROS_INFO_STREAM("psi: " << state.psi);

        ROS_INFO_STREAM("origin_x: " << costmap->getOriginX());
        ROS_INFO_STREAM("origin_y: " << costmap->getOriginY());

        return false;
      }
    }
    else
    {
      ROS_WARN("GradFieldPlannerROS is not initialized can not provide cmd_vel.");
      return false;
    }

    return true;
  }


  bool GradFieldPlannerROS::isGoalReached()
  {
    return false;
  }


  void GradFieldPlannerROS::amclCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    /*state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;
    state.psi = getYaw(msg);*/

    // getting the origin of thecostmap attractor field:
    // Unfortunatelly costmap->MapToWorld(0, 0, origin_x_attr, origin_y_attr)
    // does not always work, since for example when setting initial
    // position of the robot from rviz, the costmap is somehow not updated.
    // Hence after setting the initial condition the origin would
    // be placed incorrectly. This is why this complicated shit is needed.
    /*int orig_x = costmap->getOriginX(); // relative to the 
    int orig_y = costmap->getOriginY();

    double x_grid, y_grid;
    x_grid = floor(state.x / params.general.cell_size) * params.general.cell_size;
    y_grid = floor(state.y / params.general.cell_size) * params.general.cell_size;

    origin_x_attr = x_grid + orig_x * params.general.cell_size;
    origin_y_attr = y_grid + orig_y * params.general.cell_size;*/

    /*costmap->mapToWorld(0, 0, origin_x_attr, origin_y_attr);

    ROS_INFO("--- AMCL CALLBACK ---");
    ROS_INFO_STREAM("x: " << state.x);
    ROS_INFO_STREAM("y: " << state.y);
    ROS_INFO_STREAM("psi: " << state.psi);

    ROS_INFO_STREAM("origin_x: " << origin_x_attr);
    ROS_INFO_STREAM("origin_y: " << origin_y_attr);

    ROS_INFO_STREAM("costmap_ros name: " << costmap_ros->getName());
    ROS_INFO_STREAM("local frame id: " << costmap_ros->getBaseFrameID());
    ROS_INFO_STREAM("global frame id: " << costmap_ros->getGlobalFrameID());*/
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
        if (costmap->getCost(i, j) != costmap_2d::FREE_SPACE)
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

    // getting the actual origin of the attractor field:
    costmap->mapToWorld(0, 0, origin_x_attr, origin_y_attr);
  }
} // namespace grad_field_local_planner
