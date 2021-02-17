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
      goal_is_reached = false;

      // setting up some params mostly based on the parameter server:
      getParams();

      // create ROS node handler:
      ros::NodeHandle nh;

      // initializing the occupancy grids:
      initOccGrid(size_x_attr, size_y_attr, occ_grid_attr);
      initOccGrid(size_x_rep, size_y_rep, occ_grid_rep);

      // initializing the controller:
      /*
      controller = gradplanner::GradFieldController(&occ_grid_attr,
                                                    &occ_grid_rep,
                                                    &params);
      */
      controller = gradplanner::GradFieldController(&occ_grid_attr,
                                                    &occ_grid_attr,
                                                    &params);

      initialized = true;
    }      
  }


  bool GradFieldPlannerROS::setPlan(const vector<geometry_msgs::PoseStamped>& plan)
  {
    if (! initialized)
    {
      ROS_ERROR("Could not set goal, because the planner has not been initialized.");
      return false;
    }
    // ROS_INFO("New plan has been set to GradFieldPlannerROS.");
    plan_ptr = &plan;
    return true;
  }


  bool GradFieldPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if (! initialized)
    {
      ROS_ERROR("GradFieldPlannerROS has not been initialized, so could not call computeVelocityCommands()");
      return false;
    }

    // updating the state and the origin of the attractor field:
    if (! getState())
      return false;

    // setting the actual goal based on the plan.
    if (! getGoal())
      return false;
    
    // getting the origins of the occupancy grids:
    getOrigins();

    // updating the occupancy grids based on the actual costmap from ROS:
    updateOccGrids();

    // updating the controller state:
    if (! controller.set_state(state, origin_x_attr, origin_y_attr,
                                origin_x_rep, origin_y_rep))
    {
      ROS_WARN("Setting the state to the controller was unsuccessful!");
      return false;
    }

    // setting the new goal to the controller:
    if (! controller.set_new_goal(goal, origin_x_attr, origin_y_attr))
    {
      ROS_WARN("Setting the goal was unsuccessful!");
      ROS_INFO_STREAM("robot is free: " << controller.robot_is_free());
      ROS_INFO_STREAM("goal is reachable: " << controller.goal_is_reachable());
      return false;
    }

    // calculating the command velocities:
    double v_x, omega;
    if (controller.get_cmd_vel(v_x, omega))
    {
      cmd_vel.linear.x = v_x;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = omega;

      // for now setting back the velocities in the state:
      state.v = v_x;
      state.omega = omega;

      // checking whether the goal was reached:
      goal_is_reached = controller.goal_is_reached();
      return true;
    }
    else
    {
      ROS_WARN("Cmd_vel calculation was unsuccessful!");
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.0;

      // for now setting back the velocities in the state:
      state.v = v_x;
      state.omega = omega;

      return false;
    }
  }


  bool GradFieldPlannerROS::isGoalReached()
  {
    if (! initialized)
    {
      ROS_ERROR("GradFieldPlannerROS has not been initialized, so could not call isGoalReached().");
      return false;
    }

    if (goal_is_reached)
    {
      ROS_INFO("GOAL IS REACHED");
      // has to be set back, or else it is not possible to set a new goal.
      goal_is_reached = false;
      return true;
    }

    return false;
  }


  double GradFieldPlannerROS::getYaw(
    const geometry_msgs::PoseStamped* pose)
	{
		double q[4];
		q[0]= pose->pose.orientation.x;
		q[1]= pose->pose.orientation.y;
		q[2]= pose->pose.orientation.z;
		q[3]= pose->pose.orientation.w;

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
          occ_grid_attr[i][j] = true;
          q.push(new int [2] {i, j});
        }
        else
        {
          occ_grid_attr[i][j] = false;
        }
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
    // For now the attractor field has the same occupancy grid as the repulsive field.
    // this is why it is commented out.
    /*
    int x_diff = (size_x_attr - size_x_rep) / 2;
    int y_diff = (size_y_attr - size_y_rep) / 2;

    for (int i = 0; i < size_x_rep; i ++)
      for (int j = 0; j < size_y_rep; j ++)
        occ_grid_rep[i][j] = occ_grid_attr[i + x_diff][j + y_diff];
    */
  }


  bool GradFieldPlannerROS::getState()
  {
    if (costmap_ros->getRobotPose(rob_pose))
    {
      state.x = rob_pose.pose.position.x;
      state.y = rob_pose.pose.position.y;
      state.psi = getYaw(&rob_pose);
      return true;
    }
    else
      return false;
  }


  void GradFieldPlannerROS::getOrigins()
  {
    // The attractor field origin matches with the costmap origin:
    origin_x_attr = costmap->getOriginX();
    origin_y_attr = costmap->getOriginY();

    // The repulsive field is for now the same as the attractor field:
    origin_x_rep = origin_x_attr;
    origin_y_rep = origin_y_attr;
  }


  bool GradFieldPlannerROS::getGoal()
  {
    if (plan_ptr->size() == 0)
      return false;

    // TODO: transform the plan before finding the goal.
    // now it only works if the plan and the costmap are in the same frame.
    geometry_msgs::PoseStamped candidate;
    unsigned int mx, my;

    for (int i = 0; i < plan_ptr->size(); i ++)
    {
      candidate = (*plan_ptr)[i];
      costmap->worldToMap(candidate.pose.position.x, candidate.pose.position.y, mx, my);
      if ((mx == 0) || (mx == (size_x_attr - 1)) ||
          (my == 0) || (my == (size_y_attr - 1)))
        break;
    }

    goal.x = candidate.pose.position.x;
    goal.y = candidate.pose.position.y;
    goal.psi = getYaw(&candidate);

    return true;
  }
} // namespace grad_field_local_planner
