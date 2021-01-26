#ifndef GRAD_FIELD_LOCAL_PLANNER_H
#define GRAD_FIELD_LOCAL_PLANNER_H


/* standard includes */
// timing 
#include <chrono>

//files
#include <fstream>
#include <iostream>
using namespace std;

/* ROS related includes: */
#include <ros/ros.h>

// abstract class from which the plugin inherits
#include <nav_core/base_local_planner.h>

// tf_2:
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// msgs:
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>


namespace grad_field_local_planner
{
  class GradFieldPlannerROS : public nav_core::BaseLocalPlanner
  {
    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      GradFieldPlannerROS();

      /**
       * @brief  Destructor for the wrapper
       */
      ~GradFieldPlannerROS();

      /**
       * @brief  Constructs the local planner
       * @param name The name to give this instance of the local planner
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to local plans
       */
      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid velocity command was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    private:
      bool initialized; // true if the object is already initialized.

      // Pointers to external objects received from move_base:
      costmap_2d::Costmap2DROS* costmap_ros; // costmap
      tf2_ros::Buffer* tf_buffer; // transform buffer

      // Topics & Services
      ros::Subscriber amcl_sub; // subscribes to the amcl topic     
  };

} // namespace grad_field_local_planner


#endif