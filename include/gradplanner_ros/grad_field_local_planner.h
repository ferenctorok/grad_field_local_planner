#ifndef GRAD_FIELD_LOCAL_PLANNER_H
#define GRAD_FIELD_LOCAL_PLANNER_H

/* gradplanner includes */
#include <gradplanner/grad_field_controller.h>
#include <gradplanner/utils.h>

/* standard includes */
#include <chrono>
#include <vector>
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>


namespace grad_field_local_planner
{
  class GradFieldPlannerROS : public nav_core::BaseLocalPlanner
  {
    public:
      /**
       * @brief Default constructor of the GradFieldPlannerROS class.
       */
      GradFieldPlannerROS();

      /**
       * @brief Constructor of the GradFieldPlannerROS class.
       * @param name The name to give this instance of the local planner.
       * @param tf A pointer to a transform listener.
       * @param costmap_ros The cost map used for local planning.
       */
      GradFieldPlannerROS(string name, tf2_ros::Buffer* tf,
                          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor of the GradFieldPlannerROS class.
       */
      ~GradFieldPlannerROS() {}

      /**
       * @brief  Constructs the local planner.
       * @param name The name to give this instance of the local planner.
       * @param tf A pointer to a transform listener.
       * @param costmap_ros The cost map used for local planning.
       */
      void initialize(string name, tf2_ros::Buffer* tf_buffer_,
                      costmap_2d::Costmap2DROS* costmap_ros_);

      /**
       * @brief  Set the plan that the local planner is following.
       * @param plan The plan to pass to the local planner.
       * @return True if the plan was updated successfully, false otherwise.
       */
      bool setPlan(const vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base.
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
       * @return True if a valid velocity command was found, false otherwise.
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved by the local planner.
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      /**
       * @brief Amcl Callback. It is called every time a new pose estimate
       * is available on the amcl_pose topic.
       * @param Pointer to the received message
       */
      void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);


    private:
      bool initialized; // true if the object is already initialized.

      // Pointers to external objects received from move_base:
      costmap_2d::Costmap2DROS* costmap_ros; // costmap
      tf2_ros::Buffer* tf_buffer; // transform buffer

      // Topics & Services
      ros::Subscriber amcl_sub; // subscribes to the amcl topic

      // Controller related
      gradplanner::GradFieldController controller;  // The gradient field based controller.
      vector<vector<bool >> occ_grid_attr;  // The occupancy grid of the attractor field.
      vector<vector<bool >> occ_grid_rep;   // The occupancy grid of the repulsive field.
      gradplanner::ControlParams params;  // The parameters of the planner and the controller.  
      unsigned int size_x_attr; // The x size of the attractive field.
      unsigned int size_y_attr; // The y size of the attractive field.
      unsigned int size_x_rep; // The x size of the repulsive field.
      unsigned int size_y_rep; // The y size of the repulsive field.
      gradplanner::State state; // The state of the robot.


      /**
       * @brief Sets up the variables based on the parameter server.
       */
      void setup_from_param_sever();

  };

} // namespace grad_field_local_planner


#endif // GRAD_FIELD_LOCAL_PLANNER_H