# include <gradplanner_ros/grad_field_local_planner.h>


namespace grad_field_local_planner
{
  void GradFieldPlannerROS::getParams()
  {
    // update the params from the parameter server:
    // general //
    double control_freq;
    if (ros::param::has("/move_base/controller_frequency"))
    {
      ros::param::get("/move_base/controller_frequency", control_freq);
      params.general.Ts = 1 / control_freq;
    }

    if (ros::param::has("/move_base/local_costmap/resolution"))
      ros::param::get("/move_base/local_costmap/resolution", params.general.cell_size);

    if (ros::param::has("/move_base/GradFieldPlannerROS/general/R"))
      ros::param::get("/move_base/GradFieldPlannerROS/general/R", params.general.R);
    
    if (ros::param::has("/move_base/GradFieldPlannerROS/general/end_pos_tol"))
      ros::param::get("/move_base/GradFieldPlannerROS/general/end_pos_tol", params.general.end_pos_tol);

    if (ros::param::has("/move_base/GradFieldPlannerROS/general/end_ang_tol"))
      ros::param::get("/move_base/GradFieldPlannerROS/general/end_ang_tol", params.general.end_ang_tol);

    if (ros::param::has("/move_base/GradFieldPlannerROS/general/max_trans_vel"))
      ros::param::get("/move_base/GradFieldPlannerROS/general/max_trans_vel", params.general.max_trans_vel);

    if (ros::param::has("/move_base/GradFieldPlannerROS/general/max_trans_acc"))
      ros::param::get("/move_base/GradFieldPlannerROS/general/max_trans_acc", params.general.max_trans_acc);

    if (ros::param::has("/move_base/GradFieldPlannerROS/general/max_ang_vel"))
      ros::param::get("/move_base/GradFieldPlannerROS/general/max_ang_vel", params.general.max_ang_vel);

    if (ros::param::has("/move_base/GradFieldPlannerROS/general/max_ang_acc"))
      ros::param::get("/move_base/GradFieldPlannerROS/general/max_ang_acc", params.general.max_ang_acc);

    if (ros::param::has("/move_base/GradFieldPlannerROS/general/decel_ratio"))
      ros::param::get("/move_base/GradFieldPlannerROS/general/decel_ratio", params.general.decel_ratio);

    // grad_mode //
    if (ros::param::has("/move_base/GradFieldPlannerROS/grad_mode/K"))
      ros::param::get("/move_base/GradFieldPlannerROS/grad_mode/K", params.grad_mode.K);
    
    if (ros::param::has("/move_base/GradFieldPlannerROS/grad_mode/boundary_error"))
      ros::param::get("/move_base/GradFieldPlannerROS/grad_mode/boundary_error", params.grad_mode.boundary_error);

    if (ros::param::has("/move_base/GradFieldPlannerROS/grad_mode/max_error"))
      ros::param::get("/move_base/GradFieldPlannerROS/grad_mode/max_error", params.grad_mode.max_error);
    
    // direct_mode //
    if (ros::param::has("/move_base/GradFieldPlannerROS/direct_mode/min_obst_dist"))
      ros::param::get("/move_base/GradFieldPlannerROS/direct_mode/min_obst_dist", params.direct_mode.min_obst_dist);

    if (ros::param::has("/move_base/GradFieldPlannerROS/direct_mode/K"))
      ros::param::get("/move_base/GradFieldPlannerROS/direct_mode/K", params.direct_mode.K);
    
    if (ros::param::has("/move_base/GradFieldPlannerROS/direct_mode/boundary_error"))
      ros::param::get("/move_base/GradFieldPlannerROS/direct_mode/boundary_error", params.direct_mode.boundary_error);
    
    if (ros::param::has("/move_base/GradFieldPlannerROS/direct_mode/max_error"))
      ros::param::get("/move_base/GradFieldPlannerROS/direct_mode/max_error", params.direct_mode.max_error);
    
    // end_mode //
    if (ros::param::has("/move_base/GradFieldPlannerROS/end_mode/K"))
      ros::param::get("/move_base/GradFieldPlannerROS/end_mode/K", params.end_mode.K);
    
    // attractor //
    if (ros::param::has("/move_base/GradFieldPlannerROS/attractor/search_dir_8"))
      ros::param::get("/move_base/GradFieldPlannerROS/attractor/search_dir_8", params.attractor.search_dir_8);


    // Attractor field sizes:
    size_x_attr = costmap->getSizeInCellsX();
    size_y_attr = costmap->getSizeInCellsY();


    // Repulsive field sizes:
    // It's enough for the repulsive field to see every obstacle that is within
    // the effective radius. The others will don't have an effect anyways.
    /*
    size_x_rep = 2 * params.general.R + 2;
    if (size_x_rep > size_x_attr)
    {
      size_x_rep = size_y_attr;
      ROS_WARN("Had to resize repulsive field size in x direction because it was bigger than the occupancy grid.");
    }
    size_y_rep = 2 * params.general.R + 2;
    if (size_y_rep > size_y_attr)
    {
      size_y_rep = size_y_attr;
      ROS_WARN("Had to resize repulsive field size in y direction because it was bigger than the occupancy grid.");
    }
    */

    // For now the repulsive and the attractor fields have the same size:
    size_x_rep = size_x_attr;
    size_y_rep = size_y_attr;

    // The inflation radius:
    if (ros::param::has("/move_base/local_costmap/robot_radius"))
    {
      double R;
      ros::param::get("/move_base/local_costmap/robot_radius", R);
      safety_R = int(R) + 1;
    }
    else
      safety_R = 0;


    // Printing summary:
    printSummary();
  }


  void GradFieldPlannerROS::printSummary()
  {
    ROS_INFO_STREAM(std::endl << "--- GradFieldPlannerROS Summary Begin ---");

    // Gradient field sizes:
    ROS_INFO_STREAM("- size_x_attr: " << size_x_attr);
    ROS_INFO_STREAM("- size_y_attr: " << size_y_attr);
    ROS_INFO_STREAM("- size_x_rep: " << size_x_rep);
    ROS_INFO_STREAM("- size_y_rep: " << size_y_rep << std::endl);

    // params //
    // general //
    ROS_INFO_STREAM("- params.general.cell_size: " << params.general.cell_size);
    ROS_INFO_STREAM("- params.general.Ts: " << params.general.Ts);
    ROS_INFO_STREAM("- params.general.R: " << params.general.R);
    ROS_INFO_STREAM("- params.general.end_pos_tol: " << params.general.end_pos_tol);
    ROS_INFO_STREAM("- params.general.end_ang_tol: " << params.general.end_ang_tol);
    ROS_INFO_STREAM("- params.general.max_trans_vel: " << params.general.max_trans_vel);
    ROS_INFO_STREAM("- params.general.max_trans_acc: " << params.general.max_trans_acc);
    ROS_INFO_STREAM("- params.general.max_ang_vel: " << params.general.max_ang_vel);
    ROS_INFO_STREAM("- params.general.max_ang_acc: " << params.general.max_ang_acc);
    ROS_INFO_STREAM("- params.general.decel_ratio: " << params.general.decel_ratio << std::endl);

    // grad_mode //
    ROS_INFO_STREAM("- params.grad_mode.K: " << params.grad_mode.K);
    ROS_INFO_STREAM("- params.grad_mode.boundary_error: " << params.grad_mode.boundary_error);
    ROS_INFO_STREAM("- params.grad_mode.max_error: " << params.grad_mode.max_error << std::endl);

    // direct_mode //
    ROS_INFO_STREAM("- params.direct_mode.min_obst_dist: " << params.direct_mode.min_obst_dist);
    ROS_INFO_STREAM("- params.direct_mode.K: " << params.direct_mode.K);
    ROS_INFO_STREAM("- params.direct_mode.boundary_error: " << params.direct_mode.boundary_error);
    ROS_INFO_STREAM("- params.direct_mode.max_error: " << params.direct_mode.max_error << std::endl);

    // end mode //
    ROS_INFO_STREAM("- params.end_mode.K: " << params.end_mode.K << std::endl);

    // attractor //
    ROS_INFO_STREAM("- params.attractor.search_dir_8: " << params.attractor.search_dir_8);

    // other //
    ROS_INFO_STREAM("- Safety inflation radius: " << safety_R << std::endl);

    ROS_INFO("--- GradFieldPlannerROS Summary End ---\n");
  }
} // namespace grad_field_local_planner