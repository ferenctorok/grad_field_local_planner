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
    if (ros::param::has("/move_base/GradFieldPlannerROS/attractor/size_x"))
    {
      ros::param::get("/move_base/GradFieldPlannerROS/attractor/size_x", size_x_attr);
      // the size should be odd:
      if (int(size_x_attr / 2) == (size_x_attr / 2))
        size_x_attr ++;
    }
    else size_x_attr = 33;  // Default
    
    if (ros::param::has("/move_base/GradFieldPlannerROS/attractor/size_y"))
    {
      ros::param::get("/move_base/GradFieldPlannerROS/attractor/size_y", size_y_attr);
      // the size should be odd:
      if (int(size_y_attr / 2) == (size_y_attr / 2))
        size_y_attr ++;
    }
    else size_y_attr = 33;  // Default

    // Repulsive field sizes:
    // It's enough the field to see every obstacle that is within
    // the effective radius. The others will don't have an effect anyways.
    size_x_rep = 2 * params.general.R + 3;
    size_x_rep = 2 * params.general.R + 3;
  }
} // namespace grad_field_local_planner