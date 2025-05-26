#include <laser_uav_planner/agile_planner.hpp>

namespace laser_uav_planner
{
/* AgilePlanner() {default} //{ */
AgilePlanner::AgilePlanner() {
  /* pmm_trajectory_capsule_.max_velocity[0] = 15.0; */
  /* pmm_trajectory_capsule_.max_velocity[1] = 15.0; */
  /* pmm_trajectory_capsule_.max_velocity[2] = 10.0; */

  pmm_trajectory_capsule_.max_acc_norm = 34.32;
  pmm_trajectory_capsule_.max_vel_norm = 90.0;

  pmm_trajectory_capsule_.use_drag                    = false;
  pmm_trajectory_capsule_.thrust_decomp_acc_precision = 0.01;
  pmm_trajectory_capsule_.thrust_decomp_max_iter      = 20;

  pmm_trajectory_capsule_.first_run_alpha                  = 10.0;
  pmm_trajectory_capsule_.first_run_alpha_reduction_factor = 0.2;
  pmm_trajectory_capsule_.first_run_alpha_min_threshold    = 0.001;
  pmm_trajectory_capsule_.first_run_max_iter               = 30;

  pmm_trajectory_capsule_.run_second_opt                    = true;
  pmm_trajectory_capsule_.second_run_alpha                  = 35.0;
  pmm_trajectory_capsule_.second_run_alpha_reduction_factor = 0.1;
  pmm_trajectory_capsule_.second_run_alpha_min_threshold    = 0.01;
  pmm_trajectory_capsule_.second_run_max_iter               = 10;

  pmm_trajectory_capsule_.dt_precision  = 0.001;
  pmm_trajectory_capsule_.sampling_step = 0.001;
}
//}

/* AgilePlanner() //{ */
AgilePlanner::AgilePlanner(pmm_t pmm_params) {
  pmm_trajectory_capsule_ = pmm_params;
}
//}

/* generateTrajectory() //{ */
bool AgilePlanner::generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, laser_msgs::msg::ReferenceState end_waypoint) {
  full_trajectory_path_.clear();
  full_trajectory_path_.shrink_to_fit();

  pmm::Vector<3>              start_position;
  pmm::Vector<3>              start_velocity;
  pmm::Vector<3>              end_position;
  pmm::Vector<3>              end_velocity;
  std::vector<pmm::Vector<3>> waypoints;

  start_position[0] = start_waypoint.pose.position.x;
  start_position[1] = start_waypoint.pose.position.y;
  start_position[2] = start_waypoint.pose.position.z;
  start_velocity[0] = start_waypoint.twist.linear.x;
  start_velocity[1] = start_waypoint.twist.linear.y;
  start_velocity[2] = start_waypoint.twist.linear.z;

  end_position[0] = end_waypoint.pose.position.x;
  end_position[1] = end_waypoint.pose.position.y;
  end_position[2] = end_waypoint.pose.position.z;
  end_velocity[0] = end_waypoint.twist.linear.x;
  end_velocity[1] = end_waypoint.twist.linear.y;
  end_velocity[2] = end_waypoint.twist.linear.z;

  waypoints.push_back(start_position);
  waypoints.push_back(end_position);

  pmm::PMM_MG_Trajectory3D mp_tr(waypoints, start_velocity, end_velocity, pmm_trajectory_capsule_.max_acc_norm, pmm_trajectory_capsule_.max_vel_norm,
                                 pmm_trajectory_capsule_.dt_precision, pmm_trajectory_capsule_.first_run_max_iter, pmm_trajectory_capsule_.first_run_alpha,
                                 pmm_trajectory_capsule_.first_run_alpha_reduction_factor, pmm_trajectory_capsule_.first_run_alpha_min_threshold,
                                 pmm_trajectory_capsule_.thrust_decomp_max_iter, pmm_trajectory_capsule_.thrust_decomp_acc_precision,
                                 pmm_trajectory_capsule_.run_second_opt, pmm_trajectory_capsule_.second_run_max_iter, pmm_trajectory_capsule_.second_run_alpha,
                                 pmm_trajectory_capsule_.second_run_alpha_reduction_factor, pmm_trajectory_capsule_.second_run_alpha_min_threshold,
                                 pmm_trajectory_capsule_.use_drag, false);

  std::vector<pmm::Scalar>    t_s;
  std::vector<pmm::Vector<3>> p_s;
  std::vector<pmm::Vector<3>> v_s;
  std::vector<pmm::Vector<3>> a_s;

  std::tie(t_s, p_s, v_s, a_s) = mp_tr.get_sampled_trajectory(pmm_trajectory_capsule_.sampling_step);

  for (auto i = 0; i < (int)t_s.size(); i++) {
    laser_msgs::msg::ReferenceState ref;

    ref.pose.position.x = p_s[i][0];
    ref.pose.position.y = p_s[i][1];
    ref.pose.position.z = p_s[i][2];

    ref.pose.orientation = end_waypoint.pose.orientation;

    ref.twist.linear.x = v_s[i][0];
    ref.twist.linear.y = v_s[i][1];
    ref.twist.linear.z = v_s[i][2];

    ref.use_position         = true;
    ref.use_orientation      = true;
    ref.use_linear_velocity  = true;
    ref.use_angular_velocity = false;

    full_trajectory_path_.push_back(ref);
  }

  current_waypoint_ = 0;

  return true;
}
//}

/* updateReference() //{ */
laser_msgs::msg::ReferenceState AgilePlanner::updateReference(nav_msgs::msg::Odometry odometry) {
  if (current_waypoint_ < (int)full_trajectory_path_.size()) {
    laser_msgs::msg::ReferenceState waypoint = full_trajectory_path_[current_waypoint_];

    if (sqrt(pow(odometry.pose.pose.position.x - full_trajectory_path_[current_waypoint_].pose.position.x, 2) +
             pow(odometry.pose.pose.position.y - full_trajectory_path_[current_waypoint_].pose.position.y, 2) +
             pow(odometry.pose.pose.position.z - full_trajectory_path_[current_waypoint_].pose.position.z, 2)) <= 0.4) {
      current_waypoint_++;
    }
    return waypoint;
  } else {
    std::cout << "!-- hover state --!" << std::endl;
    return full_trajectory_path_[(int)full_trajectory_path_.size() - 1];
  }
}
//}

/* getTrajectory() //{ */
std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points, nav_msgs::msg::Odometry odometry) {

}
//}
}  // namespace laser_uav_planner
