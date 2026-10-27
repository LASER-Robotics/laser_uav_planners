#include <laser_uav_planners/agile_planner.hpp>
#include <stdexcept>

namespace laser_uav_planners
{
/* AgilePlanner() //{ */
AgilePlanner::AgilePlanner(multirotor_t multirotor_params, pmm_t pmm_params) {
  pmm_trajectory_capsule_ = pmm_params;

  mass_ = multirotor_params.mass;
  G1_   = multirotor_params.G1.inverse();

}
//}

/* getAttitudeReference() //{ */
Eigen::Quaterniond AgilePlanner::getAttitudeReference(Eigen::Vector3d& acceleration, double yaw) {
  Eigen::Vector3d thrust_vec = acceleration - gravity;
  Eigen::Vector3d z_b;

  if (thrust_vec.norm() < 1e-4) {
    z_b = Eigen::Vector3d(0.0, 0.0, 1.0);
  } else {
    z_b = thrust_vec.normalized();
  }

  Eigen::Vector3d y_c(-std::sin(yaw), std::cos(yaw), 0.0);
  Eigen::Vector3d x_b;

  if (std::abs(z_b.dot(y_c)) > 0.99) {
    x_b = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
  } else {
    x_b = y_c.cross(z_b).normalized();
  }

  Eigen::Vector3d y_b = z_b.cross(x_b).normalized();

  Eigen::Matrix3d R;
  R.col(0) = x_b;
  R.col(1) = y_b;
  R.col(2) = z_b;

  return Eigen::Quaterniond(R);
}
//}

/* generateTrajectory() //{ */
void AgilePlanner::generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, laser_msgs::msg::PoseWithHeading end_waypoint, float speed,
                                      bool use_speed) {
  full_trajectory_path_.clear();
  full_trajectory_path_.shrink_to_fit();

  pmm::Vector<3>              start_position;
  pmm::Vector<3>              start_velocity;
  pmm::Vector<3>              mid_position;
  pmm::Vector<3>              end_position;
  pmm::Vector<3>              end_velocity;
  std::vector<pmm::Vector<3>> waypoints;

  start_position[0] = start_waypoint.pose.position.x;
  start_position[1] = start_waypoint.pose.position.y;
  start_position[2] = start_waypoint.pose.position.z;
  start_velocity[0] = start_waypoint.use_linear_velocity ? start_waypoint.twist.linear.x : 0.0;
  start_velocity[1] = start_waypoint.use_linear_velocity ? start_waypoint.twist.linear.y : 0.0;
  start_velocity[2] = start_waypoint.use_linear_velocity ? start_waypoint.twist.linear.z : 0.0;

  end_position[0] = end_waypoint.position.x;
  end_position[1] = end_waypoint.position.y;
  end_position[2] = end_waypoint.position.z;
  end_velocity[0] = 0;
  end_velocity[1] = 0;
  end_velocity[2] = 0;

  mid_position[0] = start_position[0] + ((end_position[0] - start_position[0]) / 2);
  mid_position[1] = start_position[1] + ((end_position[1] - start_position[1]) / 2);
  mid_position[2] = start_position[2] + ((end_position[2] - start_position[2]) / 2);

  waypoints.push_back(start_position);
  waypoints.push_back(mid_position);
  waypoints.push_back(end_position);

  speed = std::min((pmm::Scalar)speed, pmm_trajectory_capsule_.max_vel_norm);
  if (speed <= 0) {
    speed = pmm_trajectory_capsule_.default_vel_norm;
  }

  pmm::PMM_MG_Trajectory3D mp_tr(
      waypoints, start_velocity, end_velocity, pmm_trajectory_capsule_.max_accel_norm, use_speed ? speed : pmm_trajectory_capsule_.default_vel_norm,
      pmm_trajectory_capsule_.dt_precision, pmm_trajectory_capsule_.first_run_max_iter, pmm_trajectory_capsule_.first_run_alpha,
      pmm_trajectory_capsule_.first_run_alpha_reduction_factor, pmm_trajectory_capsule_.first_run_alpha_min_threshold,
      pmm_trajectory_capsule_.thrust_decomp_max_iter, pmm_trajectory_capsule_.thrust_decomp_acc_precision, pmm_trajectory_capsule_.run_second_opt,
      pmm_trajectory_capsule_.second_run_max_iter, pmm_trajectory_capsule_.second_run_alpha, pmm_trajectory_capsule_.second_run_alpha_reduction_factor,
      pmm_trajectory_capsule_.second_run_alpha_min_threshold, pmm_trajectory_capsule_.use_drag, false);

  std::vector<pmm::Vector<3>> p_s;
  std::vector<pmm::Vector<3>> v_s;
  std::vector<pmm::Vector<3>> a_s;

  std::tie(trajectory_time_, p_s, v_s, a_s) = mp_tr.get_sampled_trajectory(pmm_trajectory_capsule_.sampling_step);

  for (auto i = 0; i < (int)trajectory_time_.size(); i++) {
    laser_msgs::msg::ReferenceState ref;

    ref.pose.position.x = p_s[i][0];
    ref.pose.position.y = p_s[i][1];
    ref.pose.position.z = p_s[i][2];
    ref.use_position    = true;

    ref.twist.linear.x      = v_s[i][0];
    ref.twist.linear.y      = v_s[i][1];
    ref.twist.linear.z      = v_s[i][2];
    ref.use_linear_velocity = true;

    Eigen::Vector3d acceleration;
    if (i + 1 == (int)trajectory_time_.size()) {
      acceleration << 0.0, 0.0, 0.0;
    } else {
      acceleration << a_s[i][0], a_s[i][1], a_s[i][2];
    }

    Eigen::Quaterniond q   = getAttitudeReference(acceleration, end_waypoint.heading);
    ref.pose.orientation.w = q.w();
    ref.pose.orientation.x = q.x();
    ref.pose.orientation.y = q.y();
    ref.pose.orientation.z = q.z();
    ref.use_orientation    = true;

    ref.twist.angular.x      = 0.0;
    ref.twist.angular.y      = 0.0;
    ref.twist.angular.z      = 0.0;
    ref.use_angular_velocity = true;

    ref.individual_thrust.data = std::vector<double>(G1_.cols(), (mass_ * (acceleration - gravity).norm()) / G1_.cols());
    ref.use_individual_thrust  = false;

    full_trajectory_path_.push_back(ref);
  }


  return;
}
//}

/* generateTrajectory() //{ */
void AgilePlanner::generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, std::vector<laser_msgs::msg::PoseWithHeading> waypoints, float speed) {
  if (waypoints.empty()) {
    throw std::invalid_argument("PMM requires at least one waypoint");
  }
  full_trajectory_path_.clear();
  full_trajectory_path_.shrink_to_fit();

  pmm::Vector<3>              start_position;
  pmm::Vector<3>              start_velocity;
  pmm::Vector<3>              end_velocity;
  std::vector<pmm::Vector<3>> waypoints_mp;

  start_position[0] = start_waypoint.pose.position.x;
  start_position[1] = start_waypoint.pose.position.y;
  start_position[2] = start_waypoint.pose.position.z;
  start_velocity[0] = start_waypoint.use_linear_velocity ? start_waypoint.twist.linear.x : 0.0;
  start_velocity[1] = start_waypoint.use_linear_velocity ? start_waypoint.twist.linear.y : 0.0;
  start_velocity[2] = start_waypoint.use_linear_velocity ? start_waypoint.twist.linear.z : 0.0;

  end_velocity[0] = 0;
  end_velocity[1] = 0;
  end_velocity[2] = 0;

  waypoints_mp.push_back(start_position);

  for (auto i = 0; i < (int)waypoints.size(); i++) {
    pmm::Vector<3> waypoint_intermediary;

    waypoint_intermediary[0] = waypoints[i].position.x;
    waypoint_intermediary[1] = waypoints[i].position.y;
    waypoint_intermediary[2] = waypoints[i].position.z;

    waypoints_mp.push_back(waypoint_intermediary);
  }

  if (speed <= 0) {
    speed = pmm_trajectory_capsule_.default_vel_norm;
  }

  pmm::PMM_MG_Trajectory3D mp_tr(waypoints_mp, start_velocity, end_velocity, pmm_trajectory_capsule_.max_accel_norm,
                                 (pmm::Scalar)std::min(speed, (float)pmm_trajectory_capsule_.max_vel_norm), pmm_trajectory_capsule_.dt_precision,
                                 pmm_trajectory_capsule_.first_run_max_iter, pmm_trajectory_capsule_.first_run_alpha,
                                 pmm_trajectory_capsule_.first_run_alpha_reduction_factor, pmm_trajectory_capsule_.first_run_alpha_min_threshold,
                                 pmm_trajectory_capsule_.thrust_decomp_max_iter, pmm_trajectory_capsule_.thrust_decomp_acc_precision,
                                 pmm_trajectory_capsule_.run_second_opt, pmm_trajectory_capsule_.second_run_max_iter, pmm_trajectory_capsule_.second_run_alpha,
                                 pmm_trajectory_capsule_.second_run_alpha_reduction_factor, pmm_trajectory_capsule_.second_run_alpha_min_threshold,
                                 pmm_trajectory_capsule_.use_drag, false);

  std::vector<pmm::Vector<3>> p_s;
  std::vector<pmm::Vector<3>> v_s;
  std::vector<pmm::Vector<3>> a_s;

  std::tie(trajectory_time_, p_s, v_s, a_s) = mp_tr.get_sampled_trajectory(pmm_trajectory_capsule_.sampling_step);

  int j = 0;
  for (auto i = 0; i < (int)trajectory_time_.size(); i++) {
    laser_msgs::msg::ReferenceState ref;

    ref.pose.position.x = p_s[i][0];
    ref.pose.position.y = p_s[i][1];
    ref.pose.position.z = p_s[i][2];
    ref.use_position    = true;

    ref.twist.linear.x      = v_s[i][0];
    ref.twist.linear.y      = v_s[i][1];
    ref.twist.linear.z      = v_s[i][2];
    ref.use_linear_velocity = true;

    Eigen::Vector3d acceleration;
    if (i + 1 == (int)trajectory_time_.size()) {
      acceleration << 0.0, 0.0, 0.0;
    } else {
      acceleration << a_s[i][0], a_s[i][1], a_s[i][2];
    }

    Eigen::Quaterniond q   = getAttitudeReference(acceleration, waypoints[j].heading);
    ref.pose.orientation.w = q.w();
    ref.pose.orientation.x = q.x();
    ref.pose.orientation.y = q.y();
    ref.pose.orientation.z = q.z();
    ref.use_orientation    = true;

    if (j + 1 < static_cast<int>(waypoints.size()) &&
        sqrt(pow(waypoints[j].position.x - p_s[i][0], 2) + pow(waypoints[j].position.y - p_s[i][1], 2) + pow(waypoints[j].position.z - p_s[i][2], 2)) <= 0.1) {
      j++;
    }

    ref.twist.angular.x      = 0.0;
    ref.twist.angular.y      = 0.0;
    ref.twist.angular.z      = 0.0;
    ref.use_angular_velocity = true;

    ref.individual_thrust.data = std::vector<double>(G1_.cols(), (mass_ * (acceleration - gravity).norm()) / G1_.cols());
    ref.use_individual_thrust  = false;

    full_trajectory_path_.push_back(ref);
  }


  return;
}
//}

const std::vector<laser_msgs::msg::ReferenceState> & AgilePlanner::getTrajectory() const
{
  return full_trajectory_path_;
}

const std::vector<double> & AgilePlanner::getTrajectoryTimes() const
{
  return trajectory_time_;
}

/* setMass() //{ */
void AgilePlanner::setMass(double mass) {
  mass_ = mass;
}
//}
}  // namespace laser_uav_planners
