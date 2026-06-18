#include <laser_uav_planners/agile_planner.hpp>

namespace laser_uav_planners
{
/* AgilePlanner() {default} //{ */
AgilePlanner::AgilePlanner() {
}
//}

/* AgilePlanner() //{ */
AgilePlanner::AgilePlanner(multirotor_t multirotor_params, pmm_t pmm_params, double controller_dt) {
  pmm_trajectory_capsule_ = pmm_params;

  mass_ = multirotor_params.mass;
  G1_   = multirotor_params.G1.inverse();

  controller_dt_ = controller_dt;
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
  generating_trajectory_ = true;
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
  start_velocity[0] = 0.0;
  start_velocity[1] = 0.0;
  start_velocity[2] = 0.0;

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

  is_hover_         = false;
  take_anchor_time_ = true;

  generating_trajectory_ = false;

  return;
}
//}

/* generateTrajectory() //{ */
void AgilePlanner::generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, std::vector<laser_msgs::msg::PoseWithHeading> waypoints, float speed) {
  generating_trajectory_ = true;
  full_trajectory_path_.clear();
  full_trajectory_path_.shrink_to_fit();

  pmm::Vector<3>              start_position;
  pmm::Vector<3>              start_velocity;
  pmm::Vector<3>              end_velocity;
  std::vector<pmm::Vector<3>> waypoints_mp;

  start_position[0] = start_waypoint.pose.position.x;
  start_position[1] = start_waypoint.pose.position.y;
  start_position[2] = start_waypoint.pose.position.z;
  start_velocity[0] = 0.0;
  start_velocity[1] = 0.0;
  start_velocity[2] = 0.0;

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

    if (sqrt(pow(waypoints[j].position.x - p_s[i][0], 2) + pow(waypoints[j].position.y - p_s[i][1], 2) + pow(waypoints[j].position.z - p_s[i][2], 2)) <= 0.1) {
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

  is_hover_              = false;
  take_anchor_time_      = true;
  generating_trajectory_ = false;

  return;
}
//}

/* getTrajectory() //{ */
std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points, double current_time) {
  std::vector<laser_msgs::msg::ReferenceState> sampled_trajectory;

  if (generating_trajectory_) {
    return std::vector<laser_msgs::msg::ReferenceState>(qty_points, hover_wait_waypoint_);
  }

  if (take_anchor_time_) {
    start_trajectory_time_ = current_time;
    take_anchor_time_      = false;
  }

  double elapsed_time = current_time - start_trajectory_time_;

  int count_hover = 0;

  for (int k = 0; k < qty_points; ++k) {
    double target_time = elapsed_time + (k * controller_dt_);

    auto it = std::lower_bound(trajectory_time_.begin(), trajectory_time_.end(), target_time);

    if (it == trajectory_time_.begin()) {
      sampled_trajectory.push_back(full_trajectory_path_.front());
      continue;
    }

    if (it == trajectory_time_.end()) {
      sampled_trajectory.push_back(full_trajectory_path_.back());
      count_hover++;
      continue;
    }

    int idx_next = std::distance(trajectory_time_.begin(), it);
    int idx_prev = idx_next - 1;

    double t_prev = trajectory_time_[idx_prev];
    double t_next = trajectory_time_[idx_next];
    double alpha  = (target_time - t_prev) / (t_next - t_prev);

    auto state_prev = full_trajectory_path_[idx_prev];
    auto state_next = full_trajectory_path_[idx_next];

    laser_msgs::msg::ReferenceState interp_state;
    interp_state.use_position          = state_prev.use_position;
    interp_state.use_orientation       = state_prev.use_orientation;
    interp_state.use_linear_velocity   = state_prev.use_linear_velocity;
    interp_state.use_angular_velocity  = state_prev.use_angular_velocity;
    interp_state.use_individual_thrust = state_prev.use_individual_thrust;

    interp_state.pose.position.x = state_prev.pose.position.x + alpha * (state_next.pose.position.x - state_prev.pose.position.x);
    interp_state.pose.position.y = state_prev.pose.position.y + alpha * (state_next.pose.position.y - state_prev.pose.position.y);
    interp_state.pose.position.z = state_prev.pose.position.z + alpha * (state_next.pose.position.z - state_prev.pose.position.z);

    double q_dot = state_prev.pose.orientation.w * state_next.pose.orientation.w + state_prev.pose.orientation.x * state_next.pose.orientation.x +
                   state_prev.pose.orientation.y * state_next.pose.orientation.y + state_prev.pose.orientation.z * state_next.pose.orientation.z;

    double q_w = state_next.pose.orientation.w;
    double q_x = state_next.pose.orientation.x;
    double q_y = state_next.pose.orientation.y;
    double q_z = state_next.pose.orientation.z;

    if (q_dot < 0.0) {
      q_w = -q_w;
      q_x = -q_x;
      q_y = -q_y;
      q_z = -q_z;
    }

    interp_state.pose.orientation.w = state_prev.pose.orientation.w + alpha * (q_w - state_prev.pose.orientation.w);
    interp_state.pose.orientation.x = state_prev.pose.orientation.x + alpha * (q_x - state_prev.pose.orientation.x);
    interp_state.pose.orientation.y = state_prev.pose.orientation.y + alpha * (q_y - state_prev.pose.orientation.y);
    interp_state.pose.orientation.z = state_prev.pose.orientation.z + alpha * (q_z - state_prev.pose.orientation.z);

    double norm =
        std::sqrt(interp_state.pose.orientation.w * interp_state.pose.orientation.w + interp_state.pose.orientation.x * interp_state.pose.orientation.x +
                  interp_state.pose.orientation.y * interp_state.pose.orientation.y + interp_state.pose.orientation.z * interp_state.pose.orientation.z);

    interp_state.pose.orientation.w /= norm;
    interp_state.pose.orientation.x /= norm;
    interp_state.pose.orientation.y /= norm;
    interp_state.pose.orientation.z /= norm;

    interp_state.twist.linear.x = state_prev.twist.linear.x + alpha * (state_next.twist.linear.x - state_prev.twist.linear.x);
    interp_state.twist.linear.y = state_prev.twist.linear.y + alpha * (state_next.twist.linear.y - state_prev.twist.linear.y);
    interp_state.twist.linear.z = state_prev.twist.linear.z + alpha * (state_next.twist.linear.z - state_prev.twist.linear.z);

    interp_state.twist.angular.x = state_prev.twist.angular.x + alpha * (state_next.twist.angular.x - state_prev.twist.angular.x);
    interp_state.twist.angular.y = state_prev.twist.angular.y + alpha * (state_next.twist.angular.y - state_prev.twist.angular.y);
    interp_state.twist.angular.z = state_prev.twist.angular.z + alpha * (state_next.twist.angular.z - state_prev.twist.angular.z);

    interp_state.individual_thrust.unit_of_measurement = state_prev.individual_thrust.unit_of_measurement;
    if (state_prev.individual_thrust.data.size() == state_next.individual_thrust.data.size()) {
      for (size_t i = 0; i < state_prev.individual_thrust.data.size(); ++i) {
        double interpolated_thrust =
            state_prev.individual_thrust.data[i] + alpha * (state_next.individual_thrust.data[i] - state_prev.individual_thrust.data[i]);
        interp_state.individual_thrust.data.push_back(interpolated_thrust);
      }
    } else {
      interp_state.individual_thrust.data = state_prev.individual_thrust.data;
    }

    sampled_trajectory.push_back(interp_state);
  }

  if (count_hover == qty_points) {
    is_hover_ = true;
  }

  hover_wait_waypoint_                       = sampled_trajectory[0];
  hover_wait_waypoint_.use_linear_velocity   = false;
  hover_wait_waypoint_.use_angular_velocity  = false;
  hover_wait_waypoint_.use_individual_thrust = false;

  return sampled_trajectory;
}
//}

/* isHover() //{ */
bool AgilePlanner::isHover() {
  return is_hover_;
}
//}

/* setMass() //{ */
void AgilePlanner::setMass(double mass) {
  mass_ = mass;
}
//}
}  // namespace laser_uav_planners
