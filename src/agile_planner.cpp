#include <laser_uav_planners/agile_planner.hpp>

namespace laser_uav_planners
{
/* AgilePlanner() {default} //{ */
AgilePlanner::AgilePlanner() {
}
//}

/* AgilePlanner() //{ */
AgilePlanner::AgilePlanner(quadrotor_t quadrotor_params, pmm_t pmm_params) {
  pmm_trajectory_capsule_ = pmm_params;

  mass_ = quadrotor_params.mass;

  G1_ = quadrotor_params.G1;

  inertia_matrix_ = quadrotor_params.inertia_matrix;
}
//}

/* generateRotationMatrix() //{ */
Eigen::Matrix3d AgilePlanner::generateRotationMatrix(Eigen::Vector3d& acceleration, double desired_heading) {
  Eigen::Vector3d       g(0.0, 0.0, -9.81);
  const Eigen::Vector3d thrust = mass_ * ((acceleration * -1) - g);

  const Eigen::Vector3d z_b_des = thrust.normalized();

  const Eigen::Vector3d x_c(1.0 * cos(desired_heading), 1.0 * sin(desired_heading), 0.0);

  const Eigen::Vector3d y_b_des = z_b_des.cross(x_c).normalized();
  const Eigen::Vector3d x_b_des = y_b_des.cross(z_b_des);

  Eigen::Matrix3d R_ref;
  R_ref.col(0) = x_b_des;
  R_ref.col(1) = y_b_des;
  R_ref.col(2) = z_b_des;

  return R_ref;
}
//}

/* generateIndividualThrust() //{ */
Eigen::Vector4d AgilePlanner::generateIndividualThrust(Eigen::Vector3d& acceleration, Eigen::Vector3d& omega) {
  Eigen::Vector3d g(0.0, 0.0, -9.81);
  Eigen::Vector3d thrust       = mass_ * ((acceleration * -1) - g);
  double          total_thrust = thrust.norm();

  /* Eigen::Vector3d alpharef = (current_omega - last_omega) / pmm_trajectory_capsule_.sampling_step; */

  Eigen::Vector3d torque = omega.cross(inertia_matrix_ * omega);

  Eigen::Vector4d wrench;
  wrench << total_thrust, torque;

  return G1_.inverse() * wrench;
}
//}

/* generateTrajectory() //{ */
bool AgilePlanner::generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, laser_msgs::msg::PoseWithHeading end_waypoint, float speed,
                                      bool use_speed) {
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

  end_position[0] = end_waypoint.position.x;
  end_position[1] = end_waypoint.position.y;
  end_position[2] = end_waypoint.position.z;
  end_velocity[0] = 0;
  end_velocity[1] = 0;
  end_velocity[2] = 0;

  waypoints.push_back(start_position);
  waypoints.push_back(end_position);

  speed = std::min((pmm::Scalar)speed, pmm_trajectory_capsule_.max_vel_norm);
  if (speed <= 0) {
    speed = pmm_trajectory_capsule_.default_vel_norm;
  }

  pmm::PMM_MG_Trajectory3D mp_tr(
      waypoints, start_velocity, end_velocity, pmm_trajectory_capsule_.max_acc_norm, use_speed ? speed : pmm_trajectory_capsule_.default_vel_norm,
      pmm_trajectory_capsule_.dt_precision, pmm_trajectory_capsule_.first_run_max_iter, pmm_trajectory_capsule_.first_run_alpha,
      pmm_trajectory_capsule_.first_run_alpha_reduction_factor, pmm_trajectory_capsule_.first_run_alpha_min_threshold,
      pmm_trajectory_capsule_.thrust_decomp_max_iter, pmm_trajectory_capsule_.thrust_decomp_acc_precision, pmm_trajectory_capsule_.run_second_opt,
      pmm_trajectory_capsule_.second_run_max_iter, pmm_trajectory_capsule_.second_run_alpha, pmm_trajectory_capsule_.second_run_alpha_reduction_factor,
      pmm_trajectory_capsule_.second_run_alpha_min_threshold, pmm_trajectory_capsule_.use_drag, false);

  std::vector<pmm::Scalar>    t_s;
  std::vector<pmm::Vector<3>> p_s;
  std::vector<pmm::Vector<3>> v_s;
  std::vector<pmm::Vector<3>> a_s;

  std::tie(t_s, p_s, v_s, a_s) = mp_tr.get_sampled_trajectory(pmm_trajectory_capsule_.sampling_step);

  Eigen::Vector3d last_omega(0.0, 0.0, 0.0);
  Eigen::Vector3d current_omega(0.0, 0.0, 0.0);
  for (auto i = 0; i < (int)t_s.size(); i++) {
    laser_msgs::msg::ReferenceState ref;

    ref.pose.position.x = p_s[i][0];
    ref.pose.position.y = p_s[i][1];
    ref.pose.position.z = p_s[i][2];
    ref.use_position    = true;

    ref.twist.linear.x      = v_s[i][0];
    ref.twist.linear.y      = v_s[i][1];
    ref.twist.linear.z      = v_s[i][2];
    ref.use_linear_velocity = true;

    ref.individual_thrust.data.push_back(0);
    ref.individual_thrust.data.push_back(0);
    ref.individual_thrust.data.push_back(0);
    ref.individual_thrust.data.push_back(0);

    // Fill all states reference (adjust for aproximate full model)
    if (i > 0 && i < (int)t_s.size() - 1) {
      Eigen::Vector3d acceleration;
      acceleration << a_s[i][0], a_s[i][1], a_s[i][2];
      Eigen::Vector3d last_acceleration;
      last_acceleration << a_s[i - 1][0], a_s[i - 1][1], a_s[i - 1][2];

      Eigen::Matrix3d rotation_matrix      = generateRotationMatrix(acceleration, end_waypoint.heading);
      Eigen::Matrix3d last_rotation_matrix = generateRotationMatrix(last_acceleration, end_waypoint.heading);

      Eigen::Quaterniond q   = Eigen::Quaterniond(Eigen::AngleAxisd(end_waypoint.heading, Eigen::Vector3d(0.0, 0.0, 1.0).normalized()));
      ref.pose.orientation.w = q.w();
      ref.pose.orientation.x = q.x();
      ref.pose.orientation.y = q.y();
      ref.pose.orientation.z = q.z();
      ref.use_orientation    = true;

      Eigen::Matrix3d dot_rotation_matrix = (rotation_matrix - last_rotation_matrix) / pmm_trajectory_capsule_.sampling_step;
      Eigen::Matrix3d s_matrix            = rotation_matrix.transpose() * dot_rotation_matrix;

      current_omega(0) = ref.twist.angular.x = s_matrix(2, 1);
      current_omega(1) = ref.twist.angular.y = s_matrix(0, 2);
      current_omega(2) = ref.twist.angular.z = s_matrix(1, 0);
      ref.use_angular_velocity               = true;

      Eigen::Vector4d individual_thrust = generateIndividualThrust(acceleration, current_omega);
      ref.individual_thrust.data[0]     = individual_thrust(0);
      ref.individual_thrust.data[1]     = individual_thrust(1);
      ref.individual_thrust.data[2]     = individual_thrust(2);
      ref.individual_thrust.data[3]     = individual_thrust(3);
      ref.use_individual_thrust         = true;

      last_omega = current_omega;
    } else {
      if (i != 0 || t_s.size() == 1) {
        Eigen::Quaterniond q   = Eigen::Quaterniond(Eigen::AngleAxisd(end_waypoint.heading, Eigen::Vector3d(0.0, 0.0, 1.0).normalized()));
        ref.pose.orientation.w = q.w();
        ref.pose.orientation.x = q.x();
        ref.pose.orientation.y = q.y();
        ref.pose.orientation.z = q.z();
      } else {
        ref.pose.orientation = start_waypoint.pose.orientation;
      }
      ref.use_orientation = true;

      ref.use_angular_velocity  = false;
      ref.use_individual_thrust = false;
    }

    full_trajectory_path_.push_back(ref);
  }

  total_waypoints_ = t_s.size();

  current_waypoint_ = 0;

  return true;
}
//}

/* generateTrajectory() //{ */
bool AgilePlanner::generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, std::vector<laser_msgs::msg::PoseWithHeading> waypoints, float speed) {
  full_trajectory_path_.clear();
  full_trajectory_path_.shrink_to_fit();

  pmm::Vector<3>              start_position;
  pmm::Vector<3>              start_velocity;
  pmm::Vector<3>              end_velocity;
  std::vector<pmm::Vector<3>> waypoints_mp;

  start_position[0] = start_waypoint.pose.position.x;
  start_position[1] = start_waypoint.pose.position.y;
  start_position[2] = start_waypoint.pose.position.z;
  start_velocity[0] = start_waypoint.twist.linear.x;
  start_velocity[1] = start_waypoint.twist.linear.y;
  start_velocity[2] = start_waypoint.twist.linear.z;

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

  pmm::PMM_MG_Trajectory3D mp_tr(waypoints_mp, start_velocity, end_velocity, pmm_trajectory_capsule_.max_acc_norm,
                                 (pmm::Scalar)std::min(speed, (float)pmm_trajectory_capsule_.max_vel_norm), pmm_trajectory_capsule_.dt_precision,
                                 pmm_trajectory_capsule_.first_run_max_iter, pmm_trajectory_capsule_.first_run_alpha,
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

  /* int             j = -1; */
  int             j = 0;
  Eigen::Vector3d last_omega(0.0, 0.0, 0.0);
  Eigen::Vector3d current_omega(0.0, 0.0, 0.0);
  for (auto i = 0; i < (int)t_s.size(); i++) {
    laser_msgs::msg::ReferenceState ref;

    ref.pose.position.x = p_s[i][0];
    ref.pose.position.y = p_s[i][1];
    ref.pose.position.z = p_s[i][2];
    ref.use_position    = true;

    ref.twist.linear.x      = v_s[i][0];
    ref.twist.linear.y      = v_s[i][1];
    ref.twist.linear.z      = v_s[i][2];
    ref.use_linear_velocity = true;

    ref.individual_thrust.data.push_back(0);
    ref.individual_thrust.data.push_back(0);
    ref.individual_thrust.data.push_back(0);
    ref.individual_thrust.data.push_back(0);

    // Fill all states reference (adjust for aproximate full model)
    if (i > 0 && i < (int)t_s.size() - 1) {
      Eigen::Vector3d acceleration;
      acceleration << a_s[i][0], a_s[i][1], a_s[i][2];
      Eigen::Vector3d last_acceleration;
      last_acceleration << a_s[i - 1][0], a_s[i - 1][1], a_s[i - 1][2];

      Eigen::Matrix3d rotation_matrix      = generateRotationMatrix(acceleration, waypoints[j].heading);
      Eigen::Matrix3d last_rotation_matrix = generateRotationMatrix(last_acceleration, waypoints[j + 1].heading);

      if (sqrt(pow(waypoints[j].position.x - p_s[i][0], 2) + pow(waypoints[j].position.y - p_s[i][1], 2) + pow(waypoints[j].position.z - p_s[i][2], 2)) <=
          0.001) {
        j++;
      }
      Eigen::Quaterniond q   = Eigen::Quaterniond(Eigen::AngleAxisd(waypoints[j].heading, Eigen::Vector3d(0.0, 0.0, 1.0).normalized()));
      ref.pose.orientation.w = q.w();
      ref.pose.orientation.x = q.x();
      ref.pose.orientation.y = q.y();
      ref.pose.orientation.z = q.z();
      ref.use_orientation    = true;

      Eigen::Matrix3d dot_rotation_matrix = (rotation_matrix - last_rotation_matrix) / pmm_trajectory_capsule_.sampling_step;
      Eigen::Matrix3d s_matrix            = rotation_matrix.transpose() * dot_rotation_matrix;

      current_omega(0) = ref.twist.angular.x = s_matrix(2, 1);
      current_omega(1) = ref.twist.angular.y = s_matrix(0, 2);
      current_omega(2) = ref.twist.angular.z = s_matrix(1, 0);
      ref.use_angular_velocity               = true;

      Eigen::Vector4d individual_thrust = generateIndividualThrust(acceleration, current_omega);
      ref.individual_thrust.data[0]     = individual_thrust(0);
      ref.individual_thrust.data[1]     = individual_thrust(1);
      ref.individual_thrust.data[2]     = individual_thrust(2);
      ref.individual_thrust.data[3]     = individual_thrust(3);
      ref.use_individual_thrust         = true;

      last_omega = current_omega;
    } else {
      if (i != 0 || t_s.size() == 1) {
        Eigen::Quaterniond q   = Eigen::Quaterniond(Eigen::AngleAxisd(waypoints[waypoints.size() - 1].heading, Eigen::Vector3d(0.0, 0.0, 1.0).normalized()));
        ref.pose.orientation.w = q.w();
        ref.pose.orientation.x = q.x();
        ref.pose.orientation.y = q.y();
        ref.pose.orientation.z = q.z();
      } else {
        ref.pose.orientation = start_waypoint.pose.orientation;
      }
      ref.use_orientation = true;

      ref.use_angular_velocity  = false;
      ref.use_individual_thrust = false;
    }

    full_trajectory_path_.push_back(ref);
  }

  total_waypoints_ = t_s.size();

  current_waypoint_ = 0;

  return true;
}
//}

/* getTrajectory() //{ */
std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points) {
  if (current_waypoint_ != total_waypoints_ && full_trajectory_path_.size() > 1) {
    full_trajectory_path_.erase(full_trajectory_path_.begin());
  }

  if ((int)full_trajectory_path_.size() >= qty_points) {
    return std::vector<laser_msgs::msg::ReferenceState>(full_trajectory_path_.begin(), full_trajectory_path_.begin() + qty_points);
  } else {
    while ((int)full_trajectory_path_.size() < qty_points) {
      full_trajectory_path_.push_back(full_trajectory_path_[(int)full_trajectory_path_.size() - 1]);
    }

    return full_trajectory_path_;
  }
}
//}

/* isHover() //{ */
bool AgilePlanner::isHover() {
  if (sqrt(pow(full_trajectory_path_[0].pose.position.x - full_trajectory_path_[full_trajectory_path_.size() - 1].pose.position.x, 2) +
           pow(full_trajectory_path_[0].pose.position.y - full_trajectory_path_[full_trajectory_path_.size() - 1].pose.position.y, 2) +
           pow(full_trajectory_path_[0].pose.position.z - full_trajectory_path_[full_trajectory_path_.size() - 1].pose.position.z, 2)) == 0.0) {
    return true;
  } else {
    return false;
  }
}
//}
}  // namespace laser_uav_planners
