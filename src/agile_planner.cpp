#include <laser_uav_planners/agile_planner.hpp>

namespace laser_uav_planners
{
/* AgilePlanner() {default} //{ */
AgilePlanner::AgilePlanner() {
}
//}

/* AgilePlanner() //{ */
AgilePlanner::AgilePlanner(multirotor_t multirotor_params, pmm_t pmm_params) {
  pmm_trajectory_capsule_ = pmm_params;

  mass_ = multirotor_params.mass;

  G1_ = multirotor_params.G1;

  inertia_matrix_ = multirotor_params.inertia_matrix;
}
//}

/* generateRotationMatrix() //{ */
Eigen::Matrix3d AgilePlanner::generateRotationMatrix(Eigen::Vector3d& acceleration) {
  Eigen::Vector3d       g(0.0, 0.0, -9.81);
  const Eigen::Vector3d thrust = mass_ * (acceleration - g);

  const Eigen::Vector3d z_b_des = thrust.normalized();

  const Eigen::Vector3d x_c(1.0, 0.0, 0.0);

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
Eigen::VectorXd AgilePlanner::generateIndividualThrust(Eigen::Vector3d& acceleration, Eigen::Vector3d& current_omega) {
  Eigen::Vector3d g(0.0, 0.0, -9.81);
  Eigen::Vector3d thrust       = mass_ * (acceleration - g);
  double          total_thrust = thrust.norm();

  Eigen::Vector3d torque = current_omega.cross(inertia_matrix_ * current_omega);

  Eigen::Vector4d wrench;
  wrench << total_thrust, torque;

  return G1_.inverse() * wrench;
}
//}

/* processImpulse() //{ */
void AgilePlanner::processImpulse(int window) {
  std::vector<laser_msgs::msg::ReferenceState> aux = full_trajectory_path_;
  int                                          n   = aux.size();

  for (int i = 0; i < n; ++i) {
    int start = std::max(0, i - window / 2);
    int end   = std::min(n, i + window / 2 + 1);

    double sum_angular_velocity_x =
        std::accumulate(aux.begin() + start, aux.begin() + end, 0.0, [](double sum, const auto& msg) { return sum + msg.twist.angular.x; });

    double sum_angular_velocity_y =
        std::accumulate(aux.begin() + start, aux.begin() + end, 0.0, [](double sum, const auto& msg) { return sum + msg.twist.angular.y; });

    double sum_angular_velocity_z =
        std::accumulate(aux.begin() + start, aux.begin() + end, 0.0, [](double sum, const auto& msg) { return sum + msg.twist.angular.z; });

    full_trajectory_path_[i].twist.angular.x = sum_angular_velocity_x / (end - start);
    full_trajectory_path_[i].twist.angular.y = sum_angular_velocity_y / (end - start);
    full_trajectory_path_[i].twist.angular.z = sum_angular_velocity_z / (end - start);

    for (auto j = 0; j < (int)aux[i].individual_thrust.data.size(); j++) {
      double sum_individual_thrust =
          std::accumulate(aux.begin() + start, aux.begin() + end, 0.0, [&](double sum, const auto& msg) { return sum + msg.individual_thrust.data[j]; });
      full_trajectory_path_[i].individual_thrust.data[j] = sum_individual_thrust / (end - start);
    }
  }
}
//}

/* generateTrajectory() //{ */
bool AgilePlanner::generateTrajectory(nav_msgs::msg::Odometry start_waypoint, laser_msgs::msg::PoseWithHeading end_waypoint, float speed, bool use_speed) {
  current_anchor_time_ = -1.0;
  full_trajectory_path_.clear();
  full_trajectory_path_.shrink_to_fit();

  pmm::Vector<3>              start_position;
  pmm::Vector<3>              start_velocity;
  pmm::Vector<3>              mid_position;
  pmm::Vector<3>              end_position;
  pmm::Vector<3>              end_velocity;
  std::vector<pmm::Vector<3>> waypoints;

  start_position[0] = start_waypoint.pose.pose.position.x;
  start_position[1] = start_waypoint.pose.pose.position.y;
  start_position[2] = start_waypoint.pose.pose.position.z;
  start_velocity[0] = start_waypoint.twist.twist.linear.x;
  start_velocity[1] = start_waypoint.twist.twist.linear.y;
  start_velocity[2] = start_waypoint.twist.twist.linear.z;

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

  /* std::vector<pmm::Scalar>    t_s; */
  std::vector<pmm::Vector<3>> p_s;
  std::vector<pmm::Vector<3>> v_s;
  std::vector<pmm::Vector<3>> a_s;

  std::tie(trajectory_time_, p_s, v_s, a_s) = mp_tr.get_sampled_trajectory(pmm_trajectory_capsule_.sampling_step);

  Eigen::Vector3d last_omega(0.0, 0.0, 0.0);
  Eigen::Vector3d current_omega(0.0, 0.0, 0.0);
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

    // Fill all states reference (adjust for aproximate full model)
    if (i > 0 && i < (int)trajectory_time_.size() - 1) {
      Eigen::Vector3d acceleration;
      acceleration << a_s[i][0], a_s[i][1], a_s[i][2];
      Eigen::Vector3d last_acceleration;
      last_acceleration << a_s[i - 1][0], a_s[i - 1][1], a_s[i - 1][2];

      Eigen::Matrix3d rotation_matrix      = generateRotationMatrix(acceleration);
      Eigen::Matrix3d last_rotation_matrix = generateRotationMatrix(last_acceleration);

      Eigen::Quaterniond q   = Eigen::Quaterniond(Eigen::AngleAxisd(end_waypoint.heading, Eigen::Vector3d(0.0, 0.0, 1.0))).normalized();
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

      Eigen::VectorXd individual_thrust = generateIndividualThrust(acceleration, current_omega);
      ref.individual_thrust.data        = std::vector<double>(individual_thrust.data(), individual_thrust.data() + individual_thrust.size());
      ref.use_individual_thrust         = true;

      last_omega = current_omega;
    } else {
      if (i != 0) {
        Eigen::Quaterniond q   = Eigen::Quaterniond(Eigen::AngleAxisd(end_waypoint.heading, Eigen::Vector3d(0.0, 0.0, 1.0))).normalized();
        ref.pose.orientation.w = q.w();
        ref.pose.orientation.x = q.x();
        ref.pose.orientation.y = q.y();
        ref.pose.orientation.z = q.z();
      } else {
        ref.pose.orientation = start_waypoint.pose.pose.orientation;
      }
      ref.use_orientation = true;

      ref.use_angular_velocity = false;

      ref.individual_thrust.data = std::vector<double>(G1_.cols());
      ref.use_individual_thrust  = false;
    }

    full_trajectory_path_.push_back(ref);
  }
  processImpulse(200);

  total_waypoints_ = trajectory_time_.size();

  current_waypoint_ = 0;

  return true;
}
//}

/* generateTrajectory() //{ */
bool AgilePlanner::generateTrajectory(nav_msgs::msg::Odometry start_waypoint, std::vector<laser_msgs::msg::PoseWithHeading> waypoints, float speed) {

  current_anchor_time_ = -1.0;
  full_trajectory_path_.clear();
  full_trajectory_path_.shrink_to_fit();

  pmm::Vector<3>              start_position;
  pmm::Vector<3>              start_velocity;
  pmm::Vector<3>              end_velocity;
  std::vector<pmm::Vector<3>> waypoints_mp;

  start_position[0] = start_waypoint.pose.pose.position.x;
  start_position[1] = start_waypoint.pose.pose.position.y;
  start_position[2] = start_waypoint.pose.pose.position.z;
  start_velocity[0] = start_waypoint.twist.twist.linear.x;
  start_velocity[1] = start_waypoint.twist.twist.linear.y;
  start_velocity[2] = start_waypoint.twist.twist.linear.z;

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

  /* std::vector<pmm::Scalar>    t_s; */
  std::vector<pmm::Vector<3>> p_s;
  std::vector<pmm::Vector<3>> v_s;
  std::vector<pmm::Vector<3>> a_s;

  std::tie(trajectory_time_, p_s, v_s, a_s) = mp_tr.get_sampled_trajectory(pmm_trajectory_capsule_.sampling_step);

  int             j = 0;
  Eigen::Vector3d last_omega(0.0, 0.0, 0.0);
  Eigen::Vector3d current_omega(0.0, 0.0, 0.0);
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

    // Fill all states reference (adjust for aproximate full model)
    if (i > 0 && i < (int)trajectory_time_.size() - 1) {
      Eigen::Vector3d acceleration;
      acceleration << a_s[i][0], a_s[i][1], a_s[i][2];
      Eigen::Vector3d last_acceleration;
      last_acceleration << a_s[i - 1][0], a_s[i - 1][1], a_s[i - 1][2];

      Eigen::Matrix3d rotation_matrix      = generateRotationMatrix(acceleration);
      Eigen::Matrix3d last_rotation_matrix = generateRotationMatrix(last_acceleration);

      Eigen::Quaterniond q   = Eigen::Quaterniond(Eigen::AngleAxisd(waypoints[j].heading, Eigen::Vector3d(0.0, 0.0, 1.0))).normalized();
      ref.pose.orientation.w = q.w();
      ref.pose.orientation.x = q.x();
      ref.pose.orientation.y = q.y();
      ref.pose.orientation.z = q.z();
      ref.use_orientation    = true;

      if (sqrt(pow(waypoints[j].position.x - p_s[i][0], 2) + pow(waypoints[j].position.y - p_s[i][1], 2) + pow(waypoints[j].position.z - p_s[i][2], 2)) <=
          0.1) {
        j++;
      }

      Eigen::Matrix3d dot_rotation_matrix = (rotation_matrix - last_rotation_matrix) / pmm_trajectory_capsule_.sampling_step;
      Eigen::Matrix3d s_matrix            = rotation_matrix.transpose() * dot_rotation_matrix;

      current_omega(0) = ref.twist.angular.x = s_matrix(2, 1);
      current_omega(1) = ref.twist.angular.y = s_matrix(0, 2);
      current_omega(2) = ref.twist.angular.z = s_matrix(1, 0);
      ref.use_angular_velocity               = true;

      Eigen::VectorXd individual_thrust = generateIndividualThrust(acceleration, current_omega);
      ref.individual_thrust.data        = std::vector<double>(individual_thrust.data(), individual_thrust.data() + individual_thrust.size());
      ref.use_individual_thrust         = true;

      last_omega = current_omega;
    } else {
      Eigen::Quaterniond q;
      if (i == 0) {
        q = Eigen::Quaterniond(Eigen::AngleAxisd(waypoints[j].heading, Eigen::Vector3d(0.0, 0.0, 1.0))).normalized();
      } else {
        q = Eigen::Quaterniond(Eigen::AngleAxisd(waypoints[j - 1].heading, Eigen::Vector3d(0.0, 0.0, 1.0))).normalized();
      }

      ref.pose.orientation.w = q.w();
      ref.pose.orientation.x = q.x();
      ref.pose.orientation.y = q.y();
      ref.pose.orientation.z = q.z();
      ref.use_orientation    = true;

      ref.use_angular_velocity = false;

      ref.individual_thrust.data = std::vector<double>(G1_.cols());
      ref.use_individual_thrust  = false;
    }

    full_trajectory_path_.push_back(ref);
  }
  processImpulse(200);

  total_waypoints_ = trajectory_time_.size();

  current_waypoint_ = 0;

  return true;
}
//}

/* getTrajectory() //{ */
/* std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points) { */
/*   if (current_waypoint_ != total_waypoints_ && full_trajectory_path_.size() > 1) { */
/*     trajectory_time_.erase(trajectory_time_.begin()); */
/*     full_trajectory_path_.erase(full_trajectory_path_.begin()); */
/*   } */

/*   if ((int)full_trajectory_path_.size() >= qty_points) { */
/*     auto                                         aux = 0; */
/*     float                                        dt  = 0.05; */
/*     std::vector<laser_msgs::msg::ReferenceState> sampled_trajectory; */
/*     sampled_trajectory_.push_back(full_trajectory_path_[0]); */
/*     for (auto i = 0; i < trajectory_time_.size(); i++) { */
/*       if (std::abs(trajectory_time_[i] - trajectory_time[i + 1]) >= 0.05) { */
/*         sampled_trajectoy_.push_back(full_trajectory_path_[i + 1]); */
/*       } */
/*     } */
/*     return std::vector<laser_msgs::msg::ReferenceState>(full_trajectory_path_.begin(), full_trajectory_path_.begin() + qty_points); */
/*   } else { */
/*     while ((int)full_trajectory_path_.size() < qty_points) { */
/*       full_trajectory_path_.push_back(full_trajectory_path_[(int)full_trajectory_path_.size() - 1]); */
/*     } */

/*     return full_trajectory_path_; */
/*   } */
/* } */
/* std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points) { */
/*   // Avança a trajetória descartando o ponto passado (se não for o waypoint final) */
/*   if (current_waypoint_ != total_waypoints_ && full_trajectory_path_.size() > 1) { */
/*     trajectory_time_.erase(trajectory_time_.begin()); */
/*     full_trajectory_path_.erase(full_trajectory_path_.begin()); */
/*   } */

/*   std::vector<laser_msgs::msg::ReferenceState> sampled_trajectory; */

/*   // Se houver caminho disponível, faz a amostragem baseada no tempo */
/*   if (!full_trajectory_path_.empty() && !trajectory_time_.empty()) { */
/*     float dt = 0.05f; */
    
/*     // Adiciona o ponto atual como início do horizonte */
/*     sampled_trajectory.push_back(full_trajectory_path_[0]); */
/*     double last_sampled_time = trajectory_time_[0]; */

/*     // Itera pelo caminho, respeitando o dt, até encher o horizonte (qty_points) */
/*     for (size_t i = 1; i < trajectory_time_.size() && (int)sampled_trajectory.size() < qty_points; ++i) { */
/*       if (std::abs(trajectory_time_[i] - last_sampled_time) >= dt) { */
/*         sampled_trajectory.push_back(full_trajectory_path_[i]); */
/*         last_sampled_time = trajectory_time_[i]; // Atualiza o tempo de referência */
/*         std::cout << last_sampled_time << std::endl; */
/*       } */
/*     } */
/*   } */

/*   // Preenche (pad) com o último estado de referência caso não tenha atingido qty_points */
/*   // Isso garante que o horizonte de controle mantenha o tamanho constante */
/*   while ((int)sampled_trajectory.size() < qty_points) { */
/*     if (!sampled_trajectory.empty()) { */
/*       // Repete o último estado da amostragem (Drone deve manter a posição no final do trajeto) */
/*       sampled_trajectory.push_back(sampled_trajectory.back()); */
/*     } else { */
/*       // Fallback de segurança se as listas globais estiverem vazias */
/*       sampled_trajectory.push_back(laser_msgs::msg::ReferenceState()); */
/*     } */
/*   } */

/*   return sampled_trajectory; */
/* } */
/* std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points) { */
/*   // Avança a trajetória descartando o ponto passado (se não for o waypoint final) */
/*   if (current_waypoint_ != total_waypoints_ && full_trajectory_path_.size() > 1) { */
/*     trajectory_time_.erase(trajectory_time_.begin()); */
/*     full_trajectory_path_.erase(full_trajectory_path_.begin()); */
/*   } */

/*   std::vector<laser_msgs::msg::ReferenceState> sampled_trajectory; */

/*   if (!full_trajectory_path_.empty() && !trajectory_time_.empty()) { */
/*     float dt = 0.05f; */
    
/*     // Consideramos o índice 0 como o tempo de "onde estou agora" */
/*     double last_sampled_time = trajectory_time_[0] + dt; */

/*     // Começamos do índice 1 e procuramos o primeiro ponto que esteja a pelo menos 'dt' de distância. */
/*     // Assim, o primeiro elemento do sampled_trajectory será exatamente o (agora + dt). */
/*     for (size_t i = 1; i < trajectory_time_.size() && (int)sampled_trajectory.size() < qty_points; ++i) { */
/*       if ((trajectory_time_[i] - last_sampled_time) >= dt) { */
/*         sampled_trajectory.push_back(full_trajectory_path_[i]); */
/*         last_sampled_time = trajectory_time_[i]; // Atualiza a âncora */
/*         std::cout << last_sampled_time << std::endl; */
/*       } */
/*     } */

/*     // Failsafe: Se o drone estiver tão perto do final que o tempo restante da trajetória */
/*     // é menor que 0.05s, o loop acima não adicionará nada. Garantimos pelo menos 1 ponto. */
/*     if (sampled_trajectory.empty()) { */
/*       sampled_trajectory.push_back(full_trajectory_path_.back()); */
/*     } */
/*   } */

/*   // Preenchimento (Padding) para garantir a matriz de tamanho constante para o solver */
/*   while ((int)sampled_trajectory.size() < qty_points) { */
/*     if (!sampled_trajectory.empty()) { */
/*       // Repete o último estado da amostragem (mantém o hover no final) */
/*       sampled_trajectory.push_back(sampled_trajectory.back()); */
/*     } else { */
/*       // Fallback de segurança extremo */
/*       sampled_trajectory.push_back(laser_msgs::msg::ReferenceState()); */
/*     } */
/*   } */

/*   return sampled_trajectory; */
/* } */
/* #include <cmath>     // Para std::sqrt e std::abs */
/* #include <algorithm> // Para std::lower_bound */
void AgilePlanner::resetPlannerTime() {
  is_first_call_ = true;
}

/* std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points, double current_ros_time) { */
/*   std::vector<laser_msgs::msg::ReferenceState> sampled_trajectory; */

/*   if (full_trajectory_path_.empty() || trajectory_time_.empty()) { */
/*     return std::vector<laser_msgs::msg::ReferenceState>(qty_points, laser_msgs::msg::ReferenceState()); */
/*   } */

/*   // --- A MÁGICA DO TEMPO ZERO --- */
/*   if (is_first_call_) { */
/*     // Na primeiríssima vez que o NMPC pedir a referência, nós cravamos */
/*     // o relógio da missão usando o tempo exato desse milissegundo. */
/*     start_sim_time_ = current_ros_time; */
/*     is_first_call_ = false; */
/*   } */

/*   // O tempo decorrido agora é calculado internamente. */
/*   // Na primeira chamada, elapsed_time será matematicamente = 0.0 */
/*   double elapsed_time = current_ros_time - start_sim_time_; */
/*   // ------------------------------ */

/*   double dt = 0.05; */
/*   double delay_compensation = 0.05; // Opcional: Lookahead para compensar o solver */

/*   for (int k = 0; k < qty_points; ++k) { */
/*     // Na primeira chamada (elapsed_time = 0), o alvo do NMPC no nó k=0 */ 
/*     // será exatamente o tempo 0.0 da trajetória (+ o lookahead se usar). */
/*     double target_time = elapsed_time + delay_compensation + (k * dt); */

/*     auto it = std::lower_bound(trajectory_time_.begin(), trajectory_time_.end(), target_time); */

/*     // Condição: Antes do início (Segurança) */
/*     if (it == trajectory_time_.begin()) { */
/*       sampled_trajectory.push_back(full_trajectory_path_.front()); */
/*       continue; */
/*     } */

/*     // Condição: Fim da trajetória (Mantém Hover final) */
/*     if (it == trajectory_time_.end()) { */
/*       sampled_trajectory.push_back(full_trajectory_path_.back()); */
/*       continue; */
/*     } */

/*     // Interpolação Matemática */
/*     int idx_next = std::distance(trajectory_time_.begin(), it); */
/*     int idx_prev = idx_next - 1; */

/*     double t_prev = trajectory_time_[idx_prev]; */
/*     double t_next = trajectory_time_[idx_next]; */
/*     double alpha = (target_time - t_prev) / (t_next - t_prev); */

/*     auto state_prev = full_trajectory_path_[idx_prev]; */
/*     auto state_next = full_trajectory_path_[idx_next]; */
    
/*     laser_msgs::msg::ReferenceState interp_state; */
/*     interp_state.use_position = state_prev.use_position; */
/*     interp_state.use_orientation = state_prev.use_orientation; */
/*     interp_state.use_linear_velocity = state_prev.use_linear_velocity; */
/*     interp_state.use_angular_velocity = state_prev.use_angular_velocity; */
/*     interp_state.use_individual_thrust = state_prev.use_individual_thrust; */

/*     // Posição */
/*     interp_state.pose.position.x = state_prev.pose.position.x + alpha * (state_next.pose.position.x - state_prev.pose.position.x); */
/*     interp_state.pose.position.y = state_prev.pose.position.y + alpha * (state_next.pose.position.y - state_prev.pose.position.y); */
/*     interp_state.pose.position.z = state_prev.pose.position.z + alpha * (state_next.pose.position.z - state_prev.pose.position.z); */

/*     // Orientação (NLERP com Shortest Path) */
/*     double q_dot = state_prev.pose.orientation.w * state_next.pose.orientation.w + */
/*                    state_prev.pose.orientation.x * state_next.pose.orientation.x + */
/*                    state_prev.pose.orientation.y * state_next.pose.orientation.y + */
/*                    state_prev.pose.orientation.z * state_next.pose.orientation.z; */

/*     double q_w = state_next.pose.orientation.w; */
/*     double q_x = state_next.pose.orientation.x; */
/*     double q_y = state_next.pose.orientation.y; */
/*     double q_z = state_next.pose.orientation.z; */

/*     if (q_dot < 0.0) { q_w = -q_w; q_x = -q_x; q_y = -q_y; q_z = -q_z; } */

/*     interp_state.pose.orientation.w = state_prev.pose.orientation.w + alpha * (q_w - state_prev.pose.orientation.w); */
/*     interp_state.pose.orientation.x = state_prev.pose.orientation.x + alpha * (q_x - state_prev.pose.orientation.x); */
/*     interp_state.pose.orientation.y = state_prev.pose.orientation.y + alpha * (q_y - state_prev.pose.orientation.y); */
/*     interp_state.pose.orientation.z = state_prev.pose.orientation.z + alpha * (q_z - state_prev.pose.orientation.z); */

/*     double norm = std::sqrt(interp_state.pose.orientation.w * interp_state.pose.orientation.w + */
/*                             interp_state.pose.orientation.x * interp_state.pose.orientation.x + */
/*                             interp_state.pose.orientation.y * interp_state.pose.orientation.y + */
/*                             interp_state.pose.orientation.z * interp_state.pose.orientation.z); */
    
/*     interp_state.pose.orientation.w /= norm; interp_state.pose.orientation.x /= norm; */
/*     interp_state.pose.orientation.y /= norm; interp_state.pose.orientation.z /= norm; */

/*     // Twist Linear e Angular */
/*     interp_state.twist.linear.x = state_prev.twist.linear.x + alpha * (state_next.twist.linear.x - state_prev.twist.linear.x); */
/*     interp_state.twist.linear.y = state_prev.twist.linear.y + alpha * (state_next.twist.linear.y - state_prev.twist.linear.y); */
/*     interp_state.twist.linear.z = state_prev.twist.linear.z + alpha * (state_next.twist.linear.z - state_prev.twist.linear.z); */

/*     interp_state.twist.angular.x = state_prev.twist.angular.x + alpha * (state_next.twist.angular.x - state_prev.twist.angular.x); */
/*     interp_state.twist.angular.y = state_prev.twist.angular.y + alpha * (state_next.twist.angular.y - state_prev.twist.angular.y); */
/*     interp_state.twist.angular.z = state_prev.twist.angular.z + alpha * (state_next.twist.angular.z - state_prev.twist.angular.z); */

/*     // Thrust Individual */
/*     interp_state.individual_thrust.unit_of_measurement = state_prev.individual_thrust.unit_of_measurement; */
/*     if (state_prev.individual_thrust.data.size() == state_next.individual_thrust.data.size()) { */
/*       for (size_t i = 0; i < state_prev.individual_thrust.data.size(); ++i) { */
/*         double interpolated_thrust = state_prev.individual_thrust.data[i] + alpha * (state_next.individual_thrust.data[i] - state_prev.individual_thrust.data[i]); */
/*         interp_state.individual_thrust.data.push_back(interpolated_thrust); */
/*       } */
/*     } else { */
/*       interp_state.individual_thrust.data = state_prev.individual_thrust.data; */ 
/*     } */

/*     sampled_trajectory.push_back(interp_state); */
/*   } */

/*   // Descarte Lazy de Memória */
/*   while (trajectory_time_.size() > 2 && trajectory_time_[1] < elapsed_time - 1.0) { */
/*     trajectory_time_.erase(trajectory_time_.begin()); */
/*     full_trajectory_path_.erase(full_trajectory_path_.begin()); */
/*   } */

/*   return sampled_trajectory; */
/* } */
std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points, double current_ros_time) {
  std::vector<laser_msgs::msg::ReferenceState> sampled_trajectory;

  if (full_trajectory_path_.empty() || trajectory_time_.empty()) {
    return std::vector<laser_msgs::msg::ReferenceState>(qty_points, laser_msgs::msg::ReferenceState());
  }

  // --- A MÁGICA DO TEMPO ZERO ---
  if (is_first_call_) {
    // Na primeiríssima chamada, cravamos a âncora de tempo.
    start_sim_time_ = current_ros_time;
    is_first_call_ = false;
  }

  // Na primeira chamada, elapsed_time será exatamente 0.0
  // Nas chamadas seguintes, ele deslizará organicamente (ex: 0.012, 0.024...)
  double elapsed_time = current_ros_time - start_sim_time_;
  
  double dt = 0.05;

  for (int k = 0; k < qty_points; ++k) {
    // O alvo do NMPC agora é o tempo percorrido exato + os passos do horizonte.
    // Quando elapsed_time for 0.0 e k for 0, target_time será cravado em 0.0!
    double target_time = elapsed_time + (k * dt);

    auto it = std::lower_bound(trajectory_time_.begin(), trajectory_time_.end(), target_time);

    // Condição: Antes do início (Segurança)
    if (it == trajectory_time_.begin()) {
      sampled_trajectory.push_back(full_trajectory_path_.front());
      continue;
    }

    // Condição: Fim da trajetória (Mantém Hover final)
    if (it == trajectory_time_.end()) {
      sampled_trajectory.push_back(full_trajectory_path_.back());
      continue;
    }

    // Interpolação Matemática Contínua
    int idx_next = std::distance(trajectory_time_.begin(), it);
    int idx_prev = idx_next - 1;

    double t_prev = trajectory_time_[idx_prev];
    double t_next = trajectory_time_[idx_next];
    double alpha = (target_time - t_prev) / (t_next - t_prev);

    auto state_prev = full_trajectory_path_[idx_prev];
    auto state_next = full_trajectory_path_[idx_next];
    
    laser_msgs::msg::ReferenceState interp_state;
    interp_state.use_position = state_prev.use_position;
    interp_state.use_orientation = state_prev.use_orientation;
    interp_state.use_linear_velocity = state_prev.use_linear_velocity;
    interp_state.use_angular_velocity = state_prev.use_angular_velocity;
    interp_state.use_individual_thrust = state_prev.use_individual_thrust;

    // --- Posição ---
    interp_state.pose.position.x = state_prev.pose.position.x + alpha * (state_next.pose.position.x - state_prev.pose.position.x);
    interp_state.pose.position.y = state_prev.pose.position.y + alpha * (state_next.pose.position.y - state_prev.pose.position.y);
    interp_state.pose.position.z = state_prev.pose.position.z + alpha * (state_next.pose.position.z - state_prev.pose.position.z);

    // --- Orientação (NLERP com Shortest Path) ---
    double q_dot = state_prev.pose.orientation.w * state_next.pose.orientation.w +
                   state_prev.pose.orientation.x * state_next.pose.orientation.x +
                   state_prev.pose.orientation.y * state_next.pose.orientation.y +
                   state_prev.pose.orientation.z * state_next.pose.orientation.z;

    double q_w = state_next.pose.orientation.w;
    double q_x = state_next.pose.orientation.x;
    double q_y = state_next.pose.orientation.y;
    double q_z = state_next.pose.orientation.z;

    if (q_dot < 0.0) { q_w = -q_w; q_x = -q_x; q_y = -q_y; q_z = -q_z; }

    interp_state.pose.orientation.w = state_prev.pose.orientation.w + alpha * (q_w - state_prev.pose.orientation.w);
    interp_state.pose.orientation.x = state_prev.pose.orientation.x + alpha * (q_x - state_prev.pose.orientation.x);
    interp_state.pose.orientation.y = state_prev.pose.orientation.y + alpha * (q_y - state_prev.pose.orientation.y);
    interp_state.pose.orientation.z = state_prev.pose.orientation.z + alpha * (q_z - state_prev.pose.orientation.z);

    double norm = std::sqrt(interp_state.pose.orientation.w * interp_state.pose.orientation.w +
                            interp_state.pose.orientation.x * interp_state.pose.orientation.x +
                            interp_state.pose.orientation.y * interp_state.pose.orientation.y +
                            interp_state.pose.orientation.z * interp_state.pose.orientation.z);
    
    interp_state.pose.orientation.w /= norm; 
    interp_state.pose.orientation.x /= norm;
    interp_state.pose.orientation.y /= norm; 
    interp_state.pose.orientation.z /= norm;

    // --- Twist Linear e Angular ---
    interp_state.twist.linear.x = state_prev.twist.linear.x + alpha * (state_next.twist.linear.x - state_prev.twist.linear.x);
    interp_state.twist.linear.y = state_prev.twist.linear.y + alpha * (state_next.twist.linear.y - state_prev.twist.linear.y);
    interp_state.twist.linear.z = state_prev.twist.linear.z + alpha * (state_next.twist.linear.z - state_prev.twist.linear.z);

    interp_state.twist.angular.x = state_prev.twist.angular.x + alpha * (state_next.twist.angular.x - state_prev.twist.angular.x);
    interp_state.twist.angular.y = state_prev.twist.angular.y + alpha * (state_next.twist.angular.y - state_prev.twist.angular.y);
    interp_state.twist.angular.z = state_prev.twist.angular.z + alpha * (state_next.twist.angular.z - state_prev.twist.angular.z);

    // --- Thrust Individual ---
    interp_state.individual_thrust.unit_of_measurement = state_prev.individual_thrust.unit_of_measurement;
    if (state_prev.individual_thrust.data.size() == state_next.individual_thrust.data.size()) {
      for (size_t i = 0; i < state_prev.individual_thrust.data.size(); ++i) {
        double interpolated_thrust = state_prev.individual_thrust.data[i] + alpha * (state_next.individual_thrust.data[i] - state_prev.individual_thrust.data[i]);
        interp_state.individual_thrust.data.push_back(interpolated_thrust);
      }
    } else {
      interp_state.individual_thrust.data = state_prev.individual_thrust.data; 
    }

    sampled_trajectory.push_back(interp_state);
  }

  // Descarte Lazy de Memória: Apaga apenas o que já passou há mais de 1 segundo
  while (trajectory_time_.size() > 2 && trajectory_time_[1] < elapsed_time - 1.0) {
    trajectory_time_.erase(trajectory_time_.begin());
    full_trajectory_path_.erase(full_trajectory_path_.begin());
  }

  return sampled_trajectory;
}

/* std::vector<laser_msgs::msg::ReferenceState> AgilePlanner::getTrajectory(int qty_points, double elapsed_time) { */
/*   std::cout << elapsed_time << std::endl; */
/*   std::vector<laser_msgs::msg::ReferenceState> sampled_trajectory; */

/*   if (full_trajectory_path_.empty() || trajectory_time_.empty()) { */
/*     return std::vector<laser_msgs::msg::ReferenceState>(qty_points, laser_msgs::msg::ReferenceState()); */
/*   } */

/*   double dt = 0.05; */
/*   double delay_compensation = 0.05; */ 

/*   for (int k = 0; k < qty_points; ++k) { */
/*     double target_time = elapsed_time + delay_compensation + (k * dt); */
/*     auto it = std::lower_bound(trajectory_time_.begin(), trajectory_time_.end(), target_time); */

/*     if (it == trajectory_time_.begin()) { */
/*       sampled_trajectory.push_back(full_trajectory_path_.front()); */
/*       continue; */
/*     } */
/*     if (it == trajectory_time_.end()) { */
/*       sampled_trajectory.push_back(full_trajectory_path_.back()); */
/*       continue; */
/*     } */

/*     int idx_next = std::distance(trajectory_time_.begin(), it); */
/*     int idx_prev = idx_next - 1; */

/*     double t_prev = trajectory_time_[idx_prev]; */
/*     double t_next = trajectory_time_[idx_next]; */
/*     double alpha = (target_time - t_prev) / (t_next - t_prev); */

/*     auto state_prev = full_trajectory_path_[idx_prev]; */
/*     auto state_next = full_trajectory_path_[idx_next]; */
    
/*     laser_msgs::msg::ReferenceState interp_state; */

/*     // 1. Copia as flags booleanas (não faz sentido interpolar booleanos) */
/*     interp_state.use_position = state_prev.use_position; */
/*     interp_state.use_orientation = state_prev.use_orientation; */
/*     interp_state.use_linear_velocity = state_prev.use_linear_velocity; */
/*     interp_state.use_angular_velocity = state_prev.use_angular_velocity; */
/*     interp_state.use_individual_thrust = state_prev.use_individual_thrust; */

/*     // 2. Interpolação de Posição (LERP) */
/*     interp_state.pose.position.x = state_prev.pose.position.x + alpha * (state_next.pose.position.x - state_prev.pose.position.x); */
/*     interp_state.pose.position.y = state_prev.pose.position.y + alpha * (state_next.pose.position.y - state_prev.pose.position.y); */
/*     interp_state.pose.position.z = state_prev.pose.position.z + alpha * (state_next.pose.position.z - state_prev.pose.position.z); */

/*     // 3. Interpolação de Orientação (NLERP com Shortest Path) */
/*     double q_dot = state_prev.pose.orientation.w * state_next.pose.orientation.w + */
/*                    state_prev.pose.orientation.x * state_next.pose.orientation.x + */
/*                    state_prev.pose.orientation.y * state_next.pose.orientation.y + */
/*                    state_prev.pose.orientation.z * state_next.pose.orientation.z; */

/*     double q_w = state_next.pose.orientation.w; */
/*     double q_x = state_next.pose.orientation.x; */
/*     double q_y = state_next.pose.orientation.y; */
/*     double q_z = state_next.pose.orientation.z; */

/*     // Se o produto escalar for negativo, os quaternions estão em hemisférios opostos. */
/*     // Invertemos o quaternion de destino para forçar a interpolação pelo caminho mais curto. */
/*     if (q_dot < 0.0) { */
/*       q_w = -q_w; q_x = -q_x; q_y = -q_y; q_z = -q_z; */
/*     } */

/*     interp_state.pose.orientation.w = state_prev.pose.orientation.w + alpha * (q_w - state_prev.pose.orientation.w); */
/*     interp_state.pose.orientation.x = state_prev.pose.orientation.x + alpha * (q_x - state_prev.pose.orientation.x); */
/*     interp_state.pose.orientation.y = state_prev.pose.orientation.y + alpha * (q_y - state_prev.pose.orientation.y); */
/*     interp_state.pose.orientation.z = state_prev.pose.orientation.z + alpha * (q_z - state_prev.pose.orientation.z); */

/*     // Normaliza o quaternion resultante */
/*     double norm = std::sqrt(interp_state.pose.orientation.w * interp_state.pose.orientation.w + */
/*                             interp_state.pose.orientation.x * interp_state.pose.orientation.x + */
/*                             interp_state.pose.orientation.y * interp_state.pose.orientation.y + */
/*                             interp_state.pose.orientation.z * interp_state.pose.orientation.z); */
    
/*     interp_state.pose.orientation.w /= norm; */
/*     interp_state.pose.orientation.x /= norm; */
/*     interp_state.pose.orientation.y /= norm; */
/*     interp_state.pose.orientation.z /= norm; */

/*     // 4. Interpolação de Velocidade Linear (LERP) */
/*     interp_state.twist.linear.x = state_prev.twist.linear.x + alpha * (state_next.twist.linear.x - state_prev.twist.linear.x); */
/*     interp_state.twist.linear.y = state_prev.twist.linear.y + alpha * (state_next.twist.linear.y - state_prev.twist.linear.y); */
/*     interp_state.twist.linear.z = state_prev.twist.linear.z + alpha * (state_next.twist.linear.z - state_prev.twist.linear.z); */

/*     // 5. Interpolação de Velocidade Angular (LERP) */
/*     interp_state.twist.angular.x = state_prev.twist.angular.x + alpha * (state_next.twist.angular.x - state_prev.twist.angular.x); */
/*     interp_state.twist.angular.y = state_prev.twist.angular.y + alpha * (state_next.twist.angular.y - state_prev.twist.angular.y); */
/*     interp_state.twist.angular.z = state_prev.twist.angular.z + alpha * (state_next.twist.angular.z - state_prev.twist.angular.z); */

/*     // 6. Interpolação de Thrust Individual (Array) */
/*     interp_state.individual_thrust.unit_of_measurement = state_prev.individual_thrust.unit_of_measurement; */
    
/*     // Confirma se os vetores de thrust têm o mesmo tamanho para evitar SegFault */
/*     if (state_prev.individual_thrust.data.size() == state_next.individual_thrust.data.size()) { */
/*       for (size_t i = 0; i < state_prev.individual_thrust.data.size(); ++i) { */
/*         double interpolated_thrust = state_prev.individual_thrust.data[i] + alpha * (state_next.individual_thrust.data[i] - state_prev.individual_thrust.data[i]); */
/*         interp_state.individual_thrust.data.push_back(interpolated_thrust); */
/*       } */
/*     } else { */
/*       // Fallback: se houver descasamento, copia do anterior */
/*       interp_state.individual_thrust.data = state_prev.individual_thrust.data; */ 
/*     } */

/*     sampled_trajectory.push_back(interp_state); */
/*   } */

/*   // Descarte de memória atrasada (mantém 1 segundo de histórico por segurança) */
/*   while (trajectory_time_.size() > 2 && trajectory_time_[1] < elapsed_time - 1.0) { */
/*     trajectory_time_.erase(trajectory_time_.begin()); */
/*     full_trajectory_path_.erase(full_trajectory_path_.begin()); */
/*   } */

/*   return sampled_trajectory; */
/* } */
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

/* setMass() //{ */
void AgilePlanner::setMass(double mass) {
  mass_ = mass;
}
//}
}  // namespace laser_uav_planners
