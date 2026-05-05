#include <numeric>

#include <laser_uav_planners/common.hpp>
#include <laser_uav_planners/pmm_trajectory3d.hpp>
#include <laser_uav_planners/pmm_mg_trajectory3d.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <laser_msgs/msg/reference_state.hpp>
#include <laser_msgs/msg/pose_with_heading.hpp>

namespace laser_uav_planners
{

///* multirotor_t //{ */
struct multirotor_t
{
  double          mass;
  int             n_motors;
  Eigen::MatrixXd G1;
  Eigen::Matrix3d inertia_matrix;
};
//}

/* pmm_t //{ */
struct pmm_t
{
  pmm::Scalar max_accel_norm;
  pmm::Scalar max_vel_norm;
  pmm::Scalar default_vel_norm;
  bool        use_drag;
  pmm::Scalar thrust_decomp_acc_precision;
  int         thrust_decomp_max_iter;
  pmm::Scalar first_run_alpha;
  pmm::Scalar first_run_alpha_reduction_factor;
  pmm::Scalar first_run_alpha_min_threshold;
  int         first_run_max_iter;
  bool        run_second_opt;
  pmm::Scalar second_run_alpha;
  pmm::Scalar second_run_alpha_reduction_factor;
  pmm::Scalar second_run_alpha_min_threshold;
  int         second_run_max_iter;
  pmm::Scalar dt_precision;
  pmm::Scalar sampling_step;
};
//}

class AgilePlanner {
public:
  AgilePlanner();
  AgilePlanner(multirotor_t multirotor_params, pmm_t pmm_params, double controller_dt);

  void generateTrajectory(nav_msgs::msg::Odometry start_waypoint, laser_msgs::msg::PoseWithHeading end_waypoint, float speed, bool use_speed);
  void generateTrajectory(nav_msgs::msg::Odometry start_waypoint, std::vector<laser_msgs::msg::PoseWithHeading> waypoints, float speed);

  std::vector<laser_msgs::msg::ReferenceState> getTrajectory(int qty_points, double);

  bool isHover();
  void setMass(double mass);

private:
  Eigen::Quaterniond getAttitudeReference(Eigen::Vector3d& acceleration, double yaw);

  std::vector<pmm::Scalar>                     trajectory_time_;
  std::vector<laser_msgs::msg::ReferenceState> full_trajectory_path_;
  laser_msgs::msg::ReferenceState              hover_wait_waypoint_;
  pmm_t                                        pmm_trajectory_capsule_;

  Eigen::Vector3d gravity{0.0, 0.0, -9.81};

  double          mass_;
  Eigen::MatrixXd G1_;
  double          controller_dt_;

  double start_trajectory_time_;

  bool take_anchor_time_{false};
  bool is_hover_{false};
  bool generating_trajectory_{false};
};
}  // namespace laser_uav_planners
