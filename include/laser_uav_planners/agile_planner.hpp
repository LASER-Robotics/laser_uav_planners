#include <laser_uav_planners/common.hpp>
#include <laser_uav_planners/pmm_trajectory3d.hpp>
#include <laser_uav_planners/pmm_mg_trajectory3d.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <laser_msgs/msg/reference_state.hpp>
#include <laser_msgs/msg/pose_with_heading.hpp>

namespace laser_uav_planners
{

///* quadrotor_t //{ */
struct quadrotor_t
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
  pmm::Scalar max_acc_norm;
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
  AgilePlanner(quadrotor_t quadrotor_params, pmm_t pmm_params);

  bool generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, laser_msgs::msg::PoseWithHeading end_waypoint, float speed, bool use_speed);
  bool generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, std::vector<laser_msgs::msg::PoseWithHeading> waypoints, float speed);

  std::vector<laser_msgs::msg::ReferenceState> getTrajectory(int qty_points);

  bool isHover();

private:
  Eigen::Matrix3d generateRotationMatrix(Eigen::Vector3d& acceleration, double desired_heading);
  Eigen::Vector4d generateIndividualThrust(Eigen::Vector3d& acceleration, Eigen::Vector3d& omega);

  int                                          total_waypoints_;
  int                                          current_waypoint_;
  std::vector<laser_msgs::msg::ReferenceState> full_trajectory_path_;
  pmm_t                                        pmm_trajectory_capsule_;

  double          mass_;
  Eigen::Matrix3d inertia_matrix_;
  Eigen::MatrixXd G1_;
};
}  // namespace laser_uav_planners
