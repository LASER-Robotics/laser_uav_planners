#include <laser_uav_planner/common.hpp>
#include <laser_uav_planner/pmm_trajectory3d.hpp>
#include <laser_uav_planner/pmm_mg_trajectory3d.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <laser_msgs/msg/reference_state.hpp>

namespace laser_uav_planner
{

struct pmm_t
{
  pmm::Vector<3> max_velocity;
  pmm::Scalar    max_acc_norm;
  pmm::Scalar    max_vel_norm;
  pmm::Vector<3> drag;
  bool           use_drag;
  pmm::Scalar    thrust_decomp_acc_precision;
  int            thrust_decomp_max_iter;
  pmm::Scalar    first_run_alpha;
  pmm::Scalar    first_run_alpha_reduction_factor;
  pmm::Scalar    first_run_alpha_min_threshold;
  int            first_run_max_iter;
  bool           run_second_opt;
  pmm::Scalar    second_run_alpha;
  pmm::Scalar    second_run_alpha_reduction_factor;
  pmm::Scalar    second_run_alpha_min_threshold;
  int            second_run_max_iter;
  pmm::Scalar    dt_precision;
  pmm::Scalar    sampling_step;
};

class AgilePlanner {
public:
  AgilePlanner();
  AgilePlanner(pmm_t pmm_params);

  bool generateTrajectory(laser_msgs::msg::ReferenceState start_waypoint, laser_msgs::msg::ReferenceState end_waypoint);

  std::vector<laser_msgs::msg::ReferenceState> getTrajectory(int qty_points, nav_msgs::msg::Odometry odometry);
  laser_msgs::msg::ReferenceState updateReference(nav_msgs::msg::Odometry odometry);

private:
  int                                          total_waypoints_;
  int                                          current_waypoint_;
  std::vector<laser_msgs::msg::ReferenceState> full_trajectory_path_;
  pmm_t                                        pmm_trajectory_capsule_;
};
}  // namespace laser_uav_planner
