#include "rclcpp/rclcpp.hpp"
#include "uav_custom_msgs/msg/uav_parameters.hpp"
#include "uav_custom_msgs/msg/waypoints.hpp"
#include "uav_custom_msgs/msg/trajectory.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <Eigen/Eigen>
#include <chrono>
#include <vector>

#include "pmm_trajectory3d.hpp"
#include "pmm_mg_trajectory3d.hpp"
#include "common.hpp"

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

using std::placeholders::_1;
using std::placeholders::_2;

using namespace pmm;

class PMTTrajectoryServer : public rclcpp::Node
{
public:
  PMTTrajectoryServer()
    : Node("pmt_trajectory_server")
  {
    // Subscriber for UAV parameters
    uav_params_subscription_ = this->create_subscription<uav_custom_msgs::msg::UAVParameters>(
      "uav_parameters", 10, std::bind(&PMTTrajectoryServer::uav_parameters_callback, this, _1));

    // Subscriber for waypoints configuration
    waypoints_subscription_ = this->create_subscription<uav_custom_msgs::msg::Waypoints>(
      "waypoints", 10, std::bind(&PMTTrajectoryServer::waypoints_callback, this, _1));
  
    // Publisher for trajectory
    trajectory_publisher_ = this->create_publisher<uav_custom_msgs::msg::Trajectory>("trajectory", 10);
  }

private:
  void uav_parameters_callback(const uav_custom_msgs::msg::UAVParameters::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received UAV Parameters:");
    // Drone parameters 
    this->max_acc_norm = msg->maximum_acceleration_norm;
    this->max_vel_norm = msg->maximum_velocity;
    this->g = msg->gravitational_acceleration;
    for (size_t i = 0; i < 3; ++i) {
      this->D[i] = msg->drag_coefficients[i];
    }
    /*
    G = g;
    GVEC = {0, 0, -g};
    D_coeffs = -(Eigen::Matrix3d()<< this->D(0), 0, 0, 0, this->D(1), 0, 0, 0, this->D(2)).finished();
    */
    // Thrust decomposition
    this->drag = msg->drag; 
    this->TD_acc_precision = msg->precision;
    this->TD_max_iter = msg->max_iter;

    // First round
    this->alpha = msg->first_run_alpha;
    this->alpha_reduction_factor = msg->first_run_alpha_reduction_factor;
    this->alpha_min_threshold = msg->first_run_alpha_min_threshold;
    this->max_iter = msg->first_run_max_iter;
 
    // Second round
    this->alpha2 = msg->second_run_alpha;
    this->alpha_reduction_factor2 = msg->second_run_alpha_reduction_factor;
    this->alpha_min_threshold2 = msg->second_run_alpha_min_threshold;
    this->max_iter2 = msg->second_run_max_iter;
 
    // Others
    this->dT_precision = msg->dt_precision;
    this->second_round_opt = true;
    
    this->debug = msg->debug;
    this->export_trajectory = msg->sampled_trajectory;
    this->sampling_step = msg->sampling_step;
    this->sampled_trajectory_file = msg->sampled_trajectory_file;
  }

  void waypoints_callback(const uav_custom_msgs::msg::Waypoints::SharedPtr msg)
  {
    std::vector<Scalar> t_s;
    std::vector<Vector<3>> p_s;
    std::vector<Vector<3>> v_s;
    std::vector<Vector<3>> a_s;
    uav_custom_msgs::msg::Trajectory trajectory_msg;

    RCLCPP_INFO(this->get_logger(), "Received Waypoints Configuration");
    
    for (size_t i = 0; i < 3; ++i) {  
      this->start_velocity[i] = msg->start_velocity[i];
    }
    
    for (size_t i = 0; i < 3; ++i) {  
      this->end_velocity[i] = msg->end_velocity[i];
    }
    
    for (size_t i = 0; i < 3; ++i) {  
      this->start_position[i] = msg->start_position[i];
    }
    
    for (size_t i = 0; i < 3; ++i) {  
      this->end_position[i] = msg->end_position[i];
    }

    this->path_waypoints.push_back(this->start_position);
    
    for(long unsigned int i = 0; i < sizeof(msg->waypoints) / sizeof(msg->waypoints[0]);){
      Vector<3> aux_path;
      for(int j = 0; j < 3; j++){
        aux_path[j] = msg->waypoints[i++];
      }
      this->path_waypoints.push_back(aux_path);
    }

    this->path_waypoints.push_back(this->end_position);
    
    PMM_MG_Trajectory3D mp_tr(this->path_waypoints, this->start_velocity, this->end_velocity, this->max_acc_norm, this->max_vel_norm,
                              this->dT_precision, this->max_iter, this->alpha, this->alpha_reduction_factor, this->alpha_min_threshold,
                              this->TD_max_iter, this->TD_acc_precision, this->second_round_opt, this->max_iter2, this->alpha2,
                              this->alpha_reduction_factor2, this->alpha_min_threshold2, this->drag, this->debug);

    if (this->export_trajectory) { 
      mp_tr.sample_and_export_trajectory(this->sampling_step, this->sampled_trajectory_file);
    }
   
    std::tie(t_s, p_s, v_s, a_s) = mp_tr.get_sampled_trajectory(this->sampling_step);
    
    for(long unsigned int i = 0; i < sizeof(t_s) / sizeof(t_s[0]); i++){
      // timestamp
      trajectory_msg.t.push_back(t_s[i]);
      
      // position
      trajectory_msg.p_x.push_back(p_s[0][i]);
      trajectory_msg.p_y.push_back(p_s[1][i]);
      trajectory_msg.p_z.push_back(p_s[2][i]);

      // velocity
      trajectory_msg.v_x.push_back(v_s[0][i]);
      trajectory_msg.v_y.push_back(v_s[1][i]);
      trajectory_msg.v_z.push_back(v_s[2][i]);

      // acceleration
      trajectory_msg.a_x.push_back(a_s[0][i]);
      trajectory_msg.a_y.push_back(a_s[1][i]);
      trajectory_msg.a_z.push_back(a_s[2][i]);
    }

    trajectory_publisher_->publish(trajectory_msg);
  }

  rclcpp::Subscription<uav_custom_msgs::msg::UAVParameters>::SharedPtr uav_params_subscription_;
  rclcpp::Subscription<uav_custom_msgs::msg::Waypoints>::SharedPtr waypoints_subscription_;
  rclcpp::Publisher<uav_custom_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;

  // Drone parameters 
  Vector<3> max_velocity;
  Scalar max_acc_norm = 34.32;
  Scalar max_vel_norm = 90.0;
  Scalar g = 9.8066;
  Vector<3> D = {0.28, 0.35, 0.7};

  // Thrust decomposition
  bool drag = true; 
  Scalar TD_acc_precision = 0.01;
  int TD_max_iter = 20;

  // First round
  Scalar alpha = 10.0;
  Scalar alpha_reduction_factor = 0.2;
  Scalar alpha_min_threshold = 0.001;
  int max_iter = 30;
 
  // Second round
  Scalar alpha2 = 35.0;
  Scalar alpha_reduction_factor2 = 0.1;
  Scalar alpha_min_threshold2 = 0.01;
  int max_iter2 = 10;
 
  // Others
  double dT_precision = 0.001;
  bool second_round_opt = true;

  bool debug = false;
  bool export_trajectory = true;
  Scalar sampling_step = 0.5;
  std::string sampled_trajectory_file = "sampled_trajectory.csv";
  
  // Waypoint data
  Vector<3> start_velocity;
  Vector<3> end_velocity;
  Vector<3> start_position;
  Vector<3> end_position; 
  std::vector<Vector<3>> path_waypoints;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PMTTrajectoryServer>());
  rclcpp::shutdown();
  return 0;
}
