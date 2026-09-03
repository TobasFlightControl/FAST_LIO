#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
} EIGEN_ALIGN16;

struct FastLioConfig
{
  double filter_size_corner_min = 0.5;
  double filter_size_surf_min = 0.5;
  double filter_size_map_min = 0.5;
  double cube_len = 200.0;
  float det_range = 300.0;
  double fov_deg = 180.0;
  double gyr_cov = 0.1;
  double acc_cov = 0.1;
  double b_gyr_cov = 0.0001;
  double b_acc_cov = 0.0001;
  bool extrinsic_est_en = true;
  std::vector<double> extrinT = { 0, 0, 0 };
  std::vector<double> extrinR = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  int max_iteration = 4;
};

class FastLioCore
{
public:
  explicit FastLioCore(const FastLioConfig& config, std::function<void(const nav_msgs::msg::Odometry&)> odom_cb);
  ~FastLioCore();

  void addPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud, double time);
  void addImuData(const sensor_msgs::msg::Imu::SharedPtr& msg);

  /* Should be called periodically to process the buffers and trigger `odom_cb`. */
  void process();

private:
  std::function<void(const nav_msgs::msg::Odometry&)> odom_cb_;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))
