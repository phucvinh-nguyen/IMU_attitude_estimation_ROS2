#ifndef IMU_PREFILTER_NODE_H
#define IMU_PREFILTER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <memory>
#include "median_filter.h"
#include "butterworth_filter.h"

/**
 * @class IMUPrefilterNode
 * @brief ROS 2 node for IMU pre-filtering
 */
class IMUPrefilterNode : public rclcpp::Node {
public:
  IMUPrefilterNode();
  virtual ~IMUPrefilterNode();

private:
  // ROS 2 Subscribers và Publishers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Filters cho accelerometer (ax, ay, az)
  std::unique_ptr<MedianFilter> accel_median_[3];
  std::unique_ptr<ButterworthFilter> accel_lpf_[3];

  // Filters cho gyroscope (gx, gy, gz)
  std::unique_ptr<ButterworthFilter> gyro_lpf_[3];

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  int median_window_size_;
  double accel_lpf_freq_;
  double gyro_lpf_freq_;
  double imu_sampling_freq_;
  bool enable_median_filter_;

  // Callback
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Parameter loading
  void load_parameters();

  // Filter application
  void apply_filters(const sensor_msgs::msg::Imu& input, sensor_msgs::msg::Imu& output);
};

#endif  // IMU_PREFILTER_NODE_H
