#include "pre_filtering_imu/imu_prefilter_node.h"

IMUPrefilterNode::IMUPrefilterNode()
    : Node("imu_prefilter_node"),
      median_window_size_(5),
      accel_lpf_freq_(20.0),
      gyro_lpf_freq_(40.0),
      imu_sampling_freq_(200.0),
      enable_median_filter_(true) {

  // Load parameters
  load_parameters();

  // Initialize median filters for accelerometer
  for (int i = 0; i < 3; ++i) {
    accel_median_[i] = std::make_unique<MedianFilter>(median_window_size_);
  }

  // Initialize Butterworth LPF for accelerometer (20Hz)
  for (int i = 0; i < 3; ++i) {
    accel_lpf_[i] = std::make_unique<ButterworthFilter>(
        accel_lpf_freq_, imu_sampling_freq_);
  }

  // Initialize Butterworth LPF for gyroscope (40Hz)
  for (int i = 0; i < 3; ++i) {
    gyro_lpf_[i] = std::make_unique<ButterworthFilter>(
        gyro_lpf_freq_, imu_sampling_freq_);
  }

  // Subscribe to raw IMU data (Sử dụng cú pháp ROS 2)
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, 10, std::bind(&IMUPrefilterNode::imu_callback, this, std::placeholders::_1));

  // Publish filtered IMU data
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);

  // Log hệ thống dùng RCLCPP_INFO thay vì ROS_INFO
  RCLCPP_INFO(this->get_logger(), "[IMU PreFilter] Node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "[IMU PreFilter] Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "[IMU PreFilter] Output topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "[IMU PreFilter] Median filter enabled: %d", enable_median_filter_);
  RCLCPP_INFO(this->get_logger(), "[IMU PreFilter] Median window size: %d", median_window_size_);
  RCLCPP_INFO(this->get_logger(), "[IMU PreFilter] Accelerometer LPF cutoff: %.1f Hz", accel_lpf_freq_);
  RCLCPP_INFO(this->get_logger(), "[IMU PreFilter] Gyroscope LPF cutoff: %.1f Hz", gyro_lpf_freq_);
  RCLCPP_INFO(this->get_logger(), "[IMU PreFilter] IMU sampling frequency: %.1f Hz", imu_sampling_freq_);
}

IMUPrefilterNode::~IMUPrefilterNode() = default;

void IMUPrefilterNode::load_parameters() {
  // ROS 2 yêu cầu phải Khai báo (Declare) tham số trước khi Lấy (Get) giá trị
  this->declare_parameter<std::string>("input_topic", "/imu/data");
  this->declare_parameter<std::string>("output_topic", "/imu/data_filtered");
  this->declare_parameter<int>("median_window_size", 5);
  this->declare_parameter<double>("accel_lpf_freq", 20.0);
  this->declare_parameter<double>("gyro_lpf_freq", 40.0);
  this->declare_parameter<double>("imu_sampling_freq", 200.0);
  this->declare_parameter<bool>("enable_median_filter", true);

  this->get_parameter("input_topic", input_topic_);
  this->get_parameter("output_topic", output_topic_);
  this->get_parameter("median_window_size", median_window_size_);
  this->get_parameter("accel_lpf_freq", accel_lpf_freq_);
  this->get_parameter("gyro_lpf_freq", gyro_lpf_freq_);
  this->get_parameter("imu_sampling_freq", imu_sampling_freq_);
  this->get_parameter("enable_median_filter", enable_median_filter_);
}

void IMUPrefilterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  sensor_msgs::msg::Imu filtered_imu = *msg;
  apply_filters(*msg, filtered_imu);
  imu_pub_->publish(filtered_imu);
}

void IMUPrefilterNode::apply_filters(const sensor_msgs::msg::Imu& input,
                                     sensor_msgs::msg::Imu& output) {
  // Step 1: Apply median filter to remove impulse noise
  if (enable_median_filter_) {
    double accel_x_no_spikes = accel_median_[0]->apply(input.linear_acceleration.x);
    double accel_y_no_spikes = accel_median_[1]->apply(input.linear_acceleration.y);
    double accel_z_no_spikes = accel_median_[2]->apply(input.linear_acceleration.z);

    // Step 2: Apply Butterworth LPF to smoothed accelerometer data
    output.linear_acceleration.x = accel_lpf_[0]->apply(accel_x_no_spikes);
    output.linear_acceleration.y = accel_lpf_[1]->apply(accel_y_no_spikes);
    output.linear_acceleration.z = accel_lpf_[2]->apply(accel_z_no_spikes);
  } else {
    output.linear_acceleration.x = accel_lpf_[0]->apply(input.linear_acceleration.x);
    output.linear_acceleration.y = accel_lpf_[1]->apply(input.linear_acceleration.y);
    output.linear_acceleration.z = accel_lpf_[2]->apply(input.linear_acceleration.z);
  }

  // === GYROSCOPE FILTERING ===
  output.angular_velocity.x = gyro_lpf_[0]->apply(input.angular_velocity.x);
  output.angular_velocity.y = gyro_lpf_[1]->apply(input.angular_velocity.y);
  output.angular_velocity.z = gyro_lpf_[2]->apply(input.angular_velocity.z);
}
