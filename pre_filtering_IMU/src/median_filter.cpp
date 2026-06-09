#include "pre_filtering_imu/median_filter.h"
#include <algorithm>
#include <cmath>

MedianFilter::MedianFilter(int window_size)
    : window_size_(window_size) {
  if (window_size <= 0) {
    window_size_ = 5;  // Default window size
  }
}

double MedianFilter::apply(double value) {
  // Add new value to buffer
  buffer_.push_back(value);

  // Keep buffer size at window_size
  if (buffer_.size() > static_cast<size_t>(window_size_)) {
    buffer_.pop_front();
  }

  // Compute and return median
  return compute_median();
}

void MedianFilter::apply_array(const std::vector<double>& input,
                               std::vector<double>& output) {
  output.clear();
  output.reserve(input.size());
  
  for (double value : input) {
    output.push_back(apply(value));
  }
}

void MedianFilter::reset() {
  buffer_.clear();
}

double MedianFilter::compute_median() {
  if (buffer_.empty()) {
    return 0.0;
  }

  // Create a copy and sort for median calculation
  std::vector<double> sorted(buffer_.begin(), buffer_.end());
  std::sort(sorted.begin(), sorted.end());

  size_t n = sorted.size();
  if (n % 2 == 0) {
    // Even number of elements: average of two middle elements
    return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
  } else {
    // Odd number of elements: middle element
    return sorted[n / 2];
  }
}
