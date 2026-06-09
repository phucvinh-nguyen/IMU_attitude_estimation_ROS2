#include "pre_filtering_imu/butterworth_filter.h"
#include <cmath>
#include <algorithm>

// Mathematical constants
constexpr double PI = 3.141592653589793;

ButterworthFilter::ButterworthFilter(double cutoff_freq, double sampling_freq)
    : cutoff_freq_(cutoff_freq), sampling_freq_(sampling_freq) {
  x_history_.resize(2, 0.0);
  y_history_.resize(2, 0.0);
  calculate_coefficients();
}

double ButterworthFilter::apply(double value) {
  // IIR filter: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  double output = b0_ * value + b1_ * x_history_[0] + b2_ * x_history_[1]
                - a1_ * y_history_[0] - a2_ * y_history_[1];

  // Update history
  x_history_[1] = x_history_[0];
  x_history_[0] = value;
  y_history_[1] = y_history_[0];
  y_history_[0] = output;

  return output;
}

void ButterworthFilter::apply_array(const std::vector<double>& input,
                                    std::vector<double>& output) {
  output.clear();
  output.reserve(input.size());
  
  for (double value : input) {
    output.push_back(apply(value));
  }
}

void ButterworthFilter::reset() {
  std::fill(x_history_.begin(), x_history_.end(), 0.0);
  std::fill(y_history_.begin(), y_history_.end(), 0.0);
}

void ButterworthFilter::get_coefficients(std::vector<double>& a,
                                         std::vector<double>& b) const {
  a = {1.0, a1_, a2_};
  b = {b0_, b1_, b2_};
}

void ButterworthFilter::set_cutoff_frequency(double cutoff_freq) {
  cutoff_freq_ = cutoff_freq;
  calculate_coefficients();
  reset();
}

void ButterworthFilter::calculate_coefficients() {
  // Normalized cutoff frequency
  double wc = 2.0 * PI * cutoff_freq_ / sampling_freq_;
  
  // Butterworth 2nd-order filter coefficient
  // sin and cos of normalized cutoff frequency
  double sin_wc = std::sin(wc / 2.0);
  double cos_wc = std::cos(wc / 2.0);
  double sqrt2 = std::sqrt(2.0);

  // Denominator coefficients
  double q = sqrt2 * sin_wc;
  double a0 = 1.0 + sqrt2 * sin_wc + sin_wc * sin_wc;
  
  a1_ = 2.0 * (sin_wc * sin_wc - 1.0) / a0;
  a2_ = (1.0 - sqrt2 * sin_wc + sin_wc * sin_wc) / a0;

  // Numerator coefficients
  b0_ = sin_wc * sin_wc / a0;
  b1_ = 2.0 * sin_wc * sin_wc / a0;
  b2_ = sin_wc * sin_wc / a0;
}

