#ifndef BUTTERWORTH_FILTER_H
#define BUTTERWORTH_FILTER_H

#include <vector>

/**
 * @class ButterworthFilter
 * @brief 2nd-order Butterworth IIR Low-Pass Filter
 * 
 * Removes high-frequency mechanical vibrations and noise
 * from IMU data while maintaining phase characteristics.
 */
class ButterworthFilter {
public:
  /**
   * @brief Constructor
   * @param cutoff_freq Cutoff frequency in Hz
   * @param sampling_freq Sampling frequency in Hz (e.g., 200Hz for IMU)
   */
  ButterworthFilter(double cutoff_freq, double sampling_freq);

  /**
   * @brief Apply filter to single value
   * @param value Input value
   * @return Filtered value
   */
  double apply(double value);

  /**
   * @brief Apply filter to array
   * @param input Input array
   * @param output Output array
   */
  void apply_array(const std::vector<double>& input, std::vector<double>& output);

  /**
   * @brief Reset filter state
   */
  void reset();

  /**
   * @brief Get filter coefficients (for debugging)
   * @param a Output denominator coefficients [a0, a1, a2]
   * @param b Output numerator coefficients [b0, b1, b2]
   */
  void get_coefficients(std::vector<double>& a, std::vector<double>& b) const;

  /**
   * @brief Set new cutoff frequency
   * @param cutoff_freq New cutoff frequency in Hz
   */
  void set_cutoff_frequency(double cutoff_freq);

private:
  double cutoff_freq_;
  double sampling_freq_;
  
  // IIR filter coefficients (2nd order)
  // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  double b0_, b1_, b2_;
  double a1_, a2_;
  
  // State variables
  std::vector<double> x_history_;  // Input history [x[n-1], x[n-2]]
  std::vector<double> y_history_;  // Output history [y[n-1], y[n-2]]

  /**
   * @brief Calculate Butterworth filter coefficients
   */
  void calculate_coefficients();
};

#endif  // BUTTERWORTH_FILTER_H
