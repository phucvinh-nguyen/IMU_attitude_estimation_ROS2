#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <vector>
#include <deque>

/**
 * @class MedianFilter
 * @brief Median filter for impulse noise removal
 * 
 * Removes spike noise (impulse noise) from IMU accelerometer data
 * without introducing phase lag or blurring motion signals.
 * Window size: 5 samples (as recommended for 200Hz+ IMU)
 */
class MedianFilter {
public:
  /**
   * @brief Constructor
   * @param window_size Size of sliding window (typically 5-9)
   */
  explicit MedianFilter(int window_size = 5);

  /**
   * @brief Apply median filter to single value
   * @param value Input value
   * @return Filtered value
   */
  double apply(double value);

  /**
   * @brief Apply median filter to array
   * @param input Input array
   * @param output Output array
   */
  void apply_array(const std::vector<double>& input, std::vector<double>& output);

  /**
   * @brief Reset filter state
   */
  void reset();

  /**
   * @brief Get window size
   * @return Window size
   */
  int get_window_size() const { return window_size_; }

private:
  int window_size_;
  std::deque<double> buffer_;

  /**
   * @brief Find median of deque
   * @return Median value
   */
  double compute_median();
};

#endif  // MEDIAN_FILTER_H
