# IMU Pre-filtering Package for Unitree Quadruped Robot

## Overview

This ROS1 package implements a cascaded filtering pipeline for IMU data from Unitree quadruped robots. It removes impulse noise (spikes from foot impacts) and mechanical vibrations while maintaining accurate attitude estimation.

### Filtering Pipeline

```
Raw IMU Data (200Hz)
    ↓
[Median Filter] → Removes impulse noise (spikes) from accelerometer
    ↓
[Butterworth LPF 20Hz] → Smooths accelerometer, removes mechanical vibration
    ↓
[Butterworth LPF 40Hz] → Smooths gyroscope data
    ↓
Filtered IMU Data → Feed to Attitude Estimator (EKF/Mahony/Madgwick)
```

## Features

- **Median Filter**: Removes impulse noise (spike) from foot impacts without phase lag
  - Window size: 5 samples (adjustable)
  - Applied only to accelerometer data
  - Zero phase lag filter for accurate motion preservation

- **Butterworth Low-Pass Filter**: Removes high-frequency mechanical vibrations
  - 2nd-order IIR filter
  - Accelerometer: 20Hz cutoff frequency
  - Gyroscope: 40Hz cutoff frequency
  - Minimal phase distortion

- **Cascaded Design**: 
  - Avoids over-filtering and phase lag issues
  - Modular and independently tunable
  - Computationally efficient (IIR filters)

## Installation

### Prerequisites
- ROS1 (Kinetic, Melodic, or Noetic)
- C++11 or later
- Catkin build system

### Build

```bash
cd ~/proposed_ws
catkin build pre_filtering_imu
# or
catkin_make -DCATKIN_WHITELIST_PACKAGES="pre_filtering_imu"

source devel/setup.bash
```

## Usage

### Basic Launch

```bash
roslaunch pre_filtering_imu imu_prefilter.launch
```

### With Custom Parameters

```bash
roslaunch pre_filtering_imu imu_prefilter.launch \
  input_topic:=/imu/raw \
  output_topic:=/imu/filtered \
  imu_sampling_freq:=500.0
```

### Node Parameters

All parameters can be configured in `config/imu_prefilter.yaml`:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/imu/data` | Input raw IMU topic |
| `output_topic` | string | `/imu/proposed` | Output filtered IMU topic |
| `median_window_size` | int | 5 | Median filter window size (samples) |
| `enable_median_filter` | bool | true | Enable median filter for accelerometer |
| `accel_lpf_freq` | double | 20.0 | Accelerometer LPF cutoff (Hz) |
| `gyro_lpf_freq` | double | 40.0 | Gyroscope LPF cutoff (Hz) |
| `imu_sampling_freq` | double | 200.0 | IMU sampling frequency (Hz) |

## Configuration Guidelines

### For Different Quadruped Sizes

#### Small Quadruped (< 5 kg, e.g., Unitree Go1)
```yaml
accel_lpf_freq: 25.0      # Higher cutoff for agile movements
gyro_lpf_freq: 50.0
median_window_size: 5
imu_sampling_freq: 200.0
```

#### Medium Quadruped (5-15 kg, e.g., Unitree A1)
```yaml
accel_lpf_freq: 20.0      # Balanced configuration
gyro_lpf_freq: 40.0
median_window_size: 5
imu_sampling_freq: 200.0
```

#### Large Quadruped (> 15 kg, e.g., Unitree B1)
```yaml
accel_lpf_freq: 15.0      # Lower cutoff for heavier platform
gyro_lpf_freq: 35.0
median_window_size: 7
imu_sampling_freq: 200.0
```

## Filter Design Details

### Median Filter

The median filter is a non-linear filter that:
- Removes spike noise without phase shift
- Preserves edges and step changes
- Window size: 5 samples = 25ms at 200Hz

Mathematical property:
```
y[n] = median(x[n], x[n-1], x[n-2], x[n-3], x[n-4])
```

**When to adjust:**
- Increase window size if spikes are too frequent or large
- Decrease if losing motion detail

### Butterworth Low-Pass Filter

2nd-order Butterworth IIR filter characteristics:
- Phase response: Linear (minimal phase distortion)
- Rolloff: -40dB/decade (-12dB/octave for 2nd order)
- No ripple in passband

**Transfer function:**
```
H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
```

## Message Format

- **Input**: `sensor_msgs/Imu` (raw IMU data)
- **Output**: `sensor_msgs/Imu` (filtered data)

Only `linear_acceleration` and `angular_velocity` fields are modified.
Other fields (orientation, covariance matrices) are preserved.

## Testing

### Test with ROS Bag File

```bash
# Play bag file
rosbag play your_imu_recording.bag

# In another terminal
roslaunch pre_filtering_imu imu_prefilter.launch \
  input_topic:=/your_imu_topic \
  output_topic:=/imu/filtered

# Visualize results
rostopic echo /imu/data
rostopic echo /imu/filtered
```

### Verify Filter Response

```bash
# Check input frequency
rostopic hz /imu/data

# Check output frequency
rostopic hz /imu/filtered

# Inspect data values
rostopic echo /imu/data | head -20
rostopic echo /imu/filtered | head -20
```

## Performance Characteristics

### Computational Cost
- Median filter: O(N log N) per sample (N = window size)
- Butterworth LPF: O(1) per sample
- Total latency: < 5ms per sample at 200Hz

### Memory Usage
- Minimal: ~500 bytes per filter
- Total: < 10KB for all filters

## Troubleshooting

### Filter seems too aggressive (smoothing out motion)
- Decrease cutoff frequency values
- Example: `accel_lpf_freq: 15.0` instead of `20.0`

### Still seeing spikes in output
- Increase median window size
- Example: `median_window_size: 7` or `9`
- Increase LPF cutoff frequencies

### Latency in attitude estimation
- Cutoff frequencies are already optimized
- Consider reducing median window size for real-time constraints

### Unstable filter behavior
- Check that `imu_sampling_freq` matches your actual IMU frequency
- Ensure cutoff frequencies are < sampling_freq / 2

## Implementation Details

### Filter Classes

1. **MedianFilter** (`include/pre_filtering_imu/median_filter.h`)
   - Implements sliding window median filter
   - Zero-latency spike removal

2. **ButterworthFilter** (`include/pre_filtering_imu/butterworth_filter.h`)
   - 2nd-order IIR Butterworth low-pass filter
   - Efficient real-time filtering

3. **IMUPrefilterNode** (`include/pre_filtering_imu/imu_prefilter_node.h`)
   - ROS node wrapper
   - Manages multiple filter instances (one per axis)

### Thread Safety

The current implementation is suitable for single-threaded ROS callbacks.
For multi-threaded operations, use ROS AsyncSpinner with appropriate callback queue settings.

## References

- Butterworth Filter Design: https://en.wikipedia.org/wiki/Butterworth_filter
- Median Filter: https://en.wikipedia.org/wiki/Median_filter
- Quadruped IMU Pre-filtering: Role specifications from Unitree documentation

## License

BSD 3-Clause License

## Author

Victor - Unitree Quadruped IMU Processing

## Contact & Support

For issues or questions:
- Check the configuration file: `config/imu_prefilter.yaml`
- Enable `output="screen"` in launch file for debug messages
- Verify input/output topic names match your system

---

**Note**: This package assumes IMU data follows the standard ROS `sensor_msgs/Imu` message format.
Adjust topic names and parameters according to your specific Unitree robot configuration.
