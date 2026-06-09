#!/usr/bin/env python
"""
Example script demonstrating the IMU pre-filtering package.
Shows how to:
1. Subscribe to raw IMU data
2. Visualize filtering effects (optional)
3. Verify filter parameters
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from collections import deque

class IMUFilterViewer:
    """Simple viewer to compare raw and filtered IMU data"""
    
    def __init__(self):
        rospy.init_node('imu_filter_viewer')
        
        # Parameters
        self.raw_topic = rospy.get_param('~raw_topic', '/imu/data')
        self.filtered_topic = rospy.get_param('~filtered_topic', '/imu/proposed')
        self.window_size = rospy.get_param('~window_size', 30)  # seconds of history
        
        # Data storage
        self.raw_accel_history = deque(maxlen=self.window_size * 200)  # 200Hz
        self.filtered_accel_history = deque(maxlen=self.window_size * 200)
        
        # Subscriptions
        rospy.Subscriber(self.raw_topic, Imu, self.raw_imu_callback)
        rospy.Subscriber(self.filtered_topic, Imu, self.filtered_imu_callback)
        
        rospy.loginfo("IMU Filter Viewer initialized")
        rospy.loginfo("Raw topic: %s", self.raw_topic)
        rospy.loginfo("Filtered topic: %s", self.filtered_topic)
        
        # Start statistics timer
        rospy.Timer(rospy.Duration(5.0), self.print_statistics)
    
    def raw_imu_callback(self, msg):
        """Callback for raw IMU data"""
        accel_mag = np.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        self.raw_accel_history.append({
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'mag': accel_mag,
            'timestamp': msg.header.stamp.to_sec()
        })
    
    def filtered_imu_callback(self, msg):
        """Callback for filtered IMU data"""
        accel_mag = np.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        self.filtered_accel_history.append({
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'mag': accel_mag,
            'timestamp': msg.header.stamp.to_sec()
        })
    
    def print_statistics(self, event=None):
        """Print filter statistics"""
        if len(self.raw_accel_history) < 10 or len(self.filtered_accel_history) < 10:
            return
        
        raw_mags = [d['mag'] for d in self.raw_accel_history]
        filtered_mags = [d['mag'] for d in self.filtered_accel_history]
        
        raw_mean = np.mean(raw_mags)
        raw_std = np.std(raw_mags)
        raw_max = np.max(raw_mags)
        raw_min = np.min(raw_mags)
        
        filtered_mean = np.mean(filtered_mags)
        filtered_std = np.std(filtered_mags)
        filtered_max = np.max(filtered_mags)
        filtered_min = np.min(filtered_mags)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("IMU FILTER STATISTICS")
        rospy.loginfo("=" * 60)
        rospy.loginfo("RAW DATA:")
        rospy.loginfo("  Mean: %.3f m/s^2", raw_mean)
        rospy.loginfo("  Std:  %.3f m/s^2", raw_std)
        rospy.loginfo("  Max:  %.3f m/s^2", raw_max)
        rospy.loginfo("  Min:  %.3f m/s^2", raw_min)
        rospy.loginfo("FILTERED DATA:")
        rospy.loginfo("  Mean: %.3f m/s^2", filtered_mean)
        rospy.loginfo("  Std:  %.3f m/s^2", filtered_std)
        rospy.loginfo("  Max:  %.3f m/s^2", filtered_max)
        rospy.loginfo("  Min:  %.3f m/s^2", filtered_min)
        
        # Calculate noise reduction
        if raw_std > 0:
            noise_reduction = (1.0 - filtered_std / raw_std) * 100.0
            rospy.loginfo("NOISE REDUCTION: %.1f%%", noise_reduction)
        
        # Peak removal
        peak_reduction = (1.0 - filtered_max / raw_max) * 100.0 if raw_max > 0 else 0
        rospy.loginfo("PEAK REDUCTION: %.1f%%", peak_reduction)
        rospy.loginfo("=" * 60)
    
    def spin(self):
        """Run the viewer"""
        rospy.spin()


if __name__ == '__main__':
    try:
        viewer = IMUFilterViewer()
        viewer.spin()
    except rospy.ROSInterruptException:
        pass
