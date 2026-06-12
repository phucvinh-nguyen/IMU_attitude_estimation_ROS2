#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
import math

class EskfImuNode(Node):
    def __init__(self):
        super().__init__('eskf_imu_node')
        
        # --- Subscribers & Publishers ---
        self.imu_sub = self.create_subscription(Imu, '/imu/filtered', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, 10)
        self.filtered_imu_pub = self.create_publisher(Imu, '/imu/eskf', 10)
        
        # --- Biến lưu trữ dữ liệu thời gian ---
        self.last_time = None
        self.is_initialized = False
        
        # --- Khởi tạo các tham số Nhiễu (Giống y hệt MATLAB) ---
        wn_var = 1e-5 * np.ones(3)   # rot vel var
        wbn_var = 1e-9 * np.ones(3)  # gyro bias change var
        an_var = 1e-3 * np.ones(3)   # acc var
        mn_var = 1e-4 * np.ones(3)   # mag var
        
        self.Q = np.diag(np.concatenate([wn_var, wbn_var]))
        self.R = np.diag(np.concatenate([an_var, mn_var]))
        
        # --- Khởi tạo trạng thái danh nghĩa (Nominal States) ---
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) 
        self.wb = np.zeros((3, 1)) # Gyro bias ban đầu
        
        # --- Khởi tạo Ma trận hiệp biến lỗi P ---
        q_var_init = 1e-5 * np.ones(3)
        wb_var_init = 1e-7 * np.ones(3)
        self.P = np.diag(np.concatenate([q_var_init, wb_var_init]))
        
        # Bộ đệm dữ liệu cảm biến
        self.latest_mag = None
        self.g_w = np.array([[0.0], [0.0], [9.81]]) # sua NED hoac ENU o day 
        self.m_w = np.array([[0.0], [1.0], [0.0]])  

        self.get_logger().info("ESKF IMU ROS2 Node đã khởi động thành công!")

    def mag_callback(self, msg):
        self.latest_mag = np.array([[msg.magnetic_field.x], 
                                    [msg.magnetic_field.y], 
                                    [msg.magnetic_field.z]])

    def imu_callback(self, imu_msg):
        curr_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = curr_time
            self.initialize_orientation(imu_msg)
            return
            
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        self.last_time = curr_time
        if dt <= 0:
            return

        wm = np.array([[imu_msg.angular_velocity.x], 
                       [imu_msg.angular_velocity.y], 
                       [imu_msg.angular_velocity.z]])
                       
        am = np.array([[imu_msg.linear_acceleration.x], 
                       [imu_msg.linear_acceleration.y], 
                       [imu_msg.linear_acceleration.z]])

        # ========================== 1. STATE PROPAGATION ==========================
        w_unbiased = wm - self.wb
        theta_delta = w_unbiased * dt
        angle = np.linalg.norm(theta_delta)
        if angle > 1e-6:
            axis = theta_delta / angle
            dq = np.array([math.cos(angle/2), 
                           axis[0,0]*math.sin(angle/2), 
                           axis[1,0]*math.sin(angle/2), 
                           axis[2,0]*math.sin(angle/2)])
        else:
            dq = np.array([1.0, 0.5*theta_delta[0,0], 0.5*theta_delta[1,0], 0.5*theta_delta[2,0]])
            
        self.q = self.quat_multiply(self.q, dq)
        self.q = self.q / np.linalg.norm(self.q) 
        
        R_matrix = self.quat_to_rot_matrix(self.q)
        F_x = np.eye(6)
        F_x[0:3, 0:3] = np.eye(3) - self.skew_symmetric(w_unbiased) * dt
        F_x[0:3, 3:6] = -np.eye(3) * dt
        
        self.P = F_x @ self.P @ F_x.T + self.Q * dt

        # ========================== 2. FILTER UPDATE ==========================
        if self.latest_mag is not None:
            h_a = R_matrix.T @ self.g_w 
            h_m = R_matrix.T @ self.m_w 
            
            # ĐÃ SỬA: Bỏ phần gán np.get_array lỗi cú pháp
            z = np.vstack((am, self.latest_mag))
            h_x = np.vstack((h_a, h_m))
            detZ = z - h_x
            
            H = np.zeros((6, 6))
            H[0:3, 0:3] = self.skew_symmetric(h_a)
            H[3:6, 0:3] = self.skew_symmetric(h_m)
            
            S = H @ self.P @ H.T + self.R
            K = self.P @ H.T @ np.linalg.inv(S)
            
            det_x = K @ detZ
            det_theta = det_x[0:3, 0:]
            det_wb = det_x[3:6, 0:]
            
            self.P = self.P - K @ S @ K.T
            
            # ========================== 3. STATE CORRECTION ==========================
            det_q = np.array([1.0, 0.5*det_theta[0,0], 0.5*det_theta[1,0], 0.5*det_theta[2,0]])
            self.q = self.quat_multiply(self.q, det_q)
            self.q = self.q / np.linalg.norm(self.q)
            self.wb += det_wb

        # ========================== 4. PUBLISH DATA ==========================
        self.publish_filtered_imu(imu_msg)

    def initialize_orientation(self, imu_msg):
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z
        # Tính Roll, Pitch dựa trên trọng lực hệ ENU
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        roll = math.atan2(ay, az)
    
        # Tính Yaw ban đầu dựa trên từ trường (nếu có dữ liệu) theo chuẩn ENU
        yaw = 0.0
        if self.latest_mag is not None:
            mx = self.latest_mag[0, 0]
            my = self.latest_mag[1, 0]
            mz = self.latest_mag[2, 0]
            
            # Bù trừ độ nghiêng Roll/Pitch để tính hướng phẳng mặt đất
            hx = mx * math.cos(pitch) + my * math.sin(pitch) * math.sin(roll) + mz * math.sin(pitch) * math.cos(roll)
            hy = my * math.cos(roll) - mz * math.sin(roll)
            
            # Trong ENU, Yaw = atan2(-hx, hy) để góc 0 độ trùng với hướng Đông
            yaw = math.atan2(hx, hy)

        # Chuyển đổi Euler sang Quaternion [w, x, y, z]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        self.q = np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ])

    def quat_multiply(self, q, p):
        w1, x1, y1, z1 = q
        w2, x2, y2, z2 = p
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def quat_to_rot_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x**2 + y**2)]
        ])

    def skew_symmetric(self, v):
        return np.array([
            [0.0, -v[2,0], v[1,0]],
            [v[2,0], 0.0, -v[0,0]],
            [-v[1,0], v[0,0], 0.0]
        ])

    def publish_filtered_imu(self, original_msg):
        out_msg = Imu()
        out_msg.header = original_msg.header
        
        # ĐÃ SỬA: Ép kiểu float() tường minh loại bỏ hoàn toàn kiểu dữ liệu numpy.float64
        out_msg.orientation.x = float(self.q[1])
        out_msg.orientation.y = float(self.q[2])
        out_msg.orientation.z = float(self.q[3])
        out_msg.orientation.w = float(self.q[0])
        
        out_msg.angular_velocity.x = float(original_msg.angular_velocity.x - self.wb[0,0])
        out_msg.angular_velocity.y = float(original_msg.angular_velocity.y - self.wb[1,0])
        out_msg.angular_velocity.z = float(original_msg.angular_velocity.z - self.wb[2,0])
        out_msg.linear_acceleration = original_msg.linear_acceleration
        
        self.filtered_imu_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EskfImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
