#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import numpy as np
from scipy.linalg import cholesky, LinAlgError
from sensor_msgs.msg import Imu, MagneticField

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_exp(v):
    angle = np.linalg.norm(v)
    if angle < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = v / angle
    return np.array([np.cos(angle/2.0), axis[0]*np.sin(angle/2.0), axis[1]*np.sin(angle/2.0), axis[2]*np.sin(angle/2.0)])

def quat_log(q):
    norm = np.linalg.norm(q[1:])
    if norm < 1e-6:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(norm, q[0])
    return (q[1:] / norm) * angle

class CustomUKFAttitude:
    def __init__(self, frame_type="NED", mx=29.14, my=-4.46, mz=45.00, alpha=1e-3, beta=2.0, kappa=0.0):
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.n = 4  
        
        self.lam = (self.alpha**2) * (self.n + self.kappa) - self.n
        self.gamma = np.sqrt(self.n + self.lam)
        
        self.wm = np.zeros(2 * self.n + 1)
        self.wc = np.zeros(2 * self.n + 1)
        self.wm[0] = self.lam / (self.n + self.lam)
        self.wc[0] = self.wm[0] + (1.0 - self.alpha**2 + self.beta)
        for i in range(1, 2 * self.n + 1):
            self.wm[i] = 1.0 / (2.0 * (self.n + self.lam))
            self.wc[i] = self.wm[i]
            
        self.Q = np.eye(self.n) * 1e-7  
        self.R = np.diag([1e-1, 1e-1, 1e-1, 2e-1, 2e-1, 2e-1]) 
        
        self.x = np.array([1.0, 0.0, 0.0, 0.0])  
        self.P = np.eye(self.n) * 1e-3           
        self.use_mag_mode = False  
        
        self.last_mag_norm = None
        self.mag_disturbed = False

        if frame_type == "ENU":
            self.g_ref = np.array([0.0, 0.0, -1.0]) 
            m_ref = np.array([mx, my, -mz])         
        else: 
            self.g_ref = np.array([0.0, 0.0, 1.0])  
            m_ref = np.array([mx, my, mz])
            
        self.mag_norm_ref_ = np.linalg.norm(m_ref)
        self.b_ref = m_ref / self.mag_norm_ref_
        
        # ROS 2 Logger tĩnh từ hệ thống
        get_logger("ukf_core").info(f"-> [UKF CORE]: Compute mag_norm_ref_ = {self.mag_norm_ref_:.2f} uT")

    def _quaternion_to_dcm(self, q):
        qw, qx, qy, qz = q
        return np.array([
            [1 - 2*(qy**2 + qz**2),   2*(qx*qy + qw*qz),     2*(qx*qz - qw*qy)],
            [2*(qx*qy - qw*qz),       1 - 2*(qx**2 + qz**2), 2*(qy*qz + qw*qx)],
            [2*(qx*qz + qw*qy),       2*(qy*qz - qw*qx),     1 - 2*(qx**2 + qy**2)]
        ])

    def _omega_operator(self, w):
        wx, wy, wz = w
        return np.array([
            [0,  -wx, -wy, -wz],
            [wx,   0,  wz, -wy],
            [wy, -wz,   0,  wx],
            [wz,  wy, -wx,   0]
        ])

    def initialize_state(self, acc, mag=None):
        ax, ay, az = acc / np.linalg.norm(acc)
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        
        cy = np.cos(0.0)
        sy = np.sin(0.0)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        self.x = np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ])
        
        if mag is not None and np.linalg.norm(mag) > 1e-5:
            m_comp_ut = mag * 1e6 
            current_m_norm = np.linalg.norm(m_comp_ut)
            
            if np.abs(current_m_norm - self.mag_norm_ref_) < 0.5 * self.mag_norm_ref_:
                self.use_mag_mode = True
                self.last_mag_norm = current_m_norm
                get_logger("ukf_core").info("-> [UKF CORE]: 9-DoF init successfull.")
                return
                
        self.use_mag_mode = False
        self.last_mag_norm = None
        get_logger("ukf_core").info("-> [UKF CORE]: 6-DoF init successfull.")

    def update(self, gyr, acc, mag=None, dt=0.005):
        self.mag_disturbed = False
        
        if mag is not None:
            m_comp_ut = mag * 1e6 
            current_m_norm = np.linalg.norm(m_comp_ut)
            
            if np.abs(current_m_norm - self.mag_norm_ref_) > 0.4 * self.mag_norm_ref_:
                self.mag_disturbed = True
                
            if self.last_mag_norm is not None:
                mag_disturbance = abs(current_m_norm - self.last_mag_norm)
                if mag_disturbance >= 0.001:
                    self.mag_disturbed = True
                    
            self.last_mag_norm = current_m_norm

        use_mag = self.use_mag_mode and (mag is not None) and (not self.mag_disturbed)
        m_dim = 6 if use_mag else 3
        R_matrix = self.R if use_mag else self.R[:3, :3]
        
        acc_norm = acc / np.linalg.norm(acc)
        if use_mag:
            mag_norm = m_comp_ut / np.linalg.norm(m_comp_ut)
        
        P_safe = 0.5 * (self.P + self.P.T) + np.eye(self.n) * 1e-9
        try:
            L = cholesky((self.n + self.lam) * P_safe, lower=True)
        except LinAlgError:
            P_safe += np.eye(self.n) * 1e-6
            L = cholesky((self.n + self.lam) * P_safe, lower=True)
        
        sigma_points = np.zeros((2 * self.n + 1, self.n))
        sigma_points[0] = self.x
        for i in range(self.n):
            sigma_points[i + 1] = self.x + L[:, i]
            sigma_points[i + 1 + self.n] = self.x - L[:, i]
            
        Omega = self._omega_operator(gyr)
        F = np.eye(self.n) + 0.5 * dt * Omega  
        
        Y = np.zeros_like(sigma_points)
        for i in range(2 * self.n + 1):
            Y[i] = F @ sigma_points[i]
            if np.dot(Y[i], Y[0]) < 0:
                Y[i] = -Y[i]
            Y[i] /= np.linalg.norm(Y[i])
            
        x_pred = np.zeros(self.n)
        for i in range(2 * self.n + 1):
            x_pred += self.wm[i] * Y[i]
        
        norm_x_pred = np.linalg.norm(x_pred)
        x_pred = Y[0] if norm_x_pred < 1e-6 else x_pred / norm_x_pred
        
        P_pred = np.zeros((self.n, self.n))
        for i in range(2 * self.n + 1):
            dx = (Y[i] - x_pred).reshape(-1, 1)
            P_pred += self.wc[i] * (dx @ dx.T)
        P_pred += self.Q
        
        Z = np.zeros((2 * self.n + 1, m_dim))
        for i in range(2 * self.n + 1):
            Rot = self._quaternion_to_dcm(Y[i])
            h_acc = Rot @ self.g_ref
            if use_mag:
                h_mag = Rot @ self.b_ref  
                Z[i] = np.concatenate([h_acc, h_mag])
            else:
                Z[i] = h_acc
                
        z_pred = np.zeros(m_dim)
        for i in range(2 * self.n + 1):
            z_pred += self.wm[i] * Z[i]
            
        P_zz = np.zeros((m_dim, m_dim))
        P_yz = np.zeros((self.n, m_dim))
        for i in range(2 * self.n + 1):
            dz = (Z[i] - z_pred).reshape(-1, 1)
            dx = (Y[i] - x_pred).reshape(-1, 1)
            P_zz += self.wc[i] * (dz @ dz.T)
            P_yz += self.wc[i] * (dx @ dz.T)
        P_zz += R_matrix
        
        K = P_yz @ np.linalg.pinv(P_zz)
        
        z_actual = np.concatenate([acc_norm, mag_norm]) if use_mag else acc_norm
        v = (z_actual - z_pred).reshape(-1, 1)
        
        self.x = x_pred + (K @ v).flatten()
        self.x /= np.linalg.norm(self.x)
        self.P = P_pred - K @ P_zz @ K.T
        
        return self.x

class ROSUKFNode(Node):
    def __init__(self):
        super().__init__('ukf_attitude_node')
        
        # --- Declare PARAMETER ---
        self.declare_parameter('coordinate_frame', 'NED')
        self.declare_parameter('magnetic_reference_x', 29.14)
        self.declare_parameter('magnetic_reference_y', -4.46)
        self.declare_parameter('magnetic_reference_z', 45.00)
        self.declare_parameter('imu_topic', '/imu/proposed')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('output_topic', '/imu/ukf')
        
        # --- Read PARAMETER ---
        coord_frame = self.get_parameter('coordinate_frame').get_parameter_value().string_value
        mag_ref_x = self.get_parameter('magnetic_reference_x').get_parameter_value().double_value
        mag_ref_y = self.get_parameter('magnetic_reference_y').get_parameter_value().double_value
        mag_ref_z = self.get_parameter('magnetic_reference_z').get_parameter_value().double_value
        
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        # init UKF
        self.ukf = CustomUKFAttitude(frame_type=coord_frame, mx=mag_ref_x, my=mag_ref_y, mz=mag_ref_z)
        
        self.last_time = None
        self.latest_mag = None
        self.initialized = False  
        self.frame_counter = 0  
        
        # --- PUBLISHER and SUBSCRIBER ---
        self.imu_pub = self.create_publisher(Imu, pub_topic, 10)
        
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.create_subscription(MagneticField, mag_topic, self.mag_callback, 10)
        
        self.get_logger().info("-> [ROS 2 INTERFACE]: Loaded Param, system ready.")

    def mag_callback(self, msg):
        self.latest_mag = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        gyr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        if not self.initialized:
            self.frame_counter += 1
            if np.linalg.norm(acc) > 0.1:
                if self.latest_mag is not None:
                    self.ukf.initialize_state(acc, self.latest_mag)
                    self.initialized = True
                    self.last_time = current_time
                elif self.frame_counter > 20:
                    self.ukf.initialize_state(acc, None)
                    self.initialized = True
                    self.last_time = current_time
            return
            
        # Tính dt trong ROS 2 bằng cách chuyển hiệu thành nanoseconds rồi chia cho 1e9
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0: return
        if dt > 0.05: dt = 0.005  
        self.last_time = current_time
        
        q = self.ukf.update(gyr=gyr, acc=acc, mag=self.latest_mag, dt=dt)
        
        pub_msg = Imu()
        # Trong ROS 2, kiểu thời gian msg.header.stamp là tương thích đồng bộ trực tiếp
        pub_msg.header.stamp = msg.header.stamp  
        pub_msg.header.frame_id = msg.header.frame_id
        pub_msg.orientation.w = q[0]
        pub_msg.orientation.x = q[1]
        pub_msg.orientation.y = q[2]
        pub_msg.orientation.z = q[3]
        pub_msg.orientation_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        pub_msg.angular_velocity = msg.angular_velocity
        pub_msg.linear_acceleration = msg.linear_acceleration
        self.imu_pub.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ROSUKFNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
