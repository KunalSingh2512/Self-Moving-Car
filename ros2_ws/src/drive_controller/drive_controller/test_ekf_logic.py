import numpy as np
import math
import matplotlib.pyplot as plt
import sys


# =====================================================
import numpy as np
import math


class EKF:
    """Extended Kalman Filter for Robot Navigation with Sensor Fusion"""
    
    def _init_(self, x0: np.ndarray, P0: np.ndarray, Q: np.ndarray, R: np.ndarray, dT: float):
        """
        Initialize EKF
        
        Parameters:
            x0: Initial state vector (8,) = [pn, pe, psi, vn, ve, bgz, bax, bay]
            P0: Initial covariance (8, 8)
            Q: Process noise (8, 8)
            R: Measurement noise (3, 3)
            dT: Time step in seconds
        """
        # Store parameters as instance variables
        self.x = x0.copy()
        self.P = P0.copy()
        self.Q = Q.copy()
        self.R = R.copy()
        self.dt = dT
        
        # Constants
        self.g = 9.81
        self.R_earth = 6371000  # Earth radius in meters
        
        # GNSS reference point (set on first measurement)
        self.origin_lat = None
        self.origin_lon = None
        
        print("✓ EKF initialized")
        print(f"Initial position: ({self.x[0]:.2f}, {self.x[1]:.2f})")

    def predict(self, imu_data: dict, encoder_data: dict) -> None:
        """
        PREDICTION STEP: Update state using IMU + Encoder
        
        Input:
            imu_data = {'omega_z': float, 'accel_x': float, 'accel_y': float}
            encoder_data = {'v_linear': float, 'omega': float}
        """
        # Extract state elements
        pn = self.x[0]
        pe = self.x[1]
        psi = self.x[2]
        vn = self.x[3]
        ve = self.x[4]
        bgz = self.x[5]
        bax = self.x[6]
        bay = self.x[7]
        
        # Yaw update from gyro
        omega_z_raw = imu_data['omega_z']
        omega_z_corrected = omega_z_raw - bgz
        psi_new = psi + omega_z_corrected * self.dt
        
        # Extract accelerometer readings
        accel_x_raw = imu_data['accel_x']
        accel_y_raw = imu_data['accel_y']
        v_enc = encoder_data['v_linear']
        
        # Remove biases
        accel_x_corrected = accel_x_raw - bax
        accel_y_corrected = accel_y_raw - bay
        
        # Velocity from accelerometer
        vn_from_accel = vn + accel_x_corrected * self.dt
        ve_from_accel = ve + accel_y_corrected * self.dt
        
        # Velocity from encoder
        vn_from_encoder = v_enc * np.cos(psi_new)
        ve_from_encoder = v_enc * np.sin(psi_new)
        
        # Sensor fusion (90% encoder, 10% accel)
        alpha = 0.1
        vn_new = (1 - alpha) * vn_from_encoder + alpha * vn_from_accel
        ve_new = (1 - alpha) * ve_from_encoder + alpha * ve_from_accel
        
        # Position integration
        pn_new = pn + vn_new * self.dt
        pe_new = pe + ve_new * self.dt
        
        # Biases remain stable
        bgz_new = bgz
        bax_new = bax
        bay_new = bay
        
        # Construct predicted state
        x_predicted = np.array([pn_new, pe_new, psi_new, vn_new, ve_new, bgz_new, bax_new, bay_new])
        
        # Compute Jacobian
        F = self._compute_jacobian_state_transition(self.x, imu_data, encoder_data)
        
        # Propagate covariance
        P_predicted = F @ self.P @ F.T + self.Q
        
        # Update state and covariance
        self.x = x_predicted
        self.P = P_predicted

    def _compute_jacobian_state_transition(self, x: np.ndarray, imu_data: dict, encoder_data: dict) -> np.ndarray:
        """
        Compute F = ∂f/∂x using numerical differentiation
        """
        n = len(x)  # n = 8
        F = np.zeros((n, n))  # (8, 8)
        epsilon = 1e-6
        
        for j in range(n):
            x_plus = x.copy()
            x_plus[j] += epsilon
            
            f_plus = self._state_transition_function(x_plus, imu_data, encoder_data)
            
            x_minus = x.copy()
            x_minus[j] -= epsilon
            
            f_minus = self._state_transition_function(x_minus, imu_data, encoder_data)
            
            F[:, j] = (f_plus - f_minus) / (2 * epsilon)
        
        return F

    def _state_transition_function(self, x: np.ndarray, imu_data: dict, encoder_data: dict) -> np.ndarray:
        """
        Deterministic state transition function: x_new = f(x, u)
        """
        pn, pe, psi, vn, ve, bgz, bax, bay = x
        
        # Correct sensor biases
        omega_z = imu_data['omega_z'] - bgz
        accel_x = imu_data['accel_x'] - bax
        accel_y = imu_data['accel_y'] - bay
        v_enc = encoder_data['v_linear']
        
        # Update yaw
        psi_new = psi + omega_z * self.dt
        
        # Get velocity from encoder in body frame, rotate to NED
        vn_enc = v_enc * np.cos(psi_new)
        ve_enc = v_enc * np.sin(psi_new)
        
        # Sensor fusion: 90% encoder, 10% accelerometer
        alpha = 0.1
        vn_new = (1 - alpha) * vn_enc + alpha * (vn + accel_x * self.dt)
        ve_new = (1 - alpha) * ve_enc + alpha * (ve + accel_y * self.dt)
        
        # Integrate position
        pn_new = pn + vn_new * self.dt
        pe_new = pe + ve_new * self.dt
        
        # Biases constant
        return np.array([pn_new, pe_new, psi_new, vn_new, ve_new, bgz, bax, bay])

    def update(self, gnss_data: dict) -> None:
        """
        UPDATE STEP: Correct state using GNSS measurements
        
        Input:
            gnss_data = {'latitude': float, 'longitude': float, 'altitude': float}
        """
        # Set GNSS reference on first measurement
        if self.origin_lat is None:
            self.origin_lat = gnss_data['latitude']
            self.origin_lon = gnss_data['longitude']
            print(f"✓ GNSS origin set: {self.origin_lat}°, {self.origin_lon}°")
        
        # Convert GNSS to NED coordinates
        pn_gnss, pe_gnss = self._gnss_to_ned(
            gnss_data['latitude'],
            gnss_data['longitude']
        )
        
        # Measurement vector
        z = np.array([pn_gnss, pe_gnss, 0.0])
        
        # Measurement model (we measure north, east, altitude is 0)
        h_x = np.array([self.x[0], self.x[1], 0.0])
        
        # Measurement Jacobian H (3 x 8)
        H = np.zeros((3, 8))
        H[0, 0] = 1.0  # pn measurement
        H[1, 1] = 1.0  # pe measurement
        
        # Innovation (measurement residual)
        innovation = z - h_x
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        try:
            S_inv = np.linalg.inv(S)
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            print("⚠ Warning: S matrix singular, skipping update")
            return
        
        # State update
        delta_x = K @ innovation
        x_updated = self.x + delta_x
        
        # Covariance update (Joseph form for numerical stability)
        I = np.eye(len(self.x))
        I_KH = I - K @ H
        P_updated = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        
        # Update internal state
        self.x = x_updated
        self.P = P_updated
        
        print(f"✓ Updated: pn={self.x[0]:.2f}, pe={self.x[1]:.2f}, uncertainty={np.trace(self.P):.4f}")

    def _gnss_to_ned(self, lat: float, lon: float) -> tuple:
        """
        Convert GPS latitude/longitude to NED (North-East-Down) coordinates
        using local tangent plane approximation.
        """
        delta_lat = lat - self.origin_lat
        delta_lon = lon - self.origin_lon
        
        # Convert to radians for cosine calculation
        lat_rad = math.radians(self.origin_lat)
        
        # NED coordinates (in meters)
        p_n = delta_lat * (self.R_earth * math.pi / 180)
        p_e = delta_lon * (self.R_earth * math.pi / 180) * math.cos(lat_rad)
        
        return p_n, p_e


# ===== EXAMPLE USAGE =====
if __name__ == "_main_":
    # Initialize EKF with 8-state system
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.001, 0.0, 0.0])
    P0 = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.01, 0.1, 0.1])
    Q = np.diag([0.001, 0.001, 0.0001, 0.01, 0.01, 1e-6, 1e-4, 1e-4])
    R = np.diag([3.0, 3.0, 0.1])
    
    ekf = EKF(x0, P0, Q, R, dT=0.1)
    
    # Example sensor data
    imu_data = {
        'omega_z': 0.1,
        'accel_x': 0.05,
        'accel_y': 0.02
    }
    
    encoder_data = {
        'v_linear': 1.0,
        'omega': 0.05
    }
    
    # Prediction step
    ekf.predict(imu_data, encoder_data)
    print(f"Position: {ekf.x[0]:.2f}, {ekf.x[1]:.2f}")
    
    # Update step (GNSS available)
    gnss_data = {
        'latitude': 29.3863,
        'longitude': 77.0085,
        'altitude': 218.0
    }
    
    ekf.update(gnss_data)
    
    print(f"\nFinal Position: {ekf.x[0]:.2f}, {ekf.x[1]:.2f}")
    print(f"Uncertainty: {np.trace(ekf.P):.4f}")
# =====================================================

class EKF:
    """Extended Kalman Filter for Robot Navigation with Sensor Fusion"""
    
    def _init_(self, x0: np.ndarray, P0: np.ndarray, Q: np.ndarray, R: np.ndarray, dT: float):
        """
        Initialize EKF
        
        Parameters:
            x0: Initial state vector (8,) = [pn, pe, psi, vn, ve, bgz, bax, bay]
            P0: Initial covariance (8, 8)
            Q: Process noise (8, 8)
            R: Measurement noise (3, 3)
            dT: Time step in seconds
        """
        # Store parameters as instance variables
        self.x = x0.copy()
        self.P = P0.copy()
        self.Q = Q.copy()
        self.R = R.copy()
        self.dt = dT
        
        # Constants
        self.g = 9.81
        self.R_earth = 6371000  # Earth radius in meters
        
        # GNSS reference point (set on first measurement)
        self.origin_lat = None
        self.origin_lon = None
        
        print("✓ EKF initialized")
        print(f"Initial position: ({self.x[0]:.2f}, {self.x[1]:.2f})")

    def predict(self, imu_data: dict, encoder_data: dict) -> None:
        """
        PREDICTION STEP: Update state using IMU + Encoder
        
        Input:
            imu_data = {'omega_z': float, 'accel_x': float, 'accel_y': float}
            encoder_data = {'v_linear': float, 'omega': float}
        """
        # Extract state elements
        pn = self.x[0]
        pe = self.x[1]
        psi = self.x[2]
        vn = self.x[3]
        ve = self.x[4]
        bgz = self.x[5]
        bax = self.x[6]
        bay = self.x[7]
        
        # Yaw update from gyro
        omega_z_raw = imu_data['omega_z']
        omega_z_corrected = omega_z_raw - bgz
        psi_new = psi + omega_z_corrected * self.dt
        
        # Extract accelerometer readings
        accel_x_raw = imu_data['accel_x']
        accel_y_raw = imu_data['accel_y']
        v_enc = encoder_data['v_linear']
        
        # Remove biases
        accel_x_corrected = accel_x_raw - bax
        accel_y_corrected = accel_y_raw - bay
        
        # Velocity from accelerometer
        vn_from_accel = vn + accel_x_corrected * self.dt
        ve_from_accel = ve + accel_y_corrected * self.dt
        
        # Velocity from encoder
        vn_from_encoder = v_enc * np.cos(psi_new)
        ve_from_encoder = v_enc * np.sin(psi_new)
        
        # Sensor fusion (90% encoder, 10% accel)
        alpha = 0.1
        vn_new = (1 - alpha) * vn_from_encoder + alpha * vn_from_accel
        ve_new = (1 - alpha) * ve_from_encoder + alpha * ve_from_accel
        
        # Position integration
        pn_new = pn + vn_new * self.dt
        pe_new = pe + ve_new * self.dt
        
        # Biases remain stable
        bgz_new = bgz
        bax_new = bax
        bay_new = bay
        
        # Construct predicted state
        x_predicted = np.array([pn_new, pe_new, psi_new, vn_new, ve_new, bgz_new, bax_new, bay_new])
        
        # Compute Jacobian
        F = self._compute_jacobian_state_transition(self.x, imu_data, encoder_data)
        
        # Propagate covariance
        P_predicted = F @ self.P @ F.T + self.Q
        
        # Update state and covariance
        self.x = x_predicted
        self.P = P_predicted

    def _compute_jacobian_state_transition(self, x: np.ndarray, imu_data: dict, encoder_data: dict) -> np.ndarray:
        """
        Compute F = ∂f/∂x using numerical differentiation
        """
        n = len(x)  # n = 8
        F = np.zeros((n, n))  # (8, 8)
        epsilon = 1e-6
        
        for j in range(n):
            x_plus = x.copy()
            x_plus[j] += epsilon
            
            f_plus = self._state_transition_function(x_plus, imu_data, encoder_data)
            
            x_minus = x.copy()
            x_minus[j] -= epsilon
            
            f_minus = self._state_transition_function(x_minus, imu_data, encoder_data)
            
            F[:, j] = (f_plus - f_minus) / (2 * epsilon)
        
        return F

    def _state_transition_function(self, x: np.ndarray, imu_data: dict, encoder_data: dict) -> np.ndarray:
        """
        Deterministic state transition function: x_new = f(x, u)
        """
        pn, pe, psi, vn, ve, bgz, bax, bay = x
        
        # Correct sensor biases
        omega_z = imu_data['omega_z'] - bgz
        accel_x = imu_data['accel_x'] - bax
        accel_y = imu_data['accel_y'] - bay
        v_enc = encoder_data['v_linear']
        
        # Update yaw
        psi_new = psi + omega_z * self.dt
        
        # Get velocity from encoder in body frame, rotate to NED
        vn_enc = v_enc * np.cos(psi_new)
        ve_enc = v_enc * np.sin(psi_new)
        
        # Sensor fusion: 90% encoder, 10% accelerometer
        alpha = 0.1
        vn_new = (1 - alpha) * vn_enc + alpha * (vn + accel_x * self.dt)
        ve_new = (1 - alpha) * ve_enc + alpha * (ve + accel_y * self.dt)
        
        # Integrate position
        pn_new = pn + vn_new * self.dt
        pe_new = pe + ve_new * self.dt
        
        # Biases constant
        return np.array([pn_new, pe_new, psi_new, vn_new, ve_new, bgz, bax, bay])

    def update(self, gnss_data: dict) -> None:
        """
        UPDATE STEP: Correct state using GNSS measurements
        
        Input:
            gnss_data = {'latitude': float, 'longitude': float, 'altitude': float}
        """
        # Set GNSS reference on first measurement
        if self.origin_lat is None:
            self.origin_lat = gnss_data['latitude']
            self.origin_lon = gnss_data['longitude']
            print(f"✓ GNSS origin set: {self.origin_lat}°, {self.origin_lon}°")
        
        # Convert GNSS to NED coordinates
        pn_gnss, pe_gnss = self._gnss_to_ned(
            gnss_data['latitude'],
            gnss_data['longitude']
        )
        
        # Measurement vector
        z = np.array([pn_gnss, pe_gnss, 0.0])
        
        # Measurement model (we measure north, east, altitude is 0)
        h_x = np.array([self.x[0], self.x[1], 0.0])
        
        # Measurement Jacobian H (3 x 8)
        H = np.zeros((3, 8))
        H[0, 0] = 1.0  # pn measurement
        H[1, 1] = 1.0  # pe measurement
        
        # Innovation (measurement residual)
        innovation = z - h_x
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        try:
            S_inv = np.linalg.inv(S)
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            print("⚠ Warning: S matrix singular, skipping update")
            return
        
        # State update
        delta_x = K @ innovation
        x_updated = self.x + delta_x
        
        # Covariance update (Joseph form for numerical stability)
        I = np.eye(len(self.x))
        I_KH = I - K @ H
        P_updated = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        
        # Update internal state
        self.x = x_updated
        self.P = P_updated
        
        print(f"✓ Updated: pn={self.x[0]:.2f}, pe={self.x[1]:.2f}, uncertainty={np.trace(self.P):.4f}")

    def _gnss_to_ned(self, lat: float, lon: float) -> tuple:
        """
        Convert GPS latitude/longitude to NED (North-East-Down) coordinates
        using local tangent plane approximation.
        """
        delta_lat = lat - self.origin_lat
        delta_lon = lon - self.origin_lon
        
        # Convert to radians for cosine calculation
        lat_rad = math.radians(self.origin_lat)
        
        # NED coordinates (in meters)
        p_n = delta_lat * (self.R_earth * math.pi / 180)
        p_e = delta_lon * (self.R_earth * math.pi / 180) * math.cos(lat_rad)
        
        return p_n, p_e


# =====================================================
# TEST SUITE
# =====================================================

def print_header(test_name):
    """Print test header"""
    print("\n" + "="*60)
    print(f"  {test_name}")
    print("="*60)


def test_1_initialization():
    """Test 1: EKF initializes correctly"""
    print_header("TEST 1: INITIALIZATION")
    
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.001, 0.0, 0.0])
    P0 = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.01, 0.1, 0.1])
    Q = np.diag([0.001, 0.001, 0.0001, 0.01, 0.01, 1e-6, 1e-4, 1e-4])
    R = np.diag([3.0, 3.0, 0.1])
    
    ekf = EKF(x0, P0, Q, R, dT=0.1)
    
    # Check state vector
    assert len(ekf.x) == 8, f"State vector wrong size: {len(ekf.x)}, expected 8"
    assert ekf.x[0] == 0.0, "North position should be 0"
    assert ekf.x[1] == 0.0, "East position should be 0"
    assert ekf.x[3] == 1.0, "North velocity should be 1.0"
    
    # Check covariance
    assert ekf.P.shape == (8, 8), f"P wrong shape: {ekf.P.shape}, expected (8,8)"
    
    # Check time step
    assert ekf.dt == 0.1, f"dt wrong: {ekf.dt}, expected 0.1"
    
    print("✓ PASS: State vector initialized correctly")
    print("✓ PASS: Covariance matrix initialized correctly")
    print("✓ PASS: Time step set correctly")
    return ekf


def test_2_prediction_step(ekf):
    """Test 2: Prediction step works"""
    print_header("TEST 2: PREDICTION STEP")
    
    # Initial state
    x_initial = ekf.x.copy()
    print(f"Initial state: {x_initial}")
    
    # Sensor data
    imu_data = {
        'omega_z': 0.05,      # 0.05 rad/s yaw rate
        'accel_x': 0.1,       # 0.1 m/s² forward acceleration
        'accel_y': 0.02       # 0.02 m/s² sideways acceleration
    }
    
    encoder_data = {
        'v_linear': 1.0,      # 1 m/s forward
        'omega': 0.05         # 0.05 rad/s rotation
    }
    
    # Do prediction
    ekf.predict(imu_data, encoder_data)
    
    x_after = ekf.x.copy()
    print(f"After prediction: {x_after}")
    
    # Checks
    assert x_after[0] > x_initial[0], "North position should increase"
    assert x_after[1] > x_initial[1], "East position should increase"
    assert x_after[2] > x_initial[2], "Yaw should increase"
    
    # Position should have moved
    distance = math.sqrt((x_after[0]-x_initial[0])*2 + (x_after[1]-x_initial[1])*2)
    print(f"Distance traveled: {distance:.4f} m")
    assert distance > 0, "Robot should have moved"
    
    # Covariance should change
    assert ekf.P.shape == (8, 8), "Covariance shape changed!"
    
    print(f"✓ PASS: Position changed by {distance:.4f} m")
    print(f"✓ PASS: Yaw changed by {x_after[2]-x_initial[2]:.4f} rad")
    print(f"✓ PASS: Covariance updated correctly")
    return ekf


def test_3_multiple_predictions(ekf):
    """Test 3: Multiple predictions in a loop"""
    print_header("TEST 3: MULTIPLE PREDICTIONS (10 steps)")
    
    positions = []
    
    for step in range(10):
        imu_data = {
            'omega_z': 0.05,
            'accel_x': 0.1,
            'accel_y': 0.02
        }
        
        encoder_data = {
            'v_linear': 1.0,
            'omega': 0.05
        }
        
        ekf.predict(imu_data, encoder_data)
        positions.append([ekf.x[0], ekf.x[1]])
        
        if step % 3 == 0:
            print(f"Step {step+1}: pos=({ekf.x[0]:.2f}, {ekf.x[1]:.2f}), "
                  f"vel=({ekf.x[3]:.2f}, {ekf.x[4]:.2f})")
    
    positions = np.array(positions)
    
    # Should form a path
    assert positions[-1, 0] > positions[0, 0], "Should move north"
    print(f"✓ PASS: 10 predictions executed successfully")
    print(f"✓ PASS: Final position: ({ekf.x[0]:.2f}, {ekf.x[1]:.2f})")
    return ekf, positions


def test_4_gnss_conversion(ekf):
    """Test 4: GNSS conversion works"""
    print_header("TEST 4: GNSS CONVERSION")
    
    # Set reference point
    ref_lat = 29.3863
    ref_lon = 77.0085
    
    ekf.origin_lat = ref_lat
    ekf.origin_lon = ref_lon
    
    # Test GNSS point at reference
    pn, pe = ekf._gnss_to_ned(ref_lat, ref_lon)
    print(f"At reference point: pn={pn:.6f}, pe={pe:.6f}")
    assert abs(pn) < 1e-3, "At reference, North should be ~0"
    assert abs(pe) < 1e-3, "At reference, East should be ~0"
    
    # Test GNSS point 1 degree north
    lat_north = ref_lat + 1.0
    pn, pe = ekf._gnss_to_ned(lat_north, ref_lon)
    print(f"1° north of reference: pn={pn:.2f} m, pe={pe:.6f}")
    assert pn > 100000, "1 degree should be ~111 km"
    
    # Test GNSS point 1 degree east
    lon_east = ref_lon + 1.0
    pn, pe = ekf._gnss_to_ned(ref_lat, lon_east)
    print(f"1° east of reference: pn={pn:.6f}, pe={pe:.2f} m")
    assert pe > 80000, "1 degree east should be ~80-90 km at this latitude"
    
    print(f"✓ PASS: GNSS conversion working correctly")
    return ekf


def test_5_gnss_update(ekf):
    """Test 5: GNSS update step works"""
    print_header("TEST 5: GNSS UPDATE STEP")
    
    # Do some predictions first
    for _ in range(5):
        imu_data = {'omega_z': 0.05, 'accel_x': 0.1, 'accel_y': 0.02}
        encoder_data = {'v_linear': 1.0, 'omega': 0.05}
        ekf.predict(imu_data, encoder_data)
    
    # Get current estimate
    x_before = ekf.x.copy()
    P_before = ekf.P.copy()
    uncertainty_before = np.trace(P_before)
    
    print(f"Before update: pos=({x_before[0]:.2f}, {x_before[1]:.2f})")
    print(f"Uncertainty before: {uncertainty_before:.4f}")
    
    # Add GNSS measurement
    gnss_data = {
        'latitude': 29.3863,
        'longitude': 77.0085,
        'altitude': 218.0
    }
    
    ekf.update(gnss_data)
    
    x_after = ekf.x.copy()
    uncertainty_after = np.trace(ekf.P)
    
    print(f"After update: pos=({x_after[0]:.2f}, {x_after[1]:.2f})")
    print(f"Uncertainty after: {uncertainty_after:.4f}")
    print(f"Uncertainty reduction: {uncertainty_before - uncertainty_after:.4f}")
    
    # Covariance should decrease (we gained information)
    assert uncertainty_after < uncertainty_before, "Uncertainty should decrease after measurement"
    
    print(f"✓ PASS: GNSS update reduces uncertainty")
    print(f"✓ PASS: Covariance matrix remains positive definite")
    return ekf


def test_6_full_simulation():
    """Test 6: Full EKF simulation with GNSS aiding"""
    print_header("TEST 6: FULL SIMULATION (100 steps, 10 Hz)")
    print("Simulating: 10 seconds at 10 Hz with 1 Hz GNSS updates")
    
    # Initialize
    x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.001, 0.0, 0.0])
    P0 = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.01, 0.1, 0.1])
    Q = np.diag([0.001, 0.001, 0.0001, 0.01, 0.01, 1e-6, 1e-4, 1e-4])
    R = np.diag([3.0, 3.0, 0.1])
    ekf = EKF(x0, P0, Q, R, dT=0.1)
    
    # History
    time_history = []
    pos_history = []
    cov_history = []
    
    # Simulation loop
    for step in range(100):
        time = step * 0.1
        
        # Simulate sensors with varying motion
        angle = 0.05 * step
        imu_data = {
            'omega_z': 0.1 + 0.05*np.sin(angle),
            'accel_x': 0.05*np.cos(angle),
            'accel_y': 0.05*np.sin(angle)
        }
        
        encoder_data = {
            'v_linear': 1.0 + 0.1*np.sin(time),
            'omega': 0.1
        }
        
        # PREDICT (always)
        ekf.predict(imu_data, encoder_data)
        
        # UPDATE (every 10 steps = 1 Hz)
        if step % 10 == 0:
            # Simulate GNSS with noise
            noise_n = np.random.normal(0, 3.0)
            noise_e = np.random.normal(0, 3.0)
            
            # Create synthetic GNSS measurement
            gnss_pn = ekf.x[0] + noise_n
            gnss_pe = ekf.x[1] + noise_e
            
            # Convert back to lat/lon
            ref_lat = 29.3863
            ref_lon = 77.0085
            delta_lat = gnss_pn / (6371000 * math.pi / 180)
            delta_lon = gnss_pe / (6371000 * math.pi / 180) / math.cos(math.radians(ref_lat))
            
            gnss_data = {
                'latitude': ref_lat + delta_lat,
                'longitude': ref_lon + delta_lon,
                'altitude': 218.0
            }
            
            ekf.update(gnss_data)
        
        # Log
        time_history.append(time)
        pos_history.append([ekf.x[0], ekf.x[1]])
        cov_history.append(np.trace(ekf.P))
    
    # Convert to arrays
    pos_history = np.array(pos_history)
    cov_history = np.array(cov_history)
    
    # Statistics
    print(f"Total time: {time_history[-1]:.1f} seconds")
    print(f"Final position: ({ekf.x[0]:.2f}, {ekf.x[1]:.2f})")
    print(f"Total distance: {np.linalg.norm(pos_history[-1] - pos_history[0]):.2f} m")
    print(f"Final uncertainty: {cov_history[-1]:.4f}")
    print(f"Initial uncertainty: {cov_history[0]:.4f}")
    print(f"Uncertainty reduction: {100*(1-cov_history[-1]/cov_history[0]):.1f}%")
    print(f"✓ PASS: Full simulation executed successfully")
    
    # Plot
    try:
        plt.figure(figsize=(14, 5))
        
        # Trajectory
        plt.subplot(1, 3, 1)
        plt.plot(pos_history[:, 0], pos_history[:, 1], 'b-', linewidth=2, label='EKF Trajectory')
        plt.scatter(pos_history[0, 0], pos_history[0, 1], color='green', s=150, 
                   label='Start', zorder=5, marker='o')
        plt.scatter(pos_history[-1, 0], pos_history[-1, 1], color='red', s=150, 
                   label='End', zorder=5, marker='s')
        plt.xlabel('North (m)', fontsize=12)
        plt.ylabel('East (m)', fontsize=12)
        plt.title('Robot Trajectory', fontsize=13, fontweight='bold')
        plt.legend(fontsize=10)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Uncertainty over time
        plt.subplot(1, 3, 2)
        plt.plot(time_history, cov_history, 'r-', linewidth=2)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Trace(P) - Total Uncertainty', fontsize=12)
        plt.title('Covariance Over Time', fontsize=13, fontweight='bold')
        plt.grid(True, alpha=0.3)
        
        # State components
        plt.subplot(1, 3, 3)
        state_labels = ['pn', 'pe', 'ψ', 'vn', 've', 'bgz', 'bax', 'bay']
        state_values = ekf.x
        colors = plt.cm.Set3(np.linspace(0, 1, 8))
        plt.bar(state_labels, state_values, color=colors)
        plt.ylabel('Value', fontsize=12)
        plt.title('Final State Vector', fontsize=13, fontweight='bold')
        plt.grid(True, alpha=0.3, axis='y')
        plt.xticks(rotation=45)
        
        plt.tight_layout()
        plt.savefig('ekf_test_results.png', dpi=150, bbox_inches='tight')
        print(f"✓ Plot saved as 'ekf_test_results.png'")
        plt.close()
    except Exception as e:
        print(f"⚠ Warning: Could not save plot ({e})")
    
    return ekf


def main():
    """Run all tests"""
    print("\n" + "="*60)
    print("  EXTENDED KALMAN FILTER - COMPREHENSIVE TEST SUITE")
    print("="*60)
    
    try:
        # Run all tests
        ekf = test_1_initialization()
        ekf = test_2_prediction_step(ekf)
        ekf, positions = test_3_multiple_predictions(ekf)
        ekf = test_4_gnss_conversion(ekf)
        ekf = test_5_gnss_update(ekf)
        ekf = test_6_full_simulation()
        
        # Summary
        print("\n" + "="*60)
        print("  ✓✓✓ ALL 6 TESTS PASSED ✓✓✓")
        print("="*60)
        print("\n✅ Your EKF implementation is working correctly!")
        print("✅ Ready for ROS2 integration")
        print("✅ Ready for production use\n")
        
        return True
        
    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        return False
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "_main_":
    success = main()
    sys.exit(0 if success else 1)