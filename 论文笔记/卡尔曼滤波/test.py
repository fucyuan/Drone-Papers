import numpy as np
import matplotlib.pyplot as plt

# Initial state (position and velocity)
x = np.array([[0],    # Initial position
              [20]])  # Initial velocity

# State transition matrix F
dt = 1  # Time interval (1 second)
F = np.array([[1, dt],
              [0,  1]])

# Control input matrix B (assuming no external control input)
B = np.array([[0.5 * dt**2],
              [dt]])

# Measurement matrix H
H = np.array([[0.1, 0]])  # We only measure the position

# Process noise covariance matrix Q
sigma_a = 2
Q = np.array([[0.25*dt**4, 0.5*dt**3],
              [0.5*dt**3,  dt**2]]) * sigma_a**2

# Measurement noise covariance matrix R
sigma_GPS = 2
R = np.array([[sigma_GPS**2]])

# Initial error covariance matrix P
P = np.array([[1, 0],
              [0, 1]])

# Simulate true position and measurements
true_positions = []
measurements = []
kalman_estimates = []

# Simulate 20 seconds of motion
for t in range(20):
    # True position update (assuming constant velocity)
    true_position = x[0, 0] + x[1, 0] * dt
    true_positions.append(true_position)
    
    # Simulate measurement with noise
    z = true_position + np.random.normal(0, sigma_GPS)
    measurements.append(z)
    
    # Prediction step
    x = np.dot(F, x)
    P = np.dot(np.dot(F, P), F.T) + Q
    
    # Calculate Kalman Gain
    K = np.dot(np.dot(P, H.T), np.linalg.inv(np.dot(H, P).dot(H.T) + R))
    
    # Update step
    x = x + K.dot(z - np.dot(H, x))
    P = (np.eye(2) - K.dot(H)).dot(P)
    
    # Record Kalman filter estimate
    kalman_estimates.append(x[0, 0])

# Plot the results with better differentiation
plt.figure(figsize=(10, 5))
plt.plot(true_positions, label='True Position', color='blue', linewidth=2)
plt.plot(measurements, label='Measurements', linestyle='dotted', color='orange', marker='o', markersize=5)
plt.plot(kalman_estimates, label='Kalman Estimate', color='green', linewidth=2, linestyle='--')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Kalman Filter Position Tracking with Enhanced Differentiation')
plt.show()
