import numpy as np
import matplotlib.pyplot as plt

# Given rotational acceleration (rad/ms²) and velocity (rad/ms)
rotational_acceleration = [
    128.85, 94.41, 12.42, -59.99, -48.74, 37.45, 84.22, -37.23, -310.46, -535.96, 
    -511.15, -243.99, 59.45, 211.94, 201.82, 143.74, 134.37, 180.56, 235.53, 265.49, 
    275.17, 289.60, 327.64, 387.48, 448.08, 487.09, 500.96, 504.94, 516.19, 540.97, 
    570.16, 580.87, 552.60, 488.51, 413.05, 346.69, 291.04, 236.87, 176.68, 110.02, 
    43.62, -13.59, -58.79, -97.51, -135.19, -168.79, -190.52, -198.21, -198.40, -200.32, 
    -209.55, -224.68, -237.29, -238.48, -227.99, -214.51, -207.22, -209.26, -216.90, 
    -221.42, -215.21, -199.06, -181.44, -170.25, -167.33, -168.85, -167.96, -159.16, 
    -143.27, -126.65, -115.16, -110.08, -108.82, -107.08, -101.64, -93.09, -85.52, 
    -83.19, -87.84, -97.77, -108.25, -114.40, -115.25, -114.29, -116.21, -123.78, 
    -135.97, -147.69, -153.46, -153.02, -151.69, -155.79, -168.48, -187.82, -206.73, 
    -217.91, -220.19, -218.27, -217.93, -224.70, -243.47, -273.15, -300.26, -311.90
]

rotational_velocity = [
    0.04, 0.17, 0.23, 0.20, 0.13, 0.12, 0.19, 0.24, 0.08, -0.36, -0.92, -1.31, 
    -1.40, -1.24, -1.02, -0.85, -0.72, -0.57, -0.36, -0.10, 0.17, 0.45, 0.76, 
    1.11, 1.53, 2.00, 2.50, 3.00, 3.51, 4.04, 4.59, 5.17, 5.74, 6.26, 6.71, 
    7.09, 7.41, 7.68, 7.88, 8.03, 8.10, 8.12, 8.08, 8.00, 7.89, 7.73, 7.55, 
    7.36, 7.16, 6.96, 6.76, 6.54, 6.31, 6.07, 5.83, 5.61, 5.40, 5.19, 4.98, 
    4.76, 4.54, 4.33, 4.14, 3.97, 3.80, 3.63, 3.46, 3.30, 3.15, 3.01, 2.89, 
    2.78, 2.67, 2.56, 2.46, 2.36, 2.27, 2.19, 2.10, 2.01, 1.91, 1.80, 1.68, 
    1.57, 1.45, 1.33, 1.20, 1.06, 0.91, 0.76, 0.60, 0.45, 0.29, 0.11, -0.09, 
    -0.30, -0.52, -0.74, -0.96, -1.18, -1.41, -1.67, -1.96, -2.27
]

# Convert to degrees (1 radian = 57.2958 degrees)
rad_to_deg = 57.2958

# Time step (ms to s)
dt = 0.001  # 1 ms = 0.001 s

# Compute angular displacement in degrees by integrating rotational velocity
angle_deg = np.cumsum(rotational_velocity) * dt * rad_to_deg

# Generate time axis
time = np.arange(len(rotational_velocity)) * dt

# Convert rotational velocity to degrees per second
velocity_deg = np.array(rotational_velocity) * rad_to_deg

# Convert rotational acceleration to degrees per second squared
acceleration_deg = np.array(rotational_acceleration) * rad_to_deg

# Plot acceleration, velocity, and displacement
plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(time, acceleration_deg, label="Rotational Acceleration (°/s²)", color="r")
plt.ylabel("°/s²")
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, velocity_deg, label="Rotational Velocity (°/s)", color="g")
plt.ylabel("°/s")
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time, angle_deg, label="Angular Displacement (°)", color="b")
plt.xlabel("Time (seconds)")
plt.ylabel("Degrees")
plt.legend()
plt.grid()

plt.suptitle("Rotational Motion Over Time")
plt.tight_layout()
plt.show()
