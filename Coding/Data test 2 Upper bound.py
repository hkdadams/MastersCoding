import numpy as np
import matplotlib.pyplot as plt

# Given rotational acceleration (rad/ms²) and velocity (rad/ms)
rotational_acceleration = [
    -11.40,.91,28.76,42.63,14.66,-34.53,-25.86,115.07,349.51,505.42,424.42,134.61,-167.53,-310.16,-283.91,-197.46,-147.76,-150.73,-168.37,-168.14,-147.15,-117.40,-87.97,-62.13,-41.54,-27.74,-18.59,-7.19,10.76,31.55,49.85,66.74,87.59,114.02,139.54,154.32,155.67,150.68,145.85,139.18,125.90,106.71,86.75,69.05,51.76,32.59,14.74,3.73,-4.68,-26.48,-72.82,-135.27,-191.66,-225.23,-241.08,-267.17,-332.08,-435.23,-540.35,-601.62,-604.33,-585.16,-597.74,-643.60,-653.49,-564.11,-407.17,-287.07,-257.04,-253.31,-187.10,-73.96,-19.01,-83.09,-214.32,-311.89,-312.66,-211.62,-49.33,86.56,83.36,-106.47,-385.36,-565.06,-523.60,-306.22,-84.24,-7.50,-71.89,-152.94,-170.00,-143.11,-92.16,4.09,124.73,175.46,85.73,-88.47,-191.54,-125.25,35.72,134.80,121.90,104.96
]

rotational_velocity = [
    .02,.01,.02,.06,.09,.08,.04,.07,.30,.74,1.24,1.53,1.50,1.24,.93,.69,.53,.38,.22,.05,-.11,-.24,-.34,-.42,-.47,-.50,-.53,-.54,-.54,-.52,-.48,-.42,-.34,-.24,-.11,.04,.19,.34,.49,.64,.77,.89,.98,1.06,1.12,1.16,1.19,1.19,1.19,1.18,1.13,1.03,.86,.65,.42,.17,-.13,-.51,-1.00,-1.58,-2.18,-2.78,-3.36,-3.98,-4.64,-5.26,-5.75,-6.08,-6.35,-6.61,-6.84,-6.97,-7.00,-7.04,-7.19,-7.46,-7.78,-8.05,-8.19,-8.16,-8.06,-8.05,-8.30,-8.79,-9.36,-9.79,-9.97,-10.00,-10.03,-10.15,-10.32,-10.47,-10.59,-10.64,-10.58,-10.42,-10.27,-10.27,-10.42,-10.60,-10.65,-10.55,-10.41,-10.30
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
