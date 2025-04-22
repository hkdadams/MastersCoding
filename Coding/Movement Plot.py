import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- FILE PATH SETUP ---
# Instead of using a file dialog, specify the path directly:
excel_file = r"C:/Users/hkdad/OneDrive/Documents/hka21/Year 4/Masters/Coding/RugbyIMUData1/Rugby_1221_for_blender.xlsx"  # Change this to your Excel file's full path

# --- READ THE EXCEL DATA ---
# Expected columns: AccelX, AccelY, AccelZ, RotVelX, RotVelY, RotVelZ
data = pd.read_excel(excel_file)
required_columns = ["AccelX", "AccelY", "AccelZ", "RotVelX", "RotVelY", "RotVelZ"]
for col in required_columns:
    if col not in data.columns:
        raise Exception(f"Column '{col}' not found in Excel file.")

# --- SIMULATION PARAMETERS ---
dt = 0.001  # seconds per row (1 ms)
n = len(data)
time_array = np.arange(n) * dt  # time in seconds

# Initialize arrays to store results
position = np.zeros((n, 3))  # columns: [X, Y, Z] in meters
rotation = np.zeros((n, 3))  # columns: [RotX, RotY, RotZ] in radians
velocity = np.zeros(3)       # initial linear velocity in m/s

# --- INTEGRATION (Euler method) ---
# For each row in the Excel data, update velocity, position, and rotation.
for i in range(n):
    # Get acceleration (m/s²) and rotational velocity (rad/s) for this timestep.
    ax = data.at[i, "AccelX"]
    ay = data.at[i, "AccelY"]
    az = data.at[i, "AccelZ"]
    rvx = data.at[i, "RotVelX"]
    rvy = data.at[i, "RotVelY"]
    rvz = data.at[i, "RotVelZ"]
    
    # Update linear velocity: v = v + a * dt
    velocity[0] += ax * dt
    velocity[1] += ay * dt
    velocity[2] += az * dt
    
    # Update position: p = p + v * dt (Euler integration)
    if i == 0:
        position[i, :] = np.array([0, 0, 0])
    else:
        position[i, :] = position[i - 1, :] + velocity * dt
    
    # Update rotation: theta = theta + (rotational velocity * dt)
    if i == 0:
        rotation[i, :] = np.array([0, 0, 0])
    else:
        rotation[i, :] = rotation[i - 1, :] + np.array([rvx, rvy, rvz]) * dt

# Convert rotation to degrees for easier interpretation
rotation_deg = np.degrees(rotation)

# --- PLOTTING ---
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

# Plot translation (X, Y, Z)
axs[0].plot(time_array, position[:, 0], label="X")
axs[0].plot(time_array, position[:, 1], label="Y")
axs[0].plot(time_array, position[:, 2], label="Z")
axs[0].set_xlabel("Time (s)")
axs[0].set_ylabel("Position (m)")
axs[0].set_title("Translation over Time")
axs[0].legend()
axs[0].grid(True)

# Plot rotation (converted to degrees)
axs[1].plot(time_array, rotation_deg[:, 0], label="Rotation X")
axs[1].plot(time_array, rotation_deg[:, 1], label="Rotation Y")
axs[1].plot(time_array, rotation_deg[:, 2], label="Rotation Z")
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Rotation (deg)")
axs[1].set_title("Rotation over Time")
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()
