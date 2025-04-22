import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as ScipyRotation
# %matplotlib widget # Uncomment if using Jupyter Lab/Notebook for interactive plots

# --- Kinematic Functions (Slightly modified for integration) ---

def create_rotation_matrix(roll, pitch, yaw):
    """Creates a 3x3 rotation matrix from Euler angles (degrees)."""
    r = ScipyRotation.from_euler('zyx', [yaw, pitch, roll], degrees=True)
    return r.as_matrix()

def calculate_platform_points(T, R, platform_radius):
    """Calculates the world coordinates of the platform attachment points."""
    P_p = np.zeros((3, 3))
    angles_platform_rad = np.radians([0, 120, 240]) # Platform points relative to platform X
    P_p[0, :] = [platform_radius * np.cos(angles_platform_rad[0]), platform_radius * np.sin(angles_platform_rad[0]), 0]
    P_p[1, :] = [platform_radius * np.cos(angles_platform_rad[1]), platform_radius * np.sin(angles_platform_rad[1]), 0]
    P_p[2, :] = [platform_radius * np.cos(angles_platform_rad[2]), platform_radius * np.sin(angles_platform_rad[2]), 0]

    P_b = np.zeros((3, 3))
    T_col = T.reshape(3, 1)
    for i in range(3):
        Pi_p_col = P_p[i, :].reshape(3, 1)
        P_b[i, :] = (T_col + R @ Pi_p_col).flatten()
    return P_b # Returns [P1_base, P2_base, P3_base] coordinates

def inverse_kinematics(target_pos, target_orientation_eul, leg_length, platform_radius):
    """
    Calculates required slider positions. Returns slider positions or None if unreachable.
    """
    tx, ty, tz = target_pos
    roll, pitch, yaw = target_orientation_eul

    u = np.zeros((3, 3)) # Base rail unit vectors
    angles_base_rad = np.radians([0, 120, 240])
    u[0, :] = [np.cos(angles_base_rad[0]), np.sin(angles_base_rad[0]), 0]
    u[1, :] = [np.cos(angles_base_rad[1]), np.sin(angles_base_rad[1]), 0]
    u[2, :] = [np.cos(angles_base_rad[2]), np.sin(angles_base_rad[2]), 0]

    T = np.array(target_pos)
    R = create_rotation_matrix(roll, pitch, yaw)

    # Calculate target positions of platform points in Base Frame {B}
    P_b = calculate_platform_points(T, R, platform_radius)

    s = np.zeros(3)
    L_squared = leg_length**2
    reachable = True

    for i in range(3):
        Qi = P_b[i, :]
        ui = u[i, :]
        Qi_dot_ui = np.dot(Qi, ui)
        Qi_dot_Qi = np.dot(Qi, Qi)

        discriminant = Qi_dot_ui**2 - (Qi_dot_Qi - L_squared)

        if discriminant < 0:
            # print(f"Warning: Target pose potentially unreachable (Leg {i+1} discriminant: {discriminant:.4f})")
            reachable = False
            s[i] = np.nan # Use NaN to indicate failure for this leg
            # We can still calculate others to see partial results if needed
            # For simplicity here, we'll just flag the whole pose
            # break # Stop calculation if one leg fails

        else:
             # Choose the physically relevant solution (usually '+')
            s[i] = Qi_dot_ui + np.sqrt(discriminant)
            # Add constraint check based on rail length
            if s[i] < 0 or s[i] > RAIL_MAX_TRAVEL:
                # print(f"Warning: Slider {i+1} position {s[i]:.3f} out of range [0, {RAIL_MAX_TRAVEL}]")
                reachable = False
                # Keep calculated s[i] value for visualization, but flag as potentially invalid state

    if not reachable:
         # Return current 's' which might contain NaNs or out-of-range values
         # Let the update function handle visualization of this state
         pass

    return s, reachable # Return calculated positions and reachability flag


# --- Parameters ---
RAIL_MAX_TRAVEL = 0.5   # 50cm max slider movement from origin
RAIL_VISUAL_LENGTH = 0.6 # Slightly longer for visualization
LEG_LENGTH = 0.5        # 50cm post length
PLATFORM_SIDE = 0.1     # 10cm platform side length
PLATFORM_RADIUS = PLATFORM_SIDE / np.sqrt(3) # Circumradius

# Base rail unit vectors (used for plotting too)
u = np.zeros((3, 3))
angles_base_rad = np.radians([0, 120, 240])
u[0, :] = [np.cos(angles_base_rad[0]), np.sin(angles_base_rad[0]), 0]
u[1, :] = [np.cos(angles_base_rad[1]), np.sin(angles_base_rad[1]), 0]
u[2, :] = [np.cos(angles_base_rad[2]), np.sin(angles_base_rad[2]), 0]

# --- Visualization Setup ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.1, bottom=0.35) # Make space for sliders

# Initial pose
initial_pos = [0.0, 0.0, 0.4] # Start centered, 40cm up
initial_eul = [0.0, 0.0, 0.0] # Start level

# Plot elements (initialize with dummy data)
rail_lines = [ax.plot([], [], [], 'k-', lw=1, label='Rails' if i==0 else "")[0] for i in range(3)]
slider_points = ax.scatter([], [], [], c='blue', s=50, label='Sliders (Trucks)')
leg_lines = [ax.plot([], [], [], 'b-', lw=2, label='Legs (Posts)' if i==0 else "")[0] for i in range(3)]
platform_lines = [ax.plot([], [], [], 'r-', lw=3, label='Platform' if i==0 else "")[0] for i in range(3)]
status_text = fig.text(0.1, 0.95, '', fontsize=12, color='red')


# --- Update Function ---
def update(val):
    global slider_positions # Allow modification if needed outside

    # Get values from sliders
    tx = slider_tx.val
    ty = slider_ty.val
    tz = slider_tz.val
    roll = slider_roll.val
    pitch = slider_pitch.val
    yaw = slider_yaw.val

    target_pos = np.array([tx, ty, tz])
    target_eul = np.array([roll, pitch, yaw])

    # Calculate inverse kinematics
    slider_positions, reachable = inverse_kinematics(
        target_pos, target_eul, LEG_LENGTH, PLATFORM_RADIUS
    )

    # Update Status Text
    if not reachable:
         status_text.set_text("Warning: Pose may be unreachable or sliders out of range!")
    elif np.isnan(slider_positions).any():
         status_text.set_text("Error: Calculation failed (NaN). Pose unreachable.")
         # Optionally clear legs/platform visualization here
    else:
         status_text.set_text("")


    # Calculate points for plotting
    T = target_pos
    R = create_rotation_matrix(roll, pitch, yaw)
    platform_points_base = calculate_platform_points(T, R, PLATFORM_RADIUS) # P1, P2, P3 in base frame

    slider_coords = np.zeros((3, 3))
    for i in range(3):
         # Handle NaN case for plotting - keep slider at origin or max? Let's use origin.
         s_i = slider_positions[i] if not np.isnan(slider_positions[i]) else 0.0
         # Clamp visual position to rail limits
         s_i = np.clip(s_i, 0, RAIL_MAX_TRAVEL)
         slider_coords[i, :] = s_i * u[i, :]

    # Update plot data
    # Rails (static, but good practice if they could change)
    origin = np.array([0,0,0])
    for i in range(3):
        rail_end = RAIL_VISUAL_LENGTH * u[i, :]
        rail_lines[i].set_data([origin[0], rail_end[0]], [origin[1], rail_end[1]])
        rail_lines[i].set_3d_properties([origin[2], rail_end[2]])

    # Sliders
    slider_points._offsets3d = (slider_coords[:, 0], slider_coords[:, 1], slider_coords[:, 2])

    # Legs and Platform (only if reachable and calculated)
    if reachable and not np.isnan(slider_positions).any():
        for i in range(3):
            leg_lines[i].set_data([slider_coords[i, 0], platform_points_base[i, 0]],
                                  [slider_coords[i, 1], platform_points_base[i, 1]])
            leg_lines[i].set_3d_properties([slider_coords[i, 2], platform_points_base[i, 2]])
            leg_lines[i].set_visible(True)

        # Platform triangle
        p_indices = [(0, 1), (1, 2), (2, 0)]
        for i in range(3):
             idx1, idx2 = p_indices[i]
             platform_lines[i].set_data([platform_points_base[idx1, 0], platform_points_base[idx2, 0]],
                                        [platform_points_base[idx1, 1], platform_points_base[idx2, 1]])
             platform_lines[i].set_3d_properties([platform_points_base[idx1, 2], platform_points_base[idx2, 2]])
             platform_lines[i].set_visible(True)
    else:
        # Hide legs and platform if unreachable
        for i in range(3):
            leg_lines[i].set_visible(False)
            platform_lines[i].set_visible(False)


    # Adjust plot limits dynamically (optional, can be slow)
    # Or set fixed limits that encompass the workspace
    # ax.autoscale_view()

    fig.canvas.draw_idle()


# --- Create Sliders ---
axcolor = 'lightgoldenrodyellow'
slider_height = 0.03
slider_vspace = 0.04
start_bottom = 0.02

ax_tx = plt.axes([0.2, start_bottom + 5*slider_vspace, 0.65, slider_height], facecolor=axcolor)
ax_ty = plt.axes([0.2, start_bottom + 4*slider_vspace, 0.65, slider_height], facecolor=axcolor)
ax_tz = plt.axes([0.2, start_bottom + 3*slider_vspace, 0.65, slider_height], facecolor=axcolor)
ax_roll = plt.axes([0.2, start_bottom + 2*slider_vspace, 0.65, slider_height], facecolor=axcolor)
ax_pitch = plt.axes([0.2, start_bottom + 1*slider_vspace, 0.65, slider_height], facecolor=axcolor)
ax_yaw = plt.axes([0.2, start_bottom + 0*slider_vspace, 0.65, slider_height], facecolor=axcolor)

slider_tx = Slider(ax_tx, 'Target X (m)', -0.3, 0.3, valinit=initial_pos[0])
slider_ty = Slider(ax_ty, 'Target Y (m)', -0.3, 0.3, valinit=initial_pos[1])
slider_tz = Slider(ax_tz, 'Target Z (m)', 0.1, 0.6, valinit=initial_pos[2]) # Z must be > 0
slider_roll = Slider(ax_roll, 'Roll (deg)', -45, 45, valinit=initial_eul[0])
slider_pitch = Slider(ax_pitch, 'Pitch (deg)', -45, 45, valinit=initial_eul[1])
slider_yaw = Slider(ax_yaw, 'Yaw (deg)', -180, 180, valinit=initial_eul[2])

# Attach update function to sliders
slider_tx.on_changed(update)
slider_ty.on_changed(update)
slider_tz.on_changed(update)
slider_roll.on_changed(update)
slider_pitch.on_changed(update)
slider_yaw.on_changed(update)

# --- Final Plot Setup ---
ax.set_xlabel("X base (m)")
ax.set_ylabel("Y base (m)")
ax.set_zlabel("Z base (m)")
ax.set_title("3-Slider Platform Kinematics")

# Set reasonable fixed limits to avoid excessive rescaling
ax.set_xlim([-0.6, 0.6])
ax.set_ylim([-0.6, 0.6])
ax.set_zlim([0, 0.8])
ax.set_aspect('equal', adjustable='box') # Crucial for correct visual proportions
ax.legend(loc='upper right')
ax.grid(True)

# Initial plot draw
update(None) # Call once to draw the initial state

plt.show()