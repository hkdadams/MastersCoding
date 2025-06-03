from platform_controllerMP import PlatformController
import numpy as np
from scipy.spatial.transform import Rotation

# Create controller
controller = PlatformController(0.5, 0.5, 0.0, 'test.log')

# Test pose
position = np.array([0.1, 0.0, 0.3])
rotation_deg = np.array([10.0, 15.0, 0.0])
rotation_matrix = Rotation.from_euler('xyz', np.radians(rotation_deg)).as_matrix()

# Calculate kinematics
platform_points = controller.transform_platform_points(position, rotation_matrix)
slider_positions, motor_angle, joint_angles = controller.calculate_slider_positions(
    platform_points, platform_pos=position, platform_rot=rotation_deg, debug=False
)

# Calculate forces and torque
leg_forces = controller.calculate_leg_forces(platform_points, slider_positions, platform_mass=10.0)
motor_torque = controller.calculate_motor_torque_slider1(platform_points, slider_positions, leg_forces)

print(f"Test Results:")
print(f"Position: {position}")
print(f"Rotation: {rotation_deg}°")
print(f"Slider positions: {slider_positions}")
print(f"Motor angle: {motor_angle:.1f}°")
print(f"Leg forces: {leg_forces}")
print(f"Motor torque: {motor_torque:.6f} N⋅m")

# Check leg vector and force components
slider1_pos = controller.rail_vectors[0] * slider_positions[0]
leg1_vector = platform_points[0] - slider1_pos
leg1_force_vector = leg_forces[0] * (leg1_vector / np.linalg.norm(leg1_vector))

print(f"\nDebug Info:")
print(f"Slider 1 position: {slider1_pos}")
print(f"Leg 1 vector: {leg1_vector}")
print(f"Leg 1 force vector: {leg1_force_vector}")
print(f"r_z * F_x = {leg1_vector[2]:.6f} * {leg1_force_vector[0]:.6f} = {leg1_vector[2] * leg1_force_vector[0]:.6f}")
print(f"r_x * F_z = {leg1_vector[0]:.6f} * {leg1_force_vector[2]:.6f} = {leg1_vector[0] * leg1_force_vector[2]:.6f}")
print(f"Motor torque = r_z*F_x - r_x*F_z = {motor_torque:.6f} N⋅m")
