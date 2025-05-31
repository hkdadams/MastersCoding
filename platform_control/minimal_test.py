#!/usr/bin/env python3
import sys
import traceback

try:
    print("Starting test...")
    from platform_controllerMP import PlatformController
    print("Import successful")
    
    import numpy as np
    print("NumPy imported")
    
    from scipy.spatial.transform import Rotation
    print("Scipy imported")
    
    # Create controller
    controller = PlatformController(0.5, 0.5, 'test.log')
    print("Controller created")
    
    # Simple calculation
    position = np.array([0.0, 0.0, 0.4])  # Neutral position
    rotation_deg = np.array([0.0, 0.0, 0.0])  # No rotation
    rotation_matrix = Rotation.from_euler('xyz', np.radians(rotation_deg)).as_matrix()
    
    platform_points = controller.transform_platform_points(position, rotation_matrix)
    print("Platform points calculated")
    
    slider_positions, motor_angle, joint_angles = controller.calculate_slider_positions(
        platform_points, platform_pos=position, platform_rot=rotation_deg, debug=False
    )
    print("Slider positions calculated")
    
    leg_forces = controller.calculate_leg_forces(platform_points, slider_positions, platform_mass=10.0)
    print("Leg forces calculated")
    
    motor_torque = controller.calculate_motor_torque_slider1(platform_points, slider_positions, leg_forces)
    print(f"Motor torque calculated: {motor_torque}")
    
    print("Test completed successfully!")
    
except Exception as e:
    print(f"Error occurred: {e}")
    traceback.print_exc()
