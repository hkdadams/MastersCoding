#!/usr/bin/env python3
"""
Test motor torque calculation with file output
"""

import sys

try:
    # Redirect output to file and console
    log_file = open("test_output.txt", "w", encoding='utf-8')
    
    def log_print(msg):
        print(msg)
        log_file.write(msg + "\n")
        log_file.flush()
    
    log_print("Starting motor torque test...")
    
    # Import controller
    log_print("Importing PlatformController...")
    from platform_controllerMP import PlatformController
    log_print("OK Import successful")
    
    # Create controller
    log_print("Creating controller...")
    controller = PlatformController(0.5, 0.5, 0.0, "test_motor_torque.log")
    log_print("OK Controller created")
    
    # Test motor torque calculation
    log_print("Testing motor torque calculation...")
    import numpy as np
    
    # Create test data
    platform_points = np.array([
        [0.1, 0.1, 0.3],   # Point 1
        [0.1, -0.1, 0.3],  # Point 2  
        [-0.1, 0.0, 0.3]   # Point 3
    ])
    slider_positions = np.array([0.2, 0.25, 0.15])
    leg_forces = np.array([30.0, 35.0, 32.0])
    
    log_print(f"Platform points shape: {platform_points.shape}")
    log_print(f"Slider positions: {slider_positions}")
    log_print(f"Leg forces: {leg_forces}")
    
    # Calculate motor torque
    motor_torque = controller.calculate_motor_torque_slider1(
        platform_points, slider_positions, leg_forces
    )
    
    log_print(f"OK Motor torque calculated: {motor_torque:.6f} N*m")
    
    # Test with different leg forces to verify it's not returning zero
    for i, force_scale in enumerate([0.5, 1.0, 2.0]):
        scaled_forces = leg_forces * force_scale
        torque = controller.calculate_motor_torque_slider1(
            platform_points, slider_positions, scaled_forces
        )
        log_print(f"Test {i+1}: Force scale {force_scale} -> Torque: {torque:.6f} N*m")
    
    log_print("Test completed successfully!")
    log_file.close()
    
except Exception as e:
    try:
        log_print(f"ERROR: {e}")
        import traceback
        error_trace = traceback.format_exc()
        log_print(f"Traceback:")
        log_print(error_trace)
        log_file.close()
    except:
        print(f"CRITICAL ERROR: {e}")
