#!/usr/bin/env python3
"""
Very basic test to check imports and motor torque calculation
"""

print("Starting basic test...")

try:
    print("1. Testing import...")
    from platform_controllerMP import PlatformController
    print("   ✓ Import successful")
    
    print("2. Creating controller...")
    controller = PlatformController(0.5, 0.5, "test.log")
    print("   ✓ Controller created")
    
    print("3. Testing basic motor torque calculation...")
    import numpy as np
    
    # Simple test data
    platform_points = np.array([[0.1, 0.1, 0.3], [0.1, -0.1, 0.3], [-0.1, 0.0, 0.3]])
    slider_positions = np.array([0.2, 0.25, 0.15])
    leg_forces = np.array([30.0, 35.0, 32.0])
    
    torque = controller.calculate_motor_torque_slider1(platform_points, slider_positions, leg_forces)
    print(f"   ✓ Motor torque calculated: {torque:.3f} N⋅m")
    
    print("\nBasic test completed successfully!")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
