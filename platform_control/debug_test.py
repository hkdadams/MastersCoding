#!/usr/bin/env python3
print("Test 1: Basic Python")

try:
    import numpy as np
    print("Test 2: NumPy imported successfully")
    
    import sys
    sys.path.append(".")
    print("Test 3: Path setup complete")
    
    from platform_controllerMP import PlatformController
    print("Test 4: PlatformController imported successfully")
    
    controller = PlatformController(0.5, 0.5, "debug.log")
    print("Test 5: Controller created successfully")
    
    # Test the specific motor torque method
    platform_points = np.array([[0.1, 0.1, 0.3], [0.1, -0.1, 0.3], [-0.1, 0.0, 0.3]])
    slider_positions = np.array([0.2, 0.25, 0.15])
    leg_forces = np.array([30.0, 35.0, 32.0])
    
    print("Test 6: Test data created")
    
    result = controller.calculate_motor_torque_slider1(platform_points, slider_positions, leg_forces)
    print(f"Test 7: Motor torque result = {result}")
    
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
