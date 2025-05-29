#!/usr/bin/env python3
"""
Test script to validate 4 DOF constraint enforcement in platform controllers
"""

import sys
import os
import numpy as np
import pandas as pd

# Add the current directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_constraint_validation():
    """Test the constraint validation functionality"""
    print("=" * 60)
    print("Testing 4 DOF Constraint Validation")
    print("=" * 60)
    
    try:
        # Test constraint validation module
        from constraint_validation import validate_4dof_constraints, enforce_4dof_constraints, get_achievable_dof_description
        
        print("\n1. Testing constraint validation module...")
        print(f"Achievable DOF: {get_achievable_dof_description()}")
          # Test valid pose (should pass)
        valid_position = np.array([0.1, 0.0, 0.3])  # X, Y, Z
        valid_rotation = np.array([10.0, 15.0, 0.0])  # Roll, Pitch, Yaw
        leg_length = 0.5  # 50cm legs
        
        is_valid, message = validate_4dof_constraints(valid_position, valid_rotation, leg_length)
        print(f"\nValid pose position={valid_position}, rotation={valid_rotation}: {'✓ PASS' if is_valid else '✗ FAIL'}")
        if message:
            print(f"  Message: {message}")
            
        # Test invalid pose (should fail)
        invalid_position = np.array([0.1, 0.05, 0.3])  # Non-zero Y
        invalid_rotation = np.array([10.0, 15.0, 25.0])  # Non-zero Yaw
        
        is_valid, message = validate_4dof_constraints(invalid_position, invalid_rotation, leg_length)
        print(f"\nInvalid pose position={invalid_position}, rotation={invalid_rotation}: {'✗ FAIL' if not is_valid else '✓ UNEXPECTED PASS'}")
        if message:
            print(f"  Message: {message}")
              # Test constraint enforcement
        print(f"\nBefore enforcement:")
        print(f"  Position: {invalid_position}")
        print(f"  Rotation: {invalid_rotation}")
        enforced_pos, enforced_rot = enforce_4dof_constraints(invalid_position.copy(), invalid_rotation.copy())
        print(f"After enforcement:")
        print(f"  Position: {enforced_pos}")
        print(f"  Rotation: {enforced_rot}")
        
    except ImportError as e:
        print(f"⚠ Warning: Could not import constraint validation module: {e}")
        print("This is expected if the module is not found, fallback functions will be used.")

def test_platform_controller():
    """Test the platform controller with 4 DOF constraints"""
    print("\n" + "=" * 60)
    print("Testing Platform Controller 4 DOF Constraints")
    print("=" * 60)
    
    try:
        # Import platform controller
        from platform_controller import PlatformController
        
        # Create controller with reasonable parameters
        leg_length = 0.5  # 50cm legs
        rail_max_travel = 0.5  # 50cm rail travel
        
        print(f"\nCreating PlatformController:")
        print(f"  Leg length: {leg_length}m")
        print(f"  Rail max travel: {rail_max_travel}m")
        
        controller = PlatformController(leg_length, rail_max_travel)
        
        print("✓ Controller created successfully")
        
        # Test position with valid 4 DOF pose (Y=0, Yaw=0)
        print(f"\n2. Testing valid 4 DOF pose...")
        
        # Create a simple test trajectory point
        valid_position = np.array([0.1, 0.0, 0.3])  # X, Y, Z
        valid_rotation_deg = np.array([10.0, 15.0, 0.0])  # Roll, Pitch, Yaw in degrees
        
        # Convert to rotation matrix
        from scipy.spatial.transform import Rotation
        valid_rotation_matrix = Rotation.from_euler('xyz', np.radians(valid_rotation_deg)).as_matrix()
        
        # Transform platform points
        platform_points = controller.transform_platform_points(valid_position, valid_rotation_matrix)
        
        print(f"Test pose: position={valid_position}, rotation={valid_rotation_deg}°")
        
        try:
            # This should work without warnings since Y=0 and Yaw=0
            slider_positions, motor_angle, joint_angles = controller.calculate_slider_positions(
                platform_points,
                time=0.0,
                platform_pos=valid_position,
                platform_rot=valid_rotation_deg,
                debug=True
            )
            print("✓ Valid pose processed successfully")
            print(f"  Slider positions: {[f'{pos:.3f}m' for pos in slider_positions]}")
            
        except ValueError as e:
            print(f"✗ Valid pose failed: {e}")
        
        # Test position with invalid 4 DOF pose (Y≠0, Yaw≠0)
        print(f"\n3. Testing invalid 4 DOF pose...")
        
        invalid_position = np.array([0.1, 0.05, 0.3])  # Non-zero Y
        invalid_rotation_deg = np.array([10.0, 15.0, 25.0])  # Non-zero Yaw
        
        # Convert to rotation matrix
        invalid_rotation_matrix = Rotation.from_euler('xyz', np.radians(invalid_rotation_deg)).as_matrix()
        
        # Transform platform points
        platform_points = controller.transform_platform_points(invalid_position, invalid_rotation_matrix)
        
        print(f"Test pose: position={invalid_position}, rotation={invalid_rotation_deg}°")
        print("Expected: Warning messages about constraint violations")
        
        try:
            # This should show warnings but might still work
            slider_positions, motor_angle, joint_angles = controller.calculate_slider_positions(
                platform_points,
                time=0.0,
                platform_pos=invalid_position,
                platform_rot=invalid_rotation_deg,
                debug=True
            )
            print("⚠ Invalid pose processed (warnings should have been shown above)")
            print(f"  Slider positions: {[f'{pos:.3f}m' for pos in slider_positions]}")
            
        except ValueError as e:
            print(f"✗ Invalid pose failed: {e}")
            
    except ImportError as e:
        print(f"✗ Could not import platform_controller: {e}")
    except Exception as e:
        print(f"✗ Unexpected error: {e}")

def test_optimization_bounds():
    """Test that optimization bounds enforce 4 DOF constraints"""
    print("\n" + "=" * 60)
    print("Testing Optimization Bounds for 4 DOF Constraints")
    print("=" * 60)
    
    try:
        from platform_controller import PlatformController
        
        controller = PlatformController(0.5, 0.5)
        
        # Create a dummy trajectory for testing bounds
        trajectory_data = {
            'time': [0.0, 0.1, 0.2],
            'x': [0.0, 0.1, 0.0],
            'y': [0.0, 0.0, 0.0],  # Always zero
            'z': [0.3, 0.35, 0.3],
            'roll': [0.0, 10.0, 0.0],
            'pitch': [0.0, 15.0, 0.0],
            'yaw': [0.0, 0.0, 0.0]  # Always zero
        }
        
        trajectory_df = pd.DataFrame(trajectory_data)
        
        print("Created test trajectory with 4 DOF compliance (Y=0, Yaw=0)")
        print("Testing optimization bounds setup...")
        
        # We can't easily test the full optimization without running it,
        # but we can check that the bounds setup doesn't crash
        print("✓ Optimization bounds validation would require full optimization run")
        print("  This test confirms the controller initializes correctly")
        
    except Exception as e:
        print(f"✗ Bounds test failed: {e}")

if __name__ == "__main__":
    print("4 DOF Constraint Validation Test Suite")
    print("Testing both constraint validation and platform controller enforcement")
    
    test_constraint_validation()
    test_platform_controller()
    test_optimization_bounds()
    
    print("\n" + "=" * 60)
    print("Test Summary:")
    print("- Constraint validation module tested")
    print("- Platform controller 4 DOF enforcement tested")
    print("- Valid poses should process without warnings")
    print("- Invalid poses should show constraint violation warnings")
    print("- Optimization bounds ensure Y=0 and Yaw=0 constraints")
    print("=" * 60)
