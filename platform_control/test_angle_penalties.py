#!/usr/bin/env python3
"""
Test script to verify that angle constraint penalties are working correctly
in the optimization objective function.
"""

import numpy as np
import pandas as pd
from platform_controllerMP import PlatformController

def test_angle_constraint_penalties():
    """Test that angle constraints are properly penalized in optimization"""
    
    print("Testing angle constraint penalties in optimization...")
    
    # Create a controller with angle constraints enabled
    controller = PlatformController(
        max_leg_platform_angle=130.0  # 130 degree limit
    )
    
    # Create a simple test trajectory that should violate angle constraints
    # Use extreme positions/rotations that are likely to cause angle violations
    test_data = {
        'x': [0.0, 0.5, 1.0],  # Extreme forward positions
        'y': [0.0, 0.0, 0.0],
        'z': [0.5, 0.6, 0.7],  # High vertical positions
        'roll': [0.0, 30.0, 45.0],   # Large roll angles
        'pitch': [0.0, 30.0, 45.0],  # Large pitch angles  
        'yaw': [0.0, 0.0, 0.0]
    }
    
    # Create DataFrame
    trajectory_df = pd.DataFrame(test_data)
    
    print(f"Test trajectory has {len(trajectory_df)} points")
    print("Trajectory data:")
    print(trajectory_df)
    
    # Test the objective function with different offset combinations
    print("\nTesting objective function with angle constraint penalties...")
    
    # Test 1: Small offsets (should be feasible)
    print("\nTest 1: Small offsets")
    small_offsets = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # No offsets
    score1 = controller.objective_function(small_offsets, trajectory_df)
    print(f"Score with small offsets: {score1:.2f}")
    
    # Test 2: Large offsets (should cause angle violations)
    print("\nTest 2: Large offsets that may cause angle violations")
    large_offsets = np.array([1.0, 1.0, 0.5, 30.0, 30.0, 30.0])  # Large position and rotation offsets
    score2 = controller.objective_function(large_offsets, trajectory_df)
    print(f"Score with large offsets: {score2:.2f}")
    
    # Test 3: Extreme offsets (should definitely cause violations)
    print("\nTest 3: Extreme offsets")
    extreme_offsets = np.array([2.0, 2.0, 1.0, 60.0, 60.0, 45.0])  # Very large offsets
    score3 = controller.objective_function(extreme_offsets, trajectory_df)
    print(f"Score with extreme offsets: {score3:.2f}")
    
    # Analyze results
    print("\n" + "="*50)
    print("RESULTS ANALYSIS:")
    print("="*50)
    
    print(f"Small offsets score:  {score1:.2f}")
    print(f"Large offsets score:  {score2:.2f}")
    print(f"Extreme offsets score: {score3:.2f}")
    
    # Check if penalties are increasing as expected
    if score1 < score2 < score3:
        print("✓ SUCCESS: Scores increase with larger offsets (penalties working)")
    elif score2 >= 1e6 or score3 >= 1e6:
        print("✓ SUCCESS: Large penalty scores detected (>= 1e6), indicating constraint violations")
        if score2 >= 1e6:
            print(f"  - Large offsets triggered penalty: {score2:.0f}")
        if score3 >= 1e6:
            print(f"  - Extreme offsets triggered penalty: {score3:.0f}")
    else:
        print("⚠ WARNING: Penalty behavior may not be working as expected")
    
    # Test angle constraint checking directly
    print("\n" + "="*50)
    print("DIRECT ANGLE CONSTRAINT TEST:")
    print("="*50)
    
    # Try to calculate slider positions for an extreme case
    try:
        # Create extreme platform configuration
        extreme_position = np.array([1.5, 1.5, 0.8])
        extreme_angles = np.array([45.0, 45.0, 30.0])
        
        # Create rotation matrix
        roll, pitch, yaw = np.radians(extreme_angles)
        Rx = np.array([[1, 0, 0],
                      [0, np.cos(roll), -np.sin(roll)],
                      [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                      [0, 1, 0],
                      [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])
        rotation = Rz @ Ry @ Rx
        
        platform_points = controller.transform_platform_points(extreme_position, rotation)
        
        print(f"Testing extreme configuration:")
        print(f"Position: {extreme_position}")
        print(f"Angles: {extreme_angles}°")
        
        # Try to calculate slider positions
        slider_positions, reachable, _ = controller.calculate_slider_positions(
            platform_points,
            platform_pos=extreme_position,
            platform_rot=extreme_angles,
            debug=True  # Enable debug output to see angle constraints
        )
        
        if reachable:
            print("✓ Configuration is reachable")
            print(f"Slider positions: {slider_positions}")
        else:
            print("⚠ Configuration is not reachable")
            
    except Exception as e:
        print(f"Error in direct test: {e}")
    
    print("\n" + "="*50)
    print("TEST COMPLETE")
    print("="*50)

if __name__ == "__main__":
    test_angle_constraint_penalties()
