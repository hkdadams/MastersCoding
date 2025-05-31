#!/usr/bin/env python3
"""
Test script to validate motor torque calculation implementation
"""

import sys
import os
import numpy as np
import pandas as pd

# Add the current directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_motor_torque_calculation():
    """Test the motor torque calculation functionality"""
    print("=" * 60)
    print("Testing Motor Torque Calculation for Slider 1")
    print("=" * 60)
    
    try:
        from platform_controllerMP import PlatformController
        
        # Create controller with reasonable parameters
        leg_length = 0.5  # 50cm legs
        rail_max_travel = 0.5  # 50cm rail travel
        log_file_path = "test_torque.log"
        
        print(f"\nCreating PlatformController:")
        print(f"  Leg length: {leg_length}m")
        print(f"  Rail max travel: {rail_max_travel}m")
        
        controller = PlatformController(leg_length, rail_max_travel, log_file_path)
        
        print("✓ Controller created successfully")
        
        # Test with a simple platform pose
        print(f"\n1. Testing motor torque calculation...")
        
        # Create a test pose
        position = np.array([0.1, 0.0, 0.3])  # X, Y, Z
        rotation_deg = np.array([10.0, 15.0, 0.0])  # Roll, Pitch, Yaw in degrees
        
        # Convert to rotation matrix
        from scipy.spatial.transform import Rotation
        rotation_matrix = Rotation.from_euler('xyz', np.radians(rotation_deg)).as_matrix()
        
        # Transform platform points
        platform_points = controller.transform_platform_points(position, rotation_matrix)
        
        print(f"Test pose: position={position}, rotation={rotation_deg}°")
        
        try:
            # Calculate slider positions
            slider_positions, motor_angle, joint_angles = controller.calculate_slider_positions(
                platform_points,
                time=0.0,
                platform_pos=position,
                platform_rot=rotation_deg,
                debug=False
            )
            print("✓ Slider positions calculated successfully")
            print(f"  Slider positions: {[f'{pos:.3f}m' for pos in slider_positions]}")
            
            # Test static leg force calculation
            print(f"\n2. Testing static leg force calculation...")
            leg_forces = controller.calculate_leg_forces(platform_points, slider_positions, platform_mass=10.0)
            print("✓ Leg forces calculated successfully")
            print(f"  Leg forces: {[f'{force:.3f}N' for force in leg_forces]}")
            
            # Test motor torque calculation
            print(f"\n3. Testing motor torque calculation...")
            motor_torque = controller.calculate_motor_torque_slider1(platform_points, slider_positions, leg_forces)
            print("✓ Motor torque calculated successfully")
            print(f"  Motor torque (Slider 1): {motor_torque:.3f} N⋅m")
            
            # Test dynamic motor torque calculation
            print(f"\n4. Testing dynamic motor torque calculation...")
            platform_velocity = np.array([0.1, 0.0, 0.05])  # m/s
            platform_acceleration = np.array([0.5, 0.0, 0.2])  # m/s²
            angular_velocity = np.array([0.1, 0.2, 0.0])  # rad/s
            angular_acceleration = np.array([1.0, 2.0, 0.0])  # rad/s²
            
            motor_torque_dynamic = controller.calculate_motor_torque_with_dynamics(
                platform_points, slider_positions,
                platform_velocity=platform_velocity,
                platform_acceleration=platform_acceleration,
                angular_velocity=angular_velocity,
                angular_acceleration=angular_acceleration,
                platform_mass=10.0
            )
            print("✓ Dynamic motor torque calculated successfully")
            print(f"  Dynamic motor torque (Slider 1): {motor_torque_dynamic:.3f} N⋅m")
            
            # Compare static vs dynamic
            print(f"\n5. Comparison:")
            print(f"  Static torque:  {motor_torque:.3f} N⋅m")
            print(f"  Dynamic torque: {motor_torque_dynamic:.3f} N⋅m")
            print(f"  Difference:     {abs(motor_torque_dynamic - motor_torque):.3f} N⋅m")
            
        except ValueError as e:
            print(f"✗ Calculation failed: {e}")
        
    except ImportError as e:
        print(f"✗ Could not import platform_controllerMP: {e}")
    except Exception as e:
        print(f"✗ Unexpected error: {e}")

def test_trajectory_processing():
    """Test motor torque calculation with trajectory data"""
    print("\n" + "=" * 60)
    print("Testing Motor Torque in Trajectory Processing")
    print("=" * 60)
    
    try:
        from platform_controllerMP import PlatformController
        
        # Create controller
        controller = PlatformController(0.5, 0.5, "test_trajectory_torque.log")
        
        # Create a simple test trajectory
        trajectory_data = {
            'time': [0.0, 0.1, 0.2, 0.3],
            'x': [0.0, 0.05, 0.1, 0.05],
            'y': [0.0, 0.0, 0.0, 0.0],  # Always zero (4 DOF constraint)
            'z': [0.3, 0.32, 0.35, 0.33],
            'roll': [0.0, 5.0, 10.0, 5.0],
            'pitch': [0.0, 8.0, 15.0, 8.0],
            'yaw': [0.0, 0.0, 0.0, 0.0]  # Always zero (4 DOF constraint)
        }
        
        trajectory_df = pd.DataFrame(trajectory_data)
        
        print("Created test trajectory with 4 DOF compliance")
        print("Testing trajectory processing with motor torque calculations...")
        
        # Process without optimization to test basic functionality
        try:
            results_df = controller.process_trajectory(
                trajectory_df, 
                "test_trajectory.xlsx", 
                apply_optimization=False, 
                auto_overwrite=True
            )
            
            print("✓ Trajectory processing completed successfully")
            
            if 'motor_torque_slider1' in results_df.columns:
                print("✓ Motor torque data included in results")
                print(f"  Motor torque range: {results_df['motor_torque_slider1'].min():.3f} to {results_df['motor_torque_slider1'].max():.3f} N⋅m")
                print(f"  Mean motor torque: {results_df['motor_torque_slider1'].mean():.3f} N⋅m")
            else:
                print("✗ Motor torque data missing from results")
                
        except Exception as e:
            print(f"✗ Trajectory processing failed: {e}")
            
    except Exception as e:
        print(f"✗ Test setup failed: {e}")

if __name__ == "__main__":
    print("Motor Torque Calculation Test Suite")
    print("Testing motor torque implementation for slider 1")
    
    test_motor_torque_calculation()
    test_trajectory_processing()
    
    print("\n" + "=" * 60)
    print("Test Summary:")
    print("- Static motor torque calculation tested")
    print("- Dynamic motor torque calculation tested")
    print("- Trajectory processing with torque calculation tested")
    print("- Motor torque controls the 'nodding' motion (rotation about Y-axis)")
    print("=" * 60)
