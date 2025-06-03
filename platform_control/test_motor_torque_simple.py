#!/usr/bin/env python3
"""
Simple test script to verify motor torque calculation functionality
"""

import sys
import os
import numpy as np
import pandas as pd

# Add the current directory to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_motor_torque():
    """Test the motor torque calculation for slider 1"""
    print("=" * 60)
    print("Testing Motor Torque Calculation for Slider 1")
    print("=" * 60)
    
    try:
        from platform_controllerMP import PlatformController
        
        # Create controller with test parameters
        leg_length = 0.5  # 50cm legs
        rail_max_travel = 0.5  # 50cm rail travel
        log_file = "test_torque.log"
        
        print(f"\nCreating PlatformController:")
        print(f"  Leg length: {leg_length}m")
        print(f"  Rail max travel: {rail_max_travel}m")
        
        controller = PlatformController(leg_length, rail_max_travel, 0.0, log_file)
        print("✓ Controller created successfully")
        
        # Test motor torque calculation with a simple pose
        print(f"\nTesting motor torque calculation...")
        
        # Create a test platform pose
        position = np.array([0.1, 0.0, 0.3])  # X, Y, Z (Y=0 for 4 DOF constraint)
        rotation_deg = np.array([10.0, 15.0, 0.0])  # Roll, Pitch, Yaw (Yaw=0 for 4 DOF constraint)
        
        # Convert to rotation matrix
        from scipy.spatial.transform import Rotation
        rotation_matrix = Rotation.from_euler('xyz', np.radians(rotation_deg)).as_matrix()
        
        # Transform platform points
        platform_points = controller.transform_platform_points(position, rotation_matrix)
        
        # Calculate slider positions
        slider_positions, motor_angle, joint_angles = controller.calculate_slider_positions(
            platform_points,
            time=0.0,
            platform_pos=position,
            platform_rot=rotation_deg,
            debug=False
        )
        
        print(f"Test pose: position={position}, rotation={rotation_deg}°")
        print(f"Slider positions: {[f'{pos:.3f}m' for pos in slider_positions]}")
        print(f"Motor angle: {motor_angle:.1f}°")
        
        # Test static torque calculation
        print(f"\n1. Testing static torque calculation...")
        leg_forces = controller.calculate_leg_forces(platform_points, slider_positions, platform_mass=10.0)
        print(f"   Calculated leg forces: {[f'{force:.2f}N' for force in leg_forces]}")
        
        motor_torque_static = controller.calculate_motor_torque_slider1(platform_points, slider_positions, leg_forces)
        print(f"   Motor torque (static): {motor_torque_static:.3f} N⋅m")
        
        # Test dynamic torque calculation
        print(f"\n2. Testing dynamic torque calculation...")
        platform_velocity = np.array([0.1, 0.0, 0.05])  # m/s
        platform_acceleration = np.array([0.5, 0.0, 0.2])  # m/s²
        angular_velocity = np.radians(np.array([5.0, 10.0, 0.0]))  # rad/s
        angular_acceleration = np.radians(np.array([20.0, 15.0, 0.0]))  # rad/s²
        
        motor_torque_dynamic = controller.calculate_motor_torque_with_dynamics(
            platform_points, slider_positions,
            platform_velocity=platform_velocity,
            platform_acceleration=platform_acceleration,
            angular_velocity=angular_velocity,
            angular_acceleration=angular_acceleration,
            platform_mass=10.0
        )
        print(f"   Platform velocity: {platform_velocity} m/s")
        print(f"   Platform acceleration: {platform_acceleration} m/s²")
        print(f"   Angular velocity: {np.degrees(angular_velocity)} °/s")
        print(f"   Angular acceleration: {np.degrees(angular_acceleration)} °/s²")
        print(f"   Motor torque (dynamic): {motor_torque_dynamic:.3f} N⋅m")
        
        # Test with different poses
        print(f"\n3. Testing torque for different poses...")
        test_poses = [
            ([0.0, 0.0, 0.4], [0.0, 0.0, 0.0]),    # Neutral position
            ([0.2, 0.0, 0.3], [0.0, 20.0, 0.0]),   # Forward with pitch
            ([-0.1, 0.0, 0.35], [15.0, -10.0, 0.0]), # Back with roll
        ]
        
        for i, (pos, rot) in enumerate(test_poses):
            try:
                pos_array = np.array(pos)
                rot_array = np.array(rot)
                rot_matrix = Rotation.from_euler('xyz', np.radians(rot_array)).as_matrix()
                
                platform_pts = controller.transform_platform_points(pos_array, rot_matrix)
                slider_pos, motor_ang, _ = controller.calculate_slider_positions(
                    platform_pts, platform_pos=pos_array, platform_rot=rot_array, debug=False
                )
                
                leg_f = controller.calculate_leg_forces(platform_pts, slider_pos, platform_mass=10.0)
                torque = controller.calculate_motor_torque_slider1(platform_pts, slider_pos, leg_f)
                
                print(f"   Pose {i+1}: pos={pos}, rot={rot}° → torque={torque:.3f} N⋅m")
                
            except Exception as e:
                print(f"   Pose {i+1}: FAILED - {e}")
        
        print(f"\n✓ Motor torque calculation test completed successfully!")
        
    except ImportError as e:
        print(f"✗ Could not import platform controller: {e}")
    except Exception as e:
        print(f"✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_motor_torque()
