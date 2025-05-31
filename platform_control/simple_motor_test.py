#!/usr/bin/env python3
"""
Simple Motor Torque Test
"""

import numpy as np

def main():
    print("=== Simple Motor Torque Test ===")
    
    # Platform and leg parameters
    leg_length = 0.3  # 30cm legs
    platform_radius = 0.2 / np.sqrt(3)  # For 0.2m triangle side
    
    # Test platform point for leg 1
    P0 = np.array([platform_radius, 0.0, 0.3])  # Leg 1 at 0 degrees
    
    # Test slider position
    s1 = 0.25  # Slider position in meters
    
    # Test leg force
    F1 = 35.0  # Force in Newtons
    
    print(f"Platform attachment point: ({P0[0]:.3f}, {P0[1]:.3f}, {P0[2]:.3f}) m")
    print(f"Slider position: {s1:.3f} m")
    print(f"Leg force: {F1:.1f} N")
    
    # Slider position in 3D space
    slider_pos_3d = np.array([s1, 0.0, 0.0])
    
    # Leg vector from slider to platform attachment
    leg_vector = P0 - slider_pos_3d
    leg_length_actual = np.linalg.norm(leg_vector)
    leg_unit_vector = leg_vector / leg_length_actual
    
    print(f"Leg vector: ({leg_vector[0]:.3f}, {leg_vector[1]:.3f}, {leg_vector[2]:.3f}) m")
    print(f"Actual leg length: {leg_length_actual:.3f} m")
    
    # Force vector (tension along leg)
    force_vector = F1 * leg_unit_vector
    print(f"Force vector: ({force_vector[0]:.3f}, {force_vector[1]:.3f}, {force_vector[2]:.3f}) N")
    
    # Calculate torque using cross product: τ = r × F
    moment_arm = P0 - slider_pos_3d
    torque_vector = np.cross(moment_arm, force_vector)
    print(f"Torque vector: ({torque_vector[0]:.3f}, {torque_vector[1]:.3f}, {torque_vector[2]:.3f}) N⋅m")
    
    # Motor torque (Y-component for slider 1)
    motor_torque = torque_vector[1]
    
    print(f"\nMotor torque: {motor_torque:.3f} N⋅m")
    print(f"Motor torque magnitude: {abs(motor_torque):.3f} N⋅m")
    
    print("\nTest completed successfully!")

if __name__ == "__main__":
    main()
