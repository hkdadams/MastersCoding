"""
4 DOF CONSTRAINT VALIDATION

This module provides validation functions to ensure the platform controller
respects the mechanical constraints of the 3-leg Stewart platform variant.

MECHANICAL CONSTRAINTS:
- Y translation: IMPOSSIBLE (Leg 1 constrained to XZ plane)
- Yaw rotation: IMPOSSIBLE (Leg 1 joint prevents platform Yaw)
- Achievable: X, Z translation + Roll, Pitch rotation (4 DOF total)
"""

import numpy as np
from typing import Tuple

def validate_4dof_constraints(position: np.ndarray, rotation_deg: np.ndarray, 
                            leg_length: float, tolerance: float = 1e-6) -> Tuple[bool, str]:
    """
    Validate that the requested pose respects the 4 DOF mechanical constraints.
    
    Args:
        position: [x, y, z] platform position in meters
        rotation_deg: [roll, pitch, yaw] platform rotation in degrees
        leg_length: Maximum leg length in meters
        tolerance: Tolerance for Y and Yaw constraints (default: 1e-6)
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    x, y, z = position
    roll, pitch, yaw = rotation_deg
    
    # Check Y translation constraint (mechanically impossible)
    if abs(y) > tolerance:
        return False, f"Y translation {y:.6f}m violates mechanical constraint (must be 0)"
    
    # Check Yaw rotation constraint (mechanically impossible)
    if abs(yaw) > tolerance:
        return False, f"Yaw rotation {yaw:.6f}° violates mechanical constraint (must be 0)"
    
    # Check Z constraint (must be positive for physical reachability)
    if z < 0:
        return False, f"Z position {z:.6f}m is negative (platform below base plane)"
    
    # Check reasonable bounds for achievable DOF
    if abs(x) > leg_length:
        return False, f"X position {x:.6f}m exceeds reasonable bounds (±{leg_length:.3f}m)"
    
    if z > leg_length:
        return False, f"Z position {z:.6f}m exceeds leg length ({leg_length:.3f}m)"
    
    if abs(roll) > 60 or abs(pitch) > 60:  # Conservative mechanical limits
        return False, f"Roll ({roll:.1f}°) or Pitch ({pitch:.1f}°) exceeds safe limits (±60°)"
    
    return True, "Pose satisfies 4 DOF constraints"

def enforce_4dof_constraints(position: np.ndarray, rotation_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Enforce 4 DOF constraints by setting Y and Yaw to zero.
    
    Args:
        position: [x, y, z] platform position in meters
        rotation_deg: [roll, pitch, yaw] platform rotation in degrees
        
    Returns:
        Tuple of (corrected_position, corrected_rotation)
    """
    corrected_pos = position.copy()
    corrected_rot = rotation_deg.copy()
    
    # Force Y translation to zero (mechanically impossible)
    corrected_pos[1] = 0.0
    
    # Force Yaw rotation to zero (mechanically impossible)
    corrected_rot[2] = 0.0
    
    return corrected_pos, corrected_rot

def get_achievable_dof_description() -> str:
    """
    Return a description of the achievable degrees of freedom.
    
    Returns:
        String describing the 4 achievable DOF
    """
    return """
ACHIEVABLE DEGREES OF FREEDOM (4 total):
========================================
✓ X Translation: Full range (limited by leg length and workspace)
✗ Y Translation: MECHANICALLY IMPOSSIBLE (Leg 1 XZ plane constraint)
✓ Z Translation: Upward motion (limited by leg length)
✓ Roll Rotation: ±40° typical range (rotation about X-axis)
✓ Pitch Rotation: ±40° typical range (rotation about Y-axis)  
✗ Yaw Rotation: MECHANICALLY IMPOSSIBLE (Leg 1 joint constraint)

MECHANICAL REASON:
Leg 1 is constrained to move only in the XZ plane (Y=0) and its joint
only allows Y-axis rotation and XZ plane roll. This physically locks
the platform's Y position and prevents Yaw rotation.
"""
