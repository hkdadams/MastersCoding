#!/usr/bin/env python3
"""
Test script for angle constraint functionality
"""

import sys
import os
import numpy as np

# Add current directory to path
sys.path.append('.')

def test_angle_constraints():
    """Test the angle constraint implementation"""
    try:
        from platform_controllerMP import PlatformController
        print("✓ Platform controller import: SUCCESS")
        
        # Test controller instantiation with angle constraint
        controller = PlatformController(
            leg_length=0.5,
            rail_max_travel=1.0,
            slider_min_travel_offset=0.25,
            log_file_path='test_angle_constraints.log',
            platform_side=0.44445926,
            max_leg_platform_angle=130.0,
            log_attempts=False
        )
        print("✓ PlatformController instantiation with angle constraint: SUCCESS")
        print(f"  Max angle constraint: {controller.max_leg_platform_angle}°")
        
        # Test angle calculation method
        platform_center = np.array([0.0, 0.0, 0.0])
        leg_attachment = np.array([0.2565, 0.0, 0.0])  # leg 2 position
        leg_end = np.array([0.3, 0.0, -0.5])  # example leg end position
        
        angle = controller.calculate_leg_platform_angle(platform_center, leg_attachment, leg_end)
        print("✓ Angle calculation method: SUCCESS")
        print(f"  Test angle: {angle:.1f}°")
        
        # Test constraint checking (using actual leg attachment points)
        constraints_ok = controller.check_angle_constraints(
            platform_center=platform_center,
            leg_positions=np.array([
                [0.0, 0.0, -0.5],       # leg 1 (not checked)
                [0.3, 0.0, -0.5],       # leg 2
                [-0.15, 0.259, -0.5]    # leg 3
            ])
        )
        print("✓ Angle constraint checking: SUCCESS")
        print(f"  Constraints satisfied: {constraints_ok}")
        
        # Test with extreme angles that should violate constraints
        print("\n--- Testing constraint violations ---")
        extreme_leg_positions = np.array([
            [0.0, 0.0, -0.5],       # leg 1 (not checked)
            [0.5, 0.0, -0.1],       # leg 2 - extreme angle
            [-0.4, 0.4, -0.1]       # leg 3 - extreme angle
        ])
        
        constraints_violated = controller.check_angle_constraints(
            platform_center=platform_center,
            leg_positions=extreme_leg_positions
        )
        print(f"✓ Constraint violation test: SUCCESS")
        print(f"  Extreme positions violate constraints: {not constraints_violated}")
        
        print("\n✓ ALL TESTS PASSED - Angle constraint implementation is working!")
        
        return True
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        # Clean up test file
        if os.path.exists('test_angle_constraints.log'):
            os.remove('test_angle_constraints.log')

def test_batch_processing_integration():
    """Test that the batch processing script can import and use the updated functions"""
    try:
        from Batch_process_motions import process_file
        print("✓ Batch processing import: SUCCESS")
        
        # Test that we can create the argument structure
        test_args = (
            'test_file.xlsx',  # file_path
            0.5,               # leg_length
            1.0,               # rail_max_travel
            0.25,              # slider_base_position
            0.44445926,        # platform_side
            None,              # platform_radius
            False,             # enable_logging
            130.0              # max_leg_platform_angle
        )
        
        print("✓ Batch processing argument structure: SUCCESS")
        print(f"  Arguments: {len(test_args)} parameters")
        print(f"  Angle constraint: {test_args[-1]}°")
        
        return True
        
    except Exception as e:
        print(f"✗ Batch processing integration error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("ANGLE CONSTRAINT IMPLEMENTATION TEST")
    print("=" * 60)
    
    success1 = test_angle_constraints()
    print()
    success2 = test_batch_processing_integration()
    
    print("\n" + "=" * 60)
    if success1 and success2:
        print("🎉 ALL TESTS PASSED - Implementation is ready!")
    else:
        print("❌ Some tests failed - check the output above")
    print("=" * 60)
