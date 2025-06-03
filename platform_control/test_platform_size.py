#!/usr/bin/env python3
"""
Test script to verify platform size configuration functionality
"""

from platform_controllerMP import PlatformController
import numpy as np

def test_platform_sizes():
    """Test different platform sizes"""
    test_sizes = [0.15, 0.2, 0.25, 0.3]
    
    print("Testing different platform sizes:")
    print("=" * 50)
    
    for size in test_sizes:
        controller = PlatformController(
            leg_length=0.3,
            rail_max_travel=0.5,
            slider_min_travel_offset=0.0,
            log_file_path="test.log",
            platform_side=size
        )
        
        radius = controller.PLATFORM_RADIUS
        print(f"Platform side: {size:.3f}m -> Radius: {radius:.3f}m")
        
        # Test that platform points are correctly calculated
        points = controller.platform_points_local
        actual_radius = np.linalg.norm(points[0])
        print(f"  Calculated radius from attachment points: {actual_radius:.3f}m")
        
        # Verify all points are at the same distance from center
        radii = [np.linalg.norm(point) for point in points]
        print(f"  All attachment point radii: {[f'{r:.3f}' for r in radii]}")
        print()

if __name__ == "__main__":
    test_platform_sizes()
