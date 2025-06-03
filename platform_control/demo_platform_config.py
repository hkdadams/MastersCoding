#!/usr/bin/env python3
"""
Demo script showing the new platform size configuration feature
"""

def demo_platform_config():
    """Demonstrate the platform size configuration similar to batch processing"""
    print("=" * 60)
    print("PLATFORM CONFIGURATION DEMO")
    print("=" * 60)
    
    # Simulate user inputs (like in the actual batch script)
    leg_length = 0.3  # meters
    rail_max_travel = 0.5  # meters
    slider_base_position = 0.0  # meters
    platform_side = 0.25  # meters (instead of hardcoded 0.2)
    
    print(f"\nConfiguration:")
    print(f"  Leg length: {leg_length}m")
    print(f"  Rail max travel: {rail_max_travel}m") 
    print(f"  Slider base position: {slider_base_position}m")
    print(f"  Platform side length: {platform_side}m")
    
    # Calculate platform radius for display
    platform_radius = platform_side / (3**0.5)
    print(f"  Platform radius: {platform_radius:.3f}m")
    
    print(f"\n{'-'*50}")
    print("Creating PlatformController...")
    
    # Import and create controller with new parameters
    from platform_controllerMP import PlatformController
    
    controller = PlatformController(
        leg_length=leg_length,
        rail_max_travel=rail_max_travel,
        slider_min_travel_offset=slider_base_position,
        log_file_path="demo.log",
        platform_side=platform_side  # NEW: configurable platform size
    )
    
    print("✓ Controller created successfully!")
    print(f"✓ Platform geometry configured:")
    print(f"  - Side length: {controller.PLATFORM_SIDE}m")
    print(f"  - Radius: {controller.PLATFORM_RADIUS:.3f}m")
    
    # Show platform attachment points
    import numpy as np
    points = controller.platform_points_local
    print(f"✓ Platform attachment points:")
    for i, point in enumerate(points):
        print(f"  - Leg {i+1}: ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")
    
    print(f"\n{'-'*50}")
    print("SUCCESS: Platform size is now configurable!")
    print("Users can now specify platform size when running batch processing.")

if __name__ == "__main__":
    demo_platform_config()
