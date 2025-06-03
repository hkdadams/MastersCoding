#!/usr/bin/env python3
"""
Demo script showing the enhanced platform size configuration with dual parameter support
"""

def demo_dual_parameter_support():
    """Demonstrate the new dual parameter support for platform size"""
    from platform_controllerMP import PlatformController
    import math
    
    print("=" * 70)
    print("ENHANCED PLATFORM SIZE CONFIGURATION DEMO")
    print("=" * 70)
    
    print("\n🔧 NEW FEATURE: You can now specify platform size using either:")
    print("   • Side length (length of triangle edges)")
    print("   • Radius (distance from center to attachment points)")
    print()
    
    # Test 1: Using side length
    print("📐 METHOD 1: Specifying Side Length")
    print("-" * 40)
    side_length = 0.25
    controller1 = PlatformController(
        leg_length=0.3,
        rail_max_travel=0.5,
        slider_min_travel_offset=0.0,
        log_file_path="demo.log",
        platform_side=side_length
    )
    print(f"Input: platform_side = {side_length}m")
    print(f"Result: Side = {controller1.PLATFORM_SIDE}m, Radius = {controller1.PLATFORM_RADIUS:.3f}m")
    
    # Test 2: Using radius
    print(f"\n📏 METHOD 2: Specifying Radius")
    print("-" * 40)
    radius = 0.15
    controller2 = PlatformController(
        leg_length=0.3,
        rail_max_travel=0.5,
        slider_min_travel_offset=0.0,
        log_file_path="demo.log",
        platform_radius=radius
    )
    print(f"Input: platform_radius = {radius}m")
    print(f"Result: Side = {controller2.PLATFORM_SIDE:.3f}m, Radius = {controller2.PLATFORM_RADIUS}m")
    
    # Test 3: Default behavior
    print(f"\n⚙️  METHOD 3: Using Default Values")
    print("-" * 40)
    controller3 = PlatformController(
        leg_length=0.3,
        rail_max_travel=0.5,
        slider_min_travel_offset=0.0,
        log_file_path="demo.log"
    )
    print(f"Input: (no platform size specified)")
    print(f"Result: Side = {controller3.PLATFORM_SIDE}m, Radius = {controller3.PLATFORM_RADIUS:.3f}m (default)")
    
    # Test 4: Error handling
    print(f"\n⚠️  ERROR HANDLING: Specifying Both Parameters")
    print("-" * 40)
    try:
        controller4 = PlatformController(
            leg_length=0.3,
            rail_max_travel=0.5,
            slider_min_travel_offset=0.0,
            log_file_path="demo.log",
            platform_side=0.2,
            platform_radius=0.15
        )
        print("❌ ERROR: Should have raised an exception!")
    except ValueError as e:
        print(f"✅ Correctly prevented: {e}")
    
    print(f"\n📊 COMPARISON TABLE:")
    print("-" * 40)
    print(f"{'Method':<15} {'Input':<15} {'Side (m)':<10} {'Radius (m)':<12}")
    print("-" * 40)
    print(f"{'Side length':<15} {side_length:<15} {controller1.PLATFORM_SIDE:<10} {controller1.PLATFORM_RADIUS:<12.3f}")
    print(f"{'Radius':<15} {radius:<15} {controller2.PLATFORM_SIDE:<10.3f} {controller2.PLATFORM_RADIUS:<12}")
    print(f"{'Default':<15} {'(none)':<15} {controller3.PLATFORM_SIDE:<10} {controller3.PLATFORM_RADIUS:<12.3f}")
    
    print(f"\n🎯 GEOMETRIC RELATIONSHIP:")
    print("-" * 40)
    print(f"For an equilateral triangle:")
    print(f"• Radius = Side / √3 ≈ Side / 1.732")
    print(f"• Side = Radius × √3 ≈ Radius × 1.732")
    print(f"• The radius is the distance from center to vertices (attachment points)")
    
    print(f"\n✨ USAGE IN BATCH PROCESSING:")
    print("-" * 40)
    print("When running Batch_process_motions.py, you'll now see:")
    print("  [1] Side length (default)")
    print("  [2] Radius")
    print("Choose your preferred input method!")

if __name__ == "__main__":
    demo_dual_parameter_support()
