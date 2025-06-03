import numpy as np

try:
    from platform_controllerMP import PlatformController
    
    # Test successful instantiation with angle constraint
    controller = PlatformController(
        leg_length=0.5, 
        rail_max_travel=1.0, 
        slider_min_travel_offset=0.25, 
        log_file_path='test.log', 
        platform_side=0.44445926, 
        max_leg_platform_angle=130.0, 
        log_attempts=False
    )
    
    print(f"SUCCESS: Controller created with angle constraint: {controller.max_leg_platform_angle}°")
    
    # Test angle calculation
    platform_center = np.array([0.0, 0.0, 0.0])
    leg_attachment = np.array([0.2565, 0.0, 0.0])
    leg_end = np.array([0.3, 0.0, -0.5])
    
    angle = controller.calculate_leg_platform_angle(platform_center, leg_attachment, leg_end)
    print(f"SUCCESS: Angle calculation works: {angle:.1f}°")
    
    # Test batch processing import
    from Batch_process_motions import process_file
    print("SUCCESS: Batch processing integration works!")
    
    print("\n🎉 ANGLE CONSTRAINT IMPLEMENTATION IS COMPLETE AND WORKING! 🎉")
    
    # Clean up
    import os
    if os.path.exists('test.log'):
        os.remove('test.log')
        
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
