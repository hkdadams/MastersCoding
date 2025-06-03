from platform_controllerMP import PlatformController

# Test error handling
try:
    c = PlatformController(0.3, 0.5, 0.0, 'test.log', platform_side=0.2, platform_radius=0.15)
    print("ERROR: Should have raised an exception!")
except ValueError as e:
    print(f"✓ Error caught correctly: {e}")

print("All tests passed!")
