# Enhanced Platform Size Configuration - Dual Parameter Support

## Overview
The PlatformController now supports **dual parameter input** for platform size configuration. You can specify the platform geometry using **either** the side length **or** the radius of the triangular platform.

## New Functionality

### PlatformController Constructor
```python
PlatformController(
    leg_length: float,
    rail_max_travel: float, 
    slider_min_travel_offset: float,
    log_file_path: str,
    platform_side: float = None,        # NEW: Optional side length
    platform_radius: float = None,      # NEW: Optional radius
    log_attempts: bool = True
)
```

### Usage Options

#### Option 1: Specify Side Length
```python
controller = PlatformController(
    leg_length=0.3,
    rail_max_travel=0.5,
    slider_min_travel_offset=0.0,
    log_file_path="debug.log",
    platform_side=0.25  # 25cm triangle sides
)
# Result: Side = 0.25m, Radius ≈ 0.144m
```

#### Option 2: Specify Radius
```python
controller = PlatformController(
    leg_length=0.3,
    rail_max_travel=0.5,
    slider_min_travel_offset=0.0,
    log_file_path="debug.log",
    platform_radius=0.15  # 15cm from center to attachment points
)
# Result: Side ≈ 0.260m, Radius = 0.15m
```

#### Option 3: Use Default
```python
controller = PlatformController(
    leg_length=0.3,
    rail_max_travel=0.5,
    slider_min_travel_offset=0.0,
    log_file_path="debug.log"
)
# Result: Side = 0.2m, Radius ≈ 0.115m (default)
```

## Geometric Relationships

For an **equilateral triangle**:
- **Radius** = Side Length / √3 ≈ Side Length / 1.732
- **Side Length** = Radius × √3 ≈ Radius × 1.732

### What is the Radius?
The radius is the **circumradius** - the distance from the **center of the triangle** to each of the **three leg attachment points** (vertices).

## Enhanced Batch Processing

When running `Batch_process_motions.py`, users now see:

```
PLATFORM SIZE CONFIGURATION
--------------------------------------------------
You can specify the platform size using either:
  1. Side length (length of each edge of the triangular platform)
  2. Radius (distance from center to leg attachment points)

Choose your preferred input method:
  [1] Side length (default)
  [2] Radius

Enter choice (1 or 2, default 1): 
```

### Side Length Mode
```
Platform side length examples:
  • 0.2m: Default side length (radius ≈ 0.115m)
  • 0.3m: Larger platform (radius ≈ 0.173m)
  • 0.15m: Smaller platform (radius ≈ 0.087m)

Enter platform side length in meters (default 0.2): 0.25
✓ Platform radius will be: 0.144m
```

### Radius Mode
```
Platform radius examples:
  • 0.115m: Default radius (side ≈ 0.2m)
  • 0.173m: Larger radius (side ≈ 0.3m)
  • 0.087m: Smaller radius (side ≈ 0.15m)

Enter platform radius in meters (default 0.115): 0.15
✓ Platform side length will be: 0.260m
```

## Error Handling

The system prevents invalid configurations:
```python
# This will raise ValueError
controller = PlatformController(
    leg_length=0.3,
    rail_max_travel=0.5,
    slider_min_travel_offset=0.0,
    log_file_path="debug.log",
    platform_side=0.2,     # ❌ Cannot specify both
    platform_radius=0.15   # ❌ Cannot specify both
)
# ValueError: Cannot specify both platform_side and platform_radius. Choose one.
```

## Benefits

1. **Flexibility**: Choose the most intuitive parameter for your application
2. **Precision**: Direct radius input for applications where attachment point distance is critical
3. **Backward Compatibility**: Existing code continues to work unchanged
4. **User-Friendly**: Clear prompts and automatic conversion between parameters
5. **Error Prevention**: Validation prevents conflicting parameters

## Testing

Verified functionality:
- ✅ Side length input calculates correct radius
- ✅ Radius input calculates correct side length  
- ✅ Default behavior maintained
- ✅ Error handling for dual parameters
- ✅ Batch processing script updated
- ✅ All existing test files compatible

The enhanced platform size configuration is ready for use!
