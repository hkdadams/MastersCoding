# Time-Based Movement Commands Test Guide

## Overview
This guide covers testing the new time-based movement commands that allow you to specify movement duration instead of speed. The system automatically calculates the required speed based on the distance and time specified.

## New Commands Added

### 1. Basic Time-Based Movement
**Command:** `m[num] p[steps] t[time_ms]`
**Example:** `m1 p2000 t5000`
- Moves Motor 1 by 2000 steps in 5000 milliseconds (5 seconds)
- System calculates speed: 2000 steps ÷ 5 seconds = 400 steps per second

### 2. Time-Based Absolute Positioning  
**Command:** `goto m[num] p[mm] t[time_ms]`
**Example:** `goto m1 p25.5 t3000`
- Moves Motor 1 to absolute position 25.5mm in 3000 milliseconds (3 seconds)
- Requires motor to be homed first
- System calculates required steps and speed automatically

### 3. Time-Based Batch Instructions
**Command:** `add m[num] p[steps] t[time_ms]`
**Example:** `add m1 p1000 t2000`
- Queues an instruction for Motor 1 to move 1000 steps in 2000 milliseconds (2 seconds)
- Use with `run` command to execute all queued instructions

## Test Procedure

### Prerequisites
1. Connect to the motor control system via Serial (115200 baud)
2. Ensure motors are properly connected and configured
3. Clear any emergency stop conditions with `resetstalls`
4. Enable motors if needed with `enableall`

### Test 1: Basic Time-Based Movement
```
# Test short, fast movement
m1 p500 t1000
# Expected: Motor 1 moves 500 steps in 1 second (500 sps)

# Test longer, slower movement  
m2 p1000 t5000
# Expected: Motor 2 moves 1000 steps in 5 seconds (200 sps)

# Test very slow movement
m3 p100 t10000
# Expected: Motor 3 moves 100 steps in 10 seconds (10 sps)
```

### Test 2: Speed Validation Integration
```
# Test speed exceeding motor limits (assuming 3000 sps max)
m1 p6000 t1000
# Expected: System calculates 6000 sps but limits to 3000 sps max

# Test very slow movement (should work)
m1 p10 t10000
# Expected: System calculates 1 sps and executes
```

### Test 3: Position Limit Integration
```
# Check current position
positions

# Test movement that would exceed position limits
m1 p20000 t5000
# Expected: System validates position and either limits or rejects move

# Test valid movement within limits
m1 p1000 t2000
# Expected: Movement executes normally
```

### Test 4: Time-Based Absolute Positioning
```
# Home a motor first
home m1

# Move to specific position in specified time
goto m1 p10.0 t3000
# Expected: Motor moves to 10.0mm position in 3 seconds

# Move to another position
goto m1 p-5.0 t2000  
# Expected: Motor moves to -5.0mm position in 2 seconds

# Test without homing (should fail)
goto m2 p10.0 t3000
# Expected: Error message about motor not being homed
```

### Test 5: Time-Based Batch Instructions
```
# Clear any existing instructions
clear

# Add multiple time-based instructions
add m1 p800 t2000
add m1 p-400 t1000
add m2 p1200 t3000
add m3 p600 t1500

# View queued instructions
show

# Execute all instructions
run
# Expected: All three motors execute their sequences with calculated speeds
```

### Test 6: Error Handling
```
# Test zero time (should default to 800 sps)
m1 p1000 t0

# Test negative time (should default to 800 sps)
m1 p1000 t-500

# Test zero steps (should default to 800 sps)
m1 p0 t2000

# Test invalid motor number
m4 p1000 t2000
# Expected: Command should be rejected
```

### Test 7: Mixed Speed and Time Commands
```
# Test that both command types work together
m1 p1000 s500     # Speed-based command
m2 p1000 t2000    # Time-based command
m3 p1000 s1000    # Speed-based command

# Verify both types can be queued
clear
add m1 p500 s800     # Speed-based
add m1 p500 t1000    # Time-based
add m2 p1000 t3000   # Time-based  
add m2 p800 s600     # Speed-based
show
run
```

## Expected System Responses

### Speed Calculation Messages
When using time-based commands, you should see messages like:
```
Calculated speed: 400 sps for 2000 steps in 5000 ms
Time-based move - Motor: 1 Steps: 2000 Time: 5000ms Calculated Speed: 400
```

### Speed Validation Messages
If calculated speed exceeds limits:
```
Warning: Requested speed 6000 exceeds maximum for Motor 1 (3000 sps). Limiting to 3000 sps
```

### Position Validation Messages
If movement would exceed position limits:
```
Error: Move would exceed position limit for Motor 1. Current: 5000 steps, Requested: 15000 steps, Would result in: 20000 steps (limit: ±14000 steps)
Limiting move to 9000 steps to stay within position limits
```

## Integration with Existing Features

### Emergency Stop
- Time-based commands respect emergency stop status
- Use `stop` to halt all movements, `resetstalls` to clear

### Stall Detection  
- Time-based movements integrate with stall detection system
- Stalled motors will be reported and stopped

### Position Tracking
- All time-based movements update position tracking
- Use `positions` to view current motor positions

### Speed Limits
- Time-based commands respect configured speed limits
- Use `showspeeds` to view current limits
- Use `setspeed` to modify limits

### Position Limits
- Time-based commands respect position limits
- Use `showlimits` to view current limits
- Use `setlimit` to modify limits

## Troubleshooting

### Common Issues
1. **"Motor not homed" error for goto commands**
   - Solution: Use `home m[num]` command first

2. **Calculated speed too high**
   - The system will automatically limit to maximum allowed speed
   - Consider using longer time duration for large movements

3. **Calculated speed too low (less than 1 sps)**
   - System will use minimum 1 sps
   - Consider shorter time duration for small movements

4. **Position limit violations**
   - System will either limit the movement or reject it
   - Check current position with `positions`
   - Adjust position limits with `setlimit` if needed

### Verification Commands
- `status` - Check overall system status
- `positions` - View current motor positions  
- `showspeeds` - View speed limits
- `showlimits` - View position limits
- `help` - Complete command reference

## Performance Notes
- Minimum practical speed: 1 sps
- Maximum speed: Limited by motor configuration (default 3000 sps)
- Time precision: Milliseconds (1ms minimum)
- The system maintains the same precision and safety features as speed-based commands

This completes the implementation of time-based movement commands for your motor control system!
