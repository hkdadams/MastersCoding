# Batch Timing Diagnosis

## Critical Test to Identify Remaining Issue

The core timing bug has been fixed (removed /2 from line 528), but batch mode is still reported to take twice as long. Let's systematically isolate the problem.

## Test 1: Verify Core Timing Fix
```
# Direct command should now work correctly
m1 p1000 t2000
# Expected: 2.0 seconds exactly
# Watch for: "Calculated speed: 500 sps for 1000 steps in 2000 ms"
# Watch for: "Time-based move - Motor: 1 Steps: 1000 Time: 2000ms Calculated Speed: 500"
```

## Test 2: Verify Batch Speed Calculation
```
# Test batch speed calculation
clear
add m1 p1000 t2000
show
# Expected output should show: "1000 steps at 500 sps"
# Watch for: "Calculated speed: 500 sps for 1000 steps in 2000 ms"
# Watch for: "Time-based instruction - Motor: 1 Steps: 1000 Time: 2000ms Calculated Speed: 500"
```

## Test 3: Verify Batch Execution Timing
```
# Execute the batch and time it
run
# Expected: 2.0 seconds exactly
# If it takes 4.0 seconds, there's a secondary timing bug
```

## Key Debug Messages to Watch For

### Speed Calculation Messages
- Direct: `"Time-based move - Motor: 1 Steps: 1000 Time: 2000ms Calculated Speed: 500"`
- Batch: `"Time-based instruction - Motor: 1 Steps: 1000 Time: 2000ms Calculated Speed: 500"`
- Both should show same speed (500 sps)

### Execution Messages  
- Direct: `"Starting move for Motor 1 Steps: 1000 Speed: 500"`
- Batch: `"Starting move for Motor 1 Steps: 1000 Speed: 500"`
- Both should show same speed (500 sps)

### Timing Verification
- Both movements should take exactly 2.0 seconds
- If batch takes 4.0 seconds but shows correct speed (500 sps), there's a deeper issue

## Hypothesis
If the calculated and displayed speeds are identical but the actual timing differs, the issue may be:
1. **Speed validation**: Something in validateSpeed() affecting batch differently
2. **Motor initialization**: Different initialization between direct vs batch startMotorMove()
3. **Instruction transition**: Issue in how batch transitions affect timing
4. **Hardware timing**: Possible timing accumulation error in batch mode

## Next Steps Based on Results
- **If speeds differ**: Issue is in speed calculation or validation
- **If speeds match but timing differs**: Issue is in motor execution or hardware timing
- **If batch shows wrong speed**: Issue is in batch instruction processing
- **If both work correctly**: User timing measurement error

## Important Notes
- The core timing calculation at line 528 is now correct: `delay_us = 1000000L / motors[i].speed_sps`
- The calculateSpeedFromTime() function is correct: `steps * 1000 / time_ms`
- The batch system uses the same updateMotors() function as direct commands
- All validation functions should work identically for both modes
