# Timing Debug Test

## Purpose
This test will help verify if batch execution timing is actually different from direct command timing.

## Test 1: Direct Command Timing Test
```
# Test 1000 steps in 2 seconds (should be 500 sps)
m1 p1000 t2000
# Start a stopwatch when you send this command
# Measure actual time from command sent to "Move complete for M1" message
```

## Test 2: Batch Command Timing Test  
```
# Clear and add same instruction to batch
clear
add m1 p1000 t2000
show
# Verify it shows: "1000 steps at 500 sps"
run
# Start stopwatch when you send "run" command
# Measure actual time from "run" to "Move complete for M1" message
```

## Test 3: Speed Verification
```
# Check what speed was actually calculated
add m1 p1000 t2000
# Look for "Calculated speed: XXX sps for 1000 steps in 2000 ms" message
# Should show 500 sps
```

## Expected Results
- Both Test 1 and Test 2 should take exactly 2 seconds
- The calculated speed should be 500 sps in both cases
- If batch mode takes longer, there's still a timing bug

## Debugging Steps If Issue Persists

### Check Current File Status
1. Verify the main timing calculation at line 528 in updateMotors():
   ```cpp
   unsigned long delay_us = 1000000L / motors[i].speed_sps;
   ```
   Should NOT have `/2` at the end

### Check for Duplicate Speed Calculation
2. Look for any place where speed might be recalculated during batch execution

### Check Serial Debug Output
3. Compare the debug messages:
   - Direct: "Time-based move - Motor: X Steps: Y Time: Zms Calculated Speed: W"
   - Batch: "Time-based instruction - Motor: X Steps: Y Time: Zms Calculated Speed: W"
   - Both should show the same speed

## Additional Notes
- The timing calculation in `calculateSpeedFromTime()` is correct: `steps * 1000 / time_ms`
- For 1000 steps in 2000ms: 1000 * 1000 / 2000 = 500 sps
- The delay calculation should be: 1000000 / 500 = 2000 microseconds per step
- Total time: 1000 steps * 2000μs = 2,000,000μs = 2.0 seconds ✓

If batch mode is still taking 4 seconds instead of 2 seconds, there's a secondary timing bug to find.
