# Batch Timing Fix Verification Test

## Purpose
This test verifies that the time-based batch command fix is working correctly. The fix moves time-based `add` command parsing to happen BEFORE regular `add` command parsing to ensure time parameters are properly recognized.

## The Fix Applied
**PROBLEM**: Time-based batch commands like `add m2 p1000 t2000` were being parsed by the regular `add` parser first, which only looked for `add m[num] p[steps] s[speed]` format. This caused the time parameter to be ignored and default speed (800 sps) to be used instead of calculating speed from time.

**SOLUTION**: Moved time-based `add` command parsing to occur BEFORE regular `add` command parsing, ensuring time-based commands are caught first.

## Verification Test Procedure

### Step 1: Test Direct Command (Control)
```
# Direct time-based command (should work correctly)
m2 p1000 t2000
# Expected: 500 sps (1000 steps ÷ 2 seconds)
# Look for message: "Calculated Speed: 500"
```

### Step 2: Test Time-Based Batch Command (The Fix)
```
# Clear any existing instructions
clear

# Add time-based instruction
add m2 p1000 t2000
# Expected: "Time-based instruction - Motor: 2 Steps: 1000 Time: 2000ms Calculated Speed: 500"

# Verify it was added correctly
show
# Expected: "Motor 2: 1 instructions:"
#          "  1: 1000 steps at 500 sps"

# Execute the batch
run
# Expected: Motor 2 should move at 500 sps, taking exactly 2 seconds
```

### Step 3: Test Regular Speed-Based Batch Command (Should Still Work)
```
# Clear any existing instructions
clear

# Add regular speed-based instruction
add m2 p1000 s600
# Expected: "Added instruction to M2: 1000 steps at 600 sps"

# Verify it was added correctly
show
# Expected: "Motor 2: 1 instructions:"
#          "  1: 1000 steps at 600 sps"

# Execute the batch
run
# Expected: Motor 2 should move at 600 sps
```

### Step 4: Test Mixed Time and Speed Commands
```
# Clear any existing instructions
clear

# Add both types of instructions
add m1 p1000 t2000    # Time-based: should be 500 sps
add m1 p1000 s800     # Speed-based: should be 800 sps
add m2 p2000 t1000    # Time-based: should be 2000 sps

# Verify all were added correctly
show
# Expected:
# Motor 1: 2 instructions:
#   1: 1000 steps at 500 sps
#   2: 1000 steps at 800 sps
# Motor 2: 1 instructions:
#   1: 2000 steps at 2000 sps

# Execute the batch
run
# Expected: All commands execute with their respective calculated speeds
```

## Expected Results After Fix

### Before Fix (BROKEN):
- `add m2 p1000 t2000` → 1000 steps at **800 sps** (WRONG - used default speed)
- Batch execution time: ~1.25 seconds (too fast)

### After Fix (CORRECT):
- `add m2 p1000 t2000` → 1000 steps at **500 sps** (CORRECT - calculated from time)
- Batch execution time: ~2.0 seconds (matches direct command)

## What to Look For

### Success Indicators:
1. **Time-based add commands show calculation message**: 
   `"Time-based instruction - Motor: X Steps: Y Time: Zms Calculated Speed: W"`

2. **Show command displays calculated speed**:
   Commands added with `t[time]` parameter show calculated speed, not default 800 sps

3. **Execution timing matches direct commands**:
   Batch execution takes the same time as equivalent direct command

4. **Speed-based commands still work**:
   Regular `add m[num] p[steps] s[speed]` commands continue to work normally

### Failure Indicators:
1. **No calculation message for time-based add commands**
2. **Show command displays 800 sps for time-based commands**  
3. **Batch execution timing doesn't match direct command timing**
4. **Speed-based add commands stop working**

## Technical Details

### The Root Cause
The issue was in the command parsing order in the `loop()` function:

1. **BEFORE (Broken)**: Regular `add` parsing happened first at line ~990
2. **AFTER (Fixed)**: Time-based `add` parsing happens first at line ~987

### Code Changes Made
1. **Moved time-based add parsing logic** to occur before regular add parsing
2. **Removed duplicate parsing variables** that were declared later
3. **Removed duplicate handling logic** that was unreachable due to order

This ensures that commands like `add m2 p1000 t2000` are properly recognized as time-based commands and processed with speed calculation, rather than falling through to the regular add parser that ignores the time parameter.
