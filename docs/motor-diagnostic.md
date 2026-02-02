# Motor Diagnostic Guide

## Issue
- FWD command: only left wheel works
- REV command: only right wheel works

## Code Analysis
The code in `Motor.ino` looks correct:
- `forward()` calls both Motor A (left) and Motor B (right)
- `backward()` calls both Motor A (left) and Motor B (right)

## Most Likely Causes

### 1. **Wiring Issue** (Most Likely)
Check your H-bridge connections:
- Motor A (Left): enA=pin9, in1=pin8, in2=pin7
- Motor B (Right): enB=pin3, in3=pin6, in4=pin5

Verify:
- Both enable pins (9 and 3) are connected correctly
- All direction pins are connected correctly
- Power supply to H-bridge is adequate
- Ground is shared between Arduino and H-bridge

### 2. **Motor Polarity Reversed**
One motor may be wired with reversed polarity, causing it to not respond to certain direction signals.

### 3. **Damaged H-Bridge Driver**
One channel of the H-bridge might be partially damaged.

## Debugging Steps

1. **Add Serial Debug Output** - See Motor.ino for added debug prints
2. **Test Individual Motors** - Use commands like:
   - `SPEED 150`
   - `FWD 2000` (should run 2 seconds)
   - Watch which wheels move
3. **Check Enable Pins** - Measure voltage on pins 9 and 3 with multimeter
4. **Swap Motors** - Physically swap left/right motor connections to see if problem follows the motor or stays with the H-bridge channel

## Testing Commands
Open Serial Monitor (9600 baud) and try:
```
HELP
PING
FWD 2000 150
STOP
REV 2000 150
STOP
```

## Fix Applied
Added debug output to Motor.ino to show when each motor is being commanded.
