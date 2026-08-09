# Line Follower — PID Firmware

Firmware for an AVR (ATmega324-family, 7.3728 MHz) line-following robot: 3 analog
IR reflectance sensors (left/center/right) on ADC channels 3/4/5, differential
two-wheel drive via `PORTB` direction pins + Timer1 PWM (`OCR1A`/`OCR1B`), LCD
for debug output.

File: `line_follower_pid_final.c`

## How error is computed

Each sensor's raw ADC reading is first **normalized** onto a common 0–255 scale
using that sensor's own calibrated min/max (see Calibration below):

```
normalized = (raw - sensor_min) * 255 / (sensor_max - sensor_min)
```

The normalized left/center/right values are then combined into a single
**weighted-position error** — a centroid across the three sensor positions:

```
error = (L * -255 + R * 255) / (L + C + R)
```

- `error ≈ 0` → line centered
- `error < 0` → line is to the left
- `error > 0` → line is to the right
- magnitude → how far off-center

Using all three sensors (rather than just `R - L`) means the center sensor's
strength pulls the estimate toward 0 when the line is genuinely centered, and
dividing by the total normalizes out overall brightness changes.

## Main loop

Runs on a fixed ~10ms cycle:

1. Read all three sensors (raw values, for calibration/debug — shown on LCD).
2. Normalize each to 0–255 using its calibration constants.
3. Handle the two edge cases first:
   - **All sensors dark** (`total < LINE_THRESH`) → line lost, run recovery.
   - **All sensors bright** → thick line / junction, go straight.
4. Otherwise compute `error` and run PID → convert to left/right wheel speeds.
5. Write speeds to the motors, delay, repeat.

The fixed delay matters: `KI`/`KD` are tuned around that timestep. The
recovery and sharp-turn branches use extra blocking delay, so they reset
`integral`, `filtered_derivative`, and `prev_error` afterward to avoid feeding
the next cycle a distorted derivative ("derivative kick").

## How PID drives the motors

PID output is a **steering correction** around a fixed `BASE_SPEED`, not a
wheel speed itself:

```
left_speed  = BASE_SPEED - pid_output
right_speed = BASE_SPEED + pid_output
```

- **P** (`KP * error`) — main steering response, proportional to how far
  off-center the line currently is.
- **I** (`KI * integral`, clamped to ±200) — corrects a persistent bias, e.g.
  one motor being systematically weaker than the other.
- **D** (`KD * filtered_derivative`) — dampens oscillation by reacting to how
  fast the error is changing; the derivative is smoothed (`D_FILTER_ALPHA`)
  before use so ADC jitter doesn't make steering twitchy.

If `|error|` exceeds `SHARP_TURN_ERR`, the bot skips differential steering
entirely and pivots on the spot (`PIVOT_SPEED`) — differential PID alone can't
turn tight enough for sharp corners before running off the line.

## Calibration (do this before tuning PID)

Your three sensors have different raw sensitivities — different min/max
readings from each other. Left uncalibrated, this biases the position
estimate toward whichever sensor happens to read higher, even when the line
is physically centered.

**How to measure each sensor's min/max:**

1. Flash the firmware as-is (`L_MIN/L_MAX/C_MIN/C_MAX/R_MIN/R_MAX` default to
   0/255, a no-op, so raw values pass straight through).
2. The LCD shows each sensor's **raw** ADC reading (not normalized) — that's
   intentional, so you can read calibration numbers directly off it.
3. For each sensor: place it fully over the background (off the line), note
   the raw value → that's `_MIN`. Place it fully over the darkest part of the
   line, note the raw value → that's `_MAX`.
4. Update the six constants near the top of the file with your measured
   values, reflash.

Everything downstream (thresholds, error calculation) automatically uses the
normalized values once these are set — no other code changes needed.

## Tuning PID gains

Recommended order, on the actual hardware:

1. Set `KI = 0`, `KD = 0`. Increase `KP` until the bot follows the line but
   oscillates/wobbles side to side.
2. Increase `KD` until the oscillation damps out (bot tracks smoothly without
   overshooting).
3. Only if you see a **persistent** lean to one side (not oscillation — a
   steady bias), add a small `KI` to correct it. Most setups need very little.

Other constants worth adjusting per-robot: `BASE_SPEED` (nominal cruising
speed), `SHARP_TURN_ERR` and `PIVOT_SPEED` (how aggressively it treats sharp
corners as pivots vs. differential steering), `LINE_THRESH` (how dark counts
as "on the line" — this is applied to normalized values, so it should stay
meaningful across sensors once calibrated).

## Known limitations

- Timing uses blocking `_delay_ms()`, not a free-running timer — the
  recovery/pivot branches' extra delay slightly distorts the loop period,
  mitigated but not eliminated by resetting PID state afterward.
- Calibration constants are hardcoded, not stored in EEPROM — reflashing is
  required to change them; there's no runtime calibration sweep.
- Position estimate is limited by having only 3 discrete sensors; more
  sensors would give a smoother, less quantized estimate.
