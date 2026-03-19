# Qucs-S Analog Front-End

`sensor-frontend.sch` captures the intended PIR conditioning chain:

- LM358 buffer stage
- 2nd-order Sallen-Key low-pass filter around 10 Hz
- Comparator stage with adjustable threshold for motion-event detection

## Suggested Workflow

1. Open the schematic in Qucs-S.
2. Run an AC simulation.
3. Verify the low-pass roll-off and adjust component values if you want a different PIR response profile.

## Notes

- The schematic is intentionally lightweight and focused on the analog concept rather than device-model accuracy.
- A real board revision would validate biasing, sensor output amplitude, comparator hysteresis, and rail-to-rail headroom against the selected PIR module.
