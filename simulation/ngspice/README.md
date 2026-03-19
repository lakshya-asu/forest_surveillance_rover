# Ngspice Simulations

This folder contains simplified power-stage simulations intended to support the repository narrative and design discussion.

## Files

- `buck-converter.spice`: startup and load-step behavior for the 12V to 5V buck stage
- `battery-charger.spice`: simplified solar charger current-limit and panel sweep model

## Running

```bash
cd simulation/ngspice
ngspice buck-converter.spice
ngspice battery-charger.spice
```

## Notes

- These are compact behavioral models, not vendor-validated reference designs.
- The goal is to show expected transient behavior and design intent, not to replace final power-stage validation.
- Before hardware release, the real design should be cross-checked against datasheet compensation guidance, inductor ripple targets, and thermal limits.
