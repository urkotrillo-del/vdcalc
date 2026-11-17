# Roadmap (parked ideas, not in scope for the first version)

## Next
- Circuit data quality check: measure the curvature noise when a circuit is loaded and warn (or suggest smoothing).
  Noisy centre lines can cost several per cent of lap time without any visible sign.
- Pacejka tyre model, imported from a tyre file (format to be studied; the user supplies their own coefficients).
- Limiter map: which factor limits the car at each point of the lap (power, grip, drag, braking, corner).
- Balance versus speed (which axle saturates first, depending on aerodynamic balance).
- Ideal braking curve (optimal brake bias against the real one).
- "Improvement budget": how much mass to remove / power to add to gain one second on this circuit.
- Guidance on which input data deserve a more careful measurement (from the sensitivity ranking).
- Export of pages as images or reports, and of chart data as tables.
- Optimiser (e.g. final drive or brake bias that minimises lap time).
- Force-versus-slip chart (needs a Pacejka tyre).
- User-configurable "what if" parameters and percentages.
- Several cars open at once, drawn in different colours.

## Much later
- Upload telemetry or a real lap to overlay on the simulated one; mobile app to record it.
- Differential model.

## Discarded
- Single-screen layout, full g-g-v map, side-by-side two-car comparison, built-in competition circuits.

- Gear selection: DONE, dynamic programming over (point, gear) finds the gear schedule with the least lap time; downshifts while braking. TODO: user-facing info text (quasi-steady-state: no body roll/pitch, a real driver would not make such short shifts); the straight-line `accelerate` still uses a greedy gear with hysteresis.
