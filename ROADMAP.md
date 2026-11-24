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

- Gear selection: DONE (v2). The lap works by acceleration segments: where the speed limit rises the car starts in the gear that accelerates best, or in one of the next two, upshifts at the rev limit (constant speed during the shift) and the start gear with the least time at the end of the segment is kept. Braking and holding the cornering speed have no gear (grey in the charts). TODO: user-facing info text (quasi-steady-state: no body roll/pitch, a real driver would not make such short shifts); the straight-line `accelerate` still uses a greedy gear with hysteresis.
- Lap algorithm, next steps (decided, not done):
  - Shift point: a pre-step that finds the best point to shift when it is not the rev limit (it only matters for cars with a falling torque curve; with the example car the shift at the cut gives the same lap time as the old free choice, to 0.005 s).
  - Speed during the shift: the car decelerates (drag) instead of keeping a constant speed; the shifting distance is then longer and there is a minimum distance below which shifting is worse than staying at the cut.
  - A free gear change while holding the cornering speed (plateau of the speed limit inside a rise): today the same gear continues.
  - The curvature smoothing window changes with the spacing (the width is forced to an odd number of samples: 12, 10, 9 and 8.5 m for 4, 2, 1 and 0.5 m). It moves the lap time by about 0.005 s; make it exactly `smooth_m` with weights.
