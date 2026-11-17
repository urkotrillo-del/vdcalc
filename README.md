# vdcalc

A vehicle dynamics calculator. The user describes a vehicle with their own data and the program returns
dynamic results: 0-100 km/h, braking distances, top speed, cornering limits and lap time on a circuit.

It is a calculator: results are as accurate as the data you provide. The model is quasi-steady-state,
point-mass, with longitudinal and lateral load transfer, a friction ellipse and a perfect driver.

Status: core under construction. See `ROADMAP.md` for parked ideas.

```
python -m venv .venv
.venv\Scripts\activate
pip install -e ".[dev]"
python -m pytest
```
