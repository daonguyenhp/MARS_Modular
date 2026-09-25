# DAP / Funnel (MD3.3)

Paper: the escape path is a δ-approximate shortest path through the sequence
of bundles of line segments. Corners are cut only at vertices whose interior
angle is `< π` (taut-string / Lee–Preparata funnel). The path stays in the
explored free space `S_t^e`, and `L_return ≤ L_entry`.

This folder consumes **gates** from Module 2 (`left`/`right` along travel).
It does not build bundles. Empty gates mean an unconstrained corridor: the
taut path is the straight segment (JetTank DIRECT case).

The polyline is map-frame metres for the JetTank path follower. Motor commands
are not produced here.
