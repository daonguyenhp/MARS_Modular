# Explore (MD3.1)

Paper Algorithm 2 / Section 4.3: from the current concurrent point `C_t`,
drive toward the highest-ranked active open point in `O_t^g` that still has a
collision-free skeleton on `G_t^e`.

Ranking is Module 2. This folder only **selects**. Goal-in-sight (Section 5,
Eq. 3) is a caller flag: Module 1 proves the segment, Module 3 does not ray-cast.

The last hop is capped at `max_step` (JetTank 0.70 m) so successive scans keep
overlapping vision disks. A capped multi-hop whose end lies inside
`goal_tolerance` of the current pose is skipped; that step would retrace and
stop on the robot.
