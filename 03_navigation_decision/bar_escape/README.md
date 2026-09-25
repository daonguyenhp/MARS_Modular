# BAR Escape (MD3.2)

A Blind Alley Region is an explored neighborhood with no remaining open
direction toward the goal. Algorithm 2 then leaves the region along stored
sequences of bundles instead of inventing a new frontier.

Retreat target, in order:

1. owner concurrent point of the best remaining open point
2. skip that owner if the robot already stands on it (otherwise the mission
   spins: RETURN_REACHED then BAR forever)
3. entry pose `C_0`
4. if already at the entry → frontier exhausted

No funnel math lives here. Path generation is MD3.3.
