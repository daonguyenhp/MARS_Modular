# Ranking and Memory

Implements MD2.1 in C++17:

- paper Section 4.3 ranking without legacy bonuses or scale normalization;
- stable global open-point identity and geometric duplicate merging;
- explicit active, selected, reached, explored, reactivated, and invalid state;
- deterministic ranking tie breaks.

Public declarations are under `include/mars/graph_bundle_management`.
