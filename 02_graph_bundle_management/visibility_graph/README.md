# Visibility Graph

Implements MD2.2 as an accumulated, undirected C++17 visibility graph. Local
center-to-open-point edges retain observation evidence. Cross-observation
center edges are added only after reciprocal neighbor-sight and boundary
checks. The public API exposes both true breadth-first search and
Euclidean-cost Dijkstra search.
