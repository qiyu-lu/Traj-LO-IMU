# Voxel map vs. likd-tree map

Same binary (`feature/kdtree-map` @ 7d18493), same configs as the bench's
`trajlio-p0` runs; only `mapping.map_type` differs. kd-tree settings:
`kd_min_dist: 0.1`, `kd_max_nn_dist: 1.0`. One run per cell, so differences of
about ±0.01 m are within run-to-run noise (TBB scheduling is not deterministic).
Time is the wall time of `trajlo_headless` for the whole sequence (offline, as
fast as possible, 16 threads).

| Sequence | APE voxel | APE kd | RPE-t voxel | RPE-t kd | time voxel (s) | time kd (s) |
|---|---:|---:|---:|---:|---:|---:|
| legkilo/indoor | 0.0568 | **0.0532** | 0.0152 | **0.0120** | 15.0 | 6.4 |
| legkilo/running | **0.0614** | 0.0750 | 0.0391 | **0.0382** | 8.8 | 5.2 |
| legkilo/corridor | **0.2043** | 0.2208 | 0.0253 | **0.0229** | 96.9 | 43.1 |
| quadruped/BuildingInside00 | 0.0928 | **0.0851** | 0.0521 | **0.0496** | 115.4 | 62.2 |
| quadruped/Rescue00 | **0.1160** | 0.1214 | **0.0343** | 0.0372 | 70.1 | 41.3 |
| quadruped/Hill00 | **0.1046** | 0.1100 | **0.0562** | 0.0564 | 72.5 | 35.5 |
| quadruped/IndoorStairwell00 | 0.1599 | **0.1304** | 0.0388 | **0.0382** | 25.9 | 13.8 |
| quadruped/OutdoorNarrowStairs00 | **0.1580** | 0.1944 | 0.0450 | **0.0440** | 123.6 | 60.9 |
| **total time** | | | | | 528.2 | 268.4 |

- Speed: the kd-tree map is about 2x faster end to end on every sequence.
- Local drift (RPE-t): equal or better on 6/8.
- Global error (APE): mixed, better on 3/8 and worse on 5/8; the largest
  regression is OutdoorNarrowStairs00 (+0.036 m).
- No divergence with either map.

Reproduce: `python3 kd_compare.py OUT_DIR [name=key:val,...]`, e.g.
`kdtree05=map_type:kdtree,kd_min_dist:0.05`.
