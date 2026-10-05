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

## kd_min_dist sweep (3 repeats each)

5 sequences, every configuration run 3 times (repeats interleaved). Mean APE
in meters; the range over the 3 runs is given where it is not zero.

| Sequence | voxel | kd 0.05 | kd 0.10 | kd 0.20 |
|---|---:|---:|---:|---:|
| legkilo/indoor | 0.0568 | **0.0523** | 0.0532 | 0.0542 |
| legkilo/running | **0.0612** (0.061-0.062) | 0.0694 | 0.0750 | 0.0669 |
| quadruped/IndoorStairwell00 | 0.1599 | 0.1322 | **0.1304** | 0.1645 |
| quadruped/BuildingInside00 | 0.0928 | 0.0900 | **0.0851** | 0.0900 |
| quadruped/OutdoorNarrowStairs00 | 0.3044 (0.168-0.556) | **0.1776** | 0.1944 | 0.2193 |
| mean RPE-t, 5 seqs | 0.0393 | **0.0361** | 0.0364 | 0.0399 |
| total time (s) | 297.1 | **135.9** | 146.0 | 153.7 |

- The speed-up is not from a sparser map: the densest setting (0.05) is the
  fastest. It comes from the exact k-NN replacing the gather-and-sort over 7
  voxels (up to 140 points) per query.
- kd 0.05 has lower APE than the voxel map on 4/5 sequences and lower RPE-t on
  all 5; the exception is `running` (+0.008 m).
- Repeatability: all kd-tree runs gave identical results over 3 repeats. The
  voxel map did too except on OutdoorNarrowStairs00, where APE ranged from
  0.168 to 0.556 m.
- 0.2 is too sparse: APE and RPE return to voxel level or worse.

Default `kd_min_dist` set to 0.05.

## README sequences with the default (kd 0.05)

Single runs, for the README table: Rescue00 0.1204, IndoorStairwell01 0.1608,
BuildingOutside00 0.0950 (APE, m); the other five are from the sweep above.
