#!/usr/bin/env python3
"""Run Traj-LO-IMU with the voxel and the kd-tree map on a few sequences.

Configs come from the bench's trajlio-p0 runs (same calibration / params);
only dataset.pose_file_path and mapping.map_type are changed.
Usage: kd_compare.py OUT_DIR [variant=key:val,... ...]
Env: SEQS=seq1,seq2 limits the sequences, REPS=n repeats every run.
"""
import json, os, subprocess, sys, time
from pathlib import Path
import yaml

BENCH = Path("/home/sd101t/slam/my_slam/legkilo2_new/bench_all")
BIN = "/home/sd101t/slam/my_slam/traj-lio-new/Traj-LO-IMU/build/trajlo_headless"
sys.path.insert(0, "/home/sd101t/slam/my_slam/legkilo2_new/src/Leg-KILO/legkilo/scripts")
from eval_traj import evaluate  # noqa: E402

LK = Path("/home/sd101t/Downloads/backups/datasets/legkilo_dataset/groundtrue")
QD = Path("/media/sd101t/新加卷/ubuntu/dataset/Quadruped-SLAM-dataset")
SEQS = [
    ("legkilo", "indoor", LK / "indoor_tum.txt"),
    ("legkilo", "running", LK / "running_tum.txt"),
    ("legkilo", "corridor", LK / "corridor_tum.txt"),
    ("quadruped", "BuildingInside00", QD / "GT_BuildingInside00.txt"),
    ("quadruped", "Rescue00", QD / "GT_Rescue00.txt"),
    ("quadruped", "Hill00", QD / "GT_Hill00.txt"),
    ("quadruped", "IndoorStairwell00", QD / "GT_IndoorStairwell00.txt"),
    ("quadruped", "IndoorStairwell01", QD / "GT_IndoorStairwell01.txt"),
    ("quadruped", "BuildingOutside00", QD / "GT_BuildingOutside00.txt"),
    ("quadruped", "OutdoorNarrowStairs00", QD / "GT_OutdoorNarrowStairs00.txt"),
]

if os.environ.get("SEQS"):
    keep = os.environ["SEQS"].split(",")
    SEQS = [x for x in SEQS if x[1] in keep]
REPS = int(os.environ.get("REPS", "1"))

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
variants = {}
for spec in sys.argv[2:] or ["voxel=map_type:voxel", "kdtree=map_type:kdtree"]:
    name, kv = spec.split("=", 1)
    variants[name] = dict(x.split(":", 1) for x in kv.split(","))

for rep in range(REPS):
  for ds, seq, gt in SEQS:
    for name, mapping in variants.items():
        run = out / f"{name}__{ds}__{seq}" if REPS == 1 else \
            out / f"{name}__{ds}__{seq}__r{rep}"
        res_file = run / "result.json"
        if res_file.exists():
            continue
        run.mkdir(exist_ok=True)
        cfg = yaml.safe_load(open(BENCH / f"results/trajlio-p0__{ds}__{seq}/config_used.yaml"))
        cfg["dataset"]["pose_file_path"] = str(run / "traj.tum")
        for k, v in mapping.items():
            cfg["mapping"][k] = yaml.safe_load(v)
        yaml.safe_dump(cfg, open(run / "config.yaml", "w"), sort_keys=False)
        t0 = time.time()
        with open(run / "run.log", "w") as log:
            rc = subprocess.call([BIN, str(run / "config.yaml")], stdout=log,
                                 stderr=subprocess.STDOUT, timeout=3600)
        r = {"variant": name, "dataset": ds, "seq": seq, "rep": rep, "rc": rc,
             "duration_s": round(time.time() - t0, 1)}
        try:
            ev = evaluate(str(gt), str(run / "traj.tum"), align=True)
            r["ape"] = ev["ape"]["rmse"]
            r["rpe_t"] = ev["rpe_trans"]["rmse"]
        except Exception as e:  # noqa: BLE001
            r["error"] = str(e)
        json.dump(r, open(res_file, "w"), indent=1)
        print(json.dumps(r), flush=True)
