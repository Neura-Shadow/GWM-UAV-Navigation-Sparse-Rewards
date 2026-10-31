P5 M3 offline training tools (plan §4; separately authorized by the operator on 2026-10-31).
Offline only: no simulator, ROS or PX4 is started.

  p5_paths.py     import paths (P3/P4 packages read-only, the P5 package)
  p5_render.py    independent closed-form depth renderer and truth geometry
  p5_episode.py   synthetic closed loop: P4 perception, a supervisor, first-order PX4 position loop
  p5_layouts.py   seeded layouts, disjoint splits, acceptance-layout exclusion (contract §4.1)
  p5_oracle.py    privileged cost-to-go oracle and candidate labels (training time only)
  p5_dataset.py   DAgger collection with the shield always on
  p5_train.py     NumPy float64 listwise trainer (also the training subprocess entry point)
  p5_readiness.py held-out readiness against P4 (contract §4.2)
  p5_m3.py        the pipeline: run WORKDIR [--workers N --train-python PY]; export WORKDIR

The dataset is not committed: it is regenerated from the generator and seeds, and only its
SHA-256 manifest is kept next to the exported model in ../models/.
