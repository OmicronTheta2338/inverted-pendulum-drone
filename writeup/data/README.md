# Data

Store evidence by provenance, not by the figure where it happens to be used.

```text
data/
  raw/
    simulation/   original CSVs, logs, and run metadata
    bench/        thrust-stand, sensor, restraint, and component logs
    flight/       onboard and external-reference flight logs
  processed/      tidy tables derived reproducibly from raw files
  experiment-manifest.csv
  run-metadata-template.yaml
```

Raw files are immutable after capture. If a header or timestamp is wrong, preserve the original
and fix it in `processed/` with a script. A plot should depend on processed data or a small raw
file, never on a number transcribed from a Markdown result summary.

Suggested run-directory name:

```text
YYYY-MM-DD_<platform>_<experiment>_<configuration>_seed-NNN/
```

Examples:

```text
2026-08-22_sim_swingup_double-native-noisy_seed-001/
2026-10-04_bench_motor-step_f60v-t5147_6s-run-003/
```

Use SI units in stored data. Put units in explicit column names such as `time_s`, `theta1_rad`,
and `accel_x_m_s2`.

Add every public-facing run to `experiment-manifest.csv`. One deterministic comparison may span
multiple raw directories but should share a comparison ID in its notes.

