# Archive

The plans, designs and reviews written up to 11 Oct 2026, kept unchanged. They are
not the current state: that is [../ARCHITECTURE.md](../ARCHITECTURE.md), and what is
left is [../TODO.md](../TODO.md), which links here where a design isn't built yet.
Their links to files outside this folder (README, CHANGELOG, the old `doc/` paths)
may be broken; the files are all here or in `doc/`.

- **ROBOT_PLAN.md**: the first whole-robot plan (requirements, geometry, pose,
  movement, behaviour, milestones M0-M6). M0-M3 are built; M4-M6 are in TODO.md.
- **REWORK_PLAN.md**: the rework's state and decisions until 11 Oct (the decisions
  are now in ARCHITECTURE.md §7).
- **ARCHITECTURE_PLAN.md**: the firmware designed from first principles, budgets,
  groundwork milestones (A0-A6, R0, M-S, V3). A0 and A1 are built.
- **cross_plan.md**: the review that aligned the four plans (numbers, shared types).
- **MAP_DESIGN.md**: the map as built, with its history and the VOTES/READINGS
  comparison of 10 Oct.
- **SURROUNDINGS_PLAN.md**: the first map design (cone rule, log-odds), replaced by
  MAP_DESIGN; its layers and the L2 rule were built.
- **TOF_MOTION_PLAN.md**: VL53 movement detection; its heading background (§7) is
  built in a simpler form; the confidence model, z-test and edge timing are not.
- **CAMERA_MOTION_PLAN.md**: camera tracking (features, ego-motion, movers while
  turning); not started.
- **COMMANDS_PLAN.md**: the keys and actions, built 10-11 Oct.
- **TELEMETRY_PLAN.md**: recording, DuckDB, the viewer, replay (T1-T5 built);
  the record format field by field (§2.1); T7 (resending lost datagrams) not built.
- **ROBOT_WIFI.md**: the WiFi console's plan (built).
- **REDFLAGS.md**, **REFACTORING.md**: code reviews up to 11 Oct.
- **CAMERA_STRIPES_TODO.md**: the HM0360 binning stripes investigation.
- **RobotMotionTrackingAlgorithms.md**: a Gemini conversation used as inspiration.
