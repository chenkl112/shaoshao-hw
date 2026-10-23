# Vendored HW navigation components

Source: https://github.com/Polyacetone/HWSentryNav26
Commit: `f5f941288197e14c867d711a4c4cd85bfd7a3194` (MIT, see LICENSE).

Copied from upstream `nav_executor`: all `include/nav_executor/common`,
`include/nav_executor/path_planner`, `src/common`, `src/path_planner`, plus
`path_executor/state/state_machine` and the `progress_monitor`, `safety_monitor`,
`recovery_helpers` headers/sources. The wheel-leg ROS executor/node and original
LPV model are not built. The existing omni FDDP controller remains the actuator.

Local changes to the copies:

- PathPlanner exposes its original numerical pipeline synchronously to Nav2's
  independent planner action worker. The upstream worker API remains available.
- Cooperative cancellation checks in spatial/Kino A*, guide construction,
  reference-path shortcuts and L-BFGS. A thread-local RAII scope prevents a
  canceled request's callback leaking into another request.
- MINCO provides read-only duration/coefficient access for the ROS snapshot.
- Centimeter-scale endpoints sharing a grid cell retain both exact endpoints in
  the reference path. The speed discretization always includes a midpoint so
  short stationary-to-stationary paths can accelerate and stop.
- Recovery path-integral candidates may not cross lethal/unknown cells (>=253),
  using raw-cell DDA and corner-cut rejection, with soft costs sampled at no more
  than half a map cell.
- A failed safe-point search waits for its 4 s cooldown before searching again.

Adapter code outside this directory:

- `planner_defaults.hpp` derives defaults and validation from upstream
  `nav_executor_init.cpp:load_planner_config` and the pinned YAML configuration.
- `planner.cpp` adjusts speed/acceleration for this robot, uses an all-flat
  direction field and translates scalar travel direction independently of radar
  orientation. Global Nav2 obstacles enter this flat-ground planning snapshot.
- `trajectory_bridge.hpp` carries the exact quintic curve and HW speed profile;
  immutable path stamps and geometry bind it to the corresponding Nav2 action.
- `recovery.hpp` connects the upstream tracker/monitors/FSM to omnidirectional
  translation; reverse opposes the prior travel direction. All recovery commands
  pass through the omni MPC and the independent command relay.

These adaptations are maintained locally; they are not an upstream HW release.
