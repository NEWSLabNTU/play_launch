---
id: 3
title: "ContainerActor god struct — five responsibilities in one 20-field struct"
status: open
type: tech-debt
severity: medium
github: https://github.com/NEWSLabNTU/play_launch/issues/3
---

`container_actor/mod.rs:126`: process lifecycle, three ROS service clients
(creation inlined in a ~250-line `handle_pending`), composable supervision,
ComponentEvent bridging, and direct web-state writes — one struct; the
5-file split all `impl` it, so the separation is cosmetic.

Fix: actor(lifecycle) / supervisor(composables) / ros_client split —
design `docs/design/executor-state-ownership.md`, plan
`docs/roadmap/phase-51-state-ownership-refactor.md`.
