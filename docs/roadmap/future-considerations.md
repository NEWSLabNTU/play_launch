# Future Considerations

Potential enhancements not currently planned. Items already implemented have been removed.

---

## Subcommands

- **`play_launch show`** — pretty-print record.json (node hierarchy, parameters)
- **`play_launch diff`** — compare two record.json files, highlight changes
- **`play_launch validate`** — dry-run validation (missing executables, invalid params)

---

## Web UI

- **Advanced filtering** — filter by status, search by package/namespace, saved presets
- **Composable node unload** — runtime unload buttons (requires unload service call)
- **Multi-user support** — authentication, per-user sessions, role-based access
- **Enhanced log viewer** — error sampling, follow mode, search, download

---

## Integration

- **launch_testing** — automated launch file verification, CI regression testing
- **Prometheus/Grafana** — export metrics in Prometheus format, long-term storage
- **OpenTelemetry** — distributed tracing of node communication patterns

---

## Platform

- **Jetson GPU** — per-process GPU metrics via jtop (currently system-wide only)
- **macOS** — platform-specific I/O monitoring and process management
- **Docker image** — official container for isolated testing and cloud deployment

---

## Advanced Features

- **Parameter overrides** — override params at replay time (`--param-override node:key=val`)
- **Selective replay** — replay subset of nodes (`--only node1,container3`)
- **Launch file generation** — convert record.json back to launch XML/Python
- **Remote execution** — replay on remote machines via SSH
