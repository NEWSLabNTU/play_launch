#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

MAP_PATH="${MAP_PATH:-$HOME/autoware_map/sample-map-planning}"
SERVICE_NAME="autoware-demo"

if [ ! -f "$SCRIPT_DIR/cyclonedds.xml" ]; then
    echo "ERROR: CycloneDDS configuration file not found at $SCRIPT_DIR/cyclonedds.xml"
    exit 1
fi

# Stop existing service if running
systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true

# Build systemd-run command
SYSTEMD_CMD=(systemd-run --user
    --unit="$SERVICE_NAME"
    --description="Autoware Demo (Simulator + Autonomous Test)"
    --collect
    --setenv=MAP_PATH="$MAP_PATH"
    --working-directory="$SCRIPT_DIR"
)

# Only set DISPLAY if it's not empty
if [ -n "${DISPLAY}" ]; then
    SYSTEMD_CMD+=(--setenv=DISPLAY="${DISPLAY}")
fi

# Start service
"${SYSTEMD_CMD[@]}" bash "$SCRIPT_DIR/scripts/start-demo-inner.sh" 2>&1 | grep -q "Running as unit" && \
echo "✓ Demo service started (Web UI: http://localhost:8080)" || \
echo "✗ Failed to start demo service"
