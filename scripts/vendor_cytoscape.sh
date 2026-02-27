#!/usr/bin/env bash
# Download and vendor Cytoscape.js libraries for the graph view.
#
# Downloads ESM bundles, verifies SHA-256 checksums, and writes the result to
# src/play_launch/src/web/assets/js/vendor/. Skips download if the file
# already exists with a valid checksum.
#
# Sources:
#   cytoscape        — native ESM from jsdelivr (pinned version)
#   cytoscape-fcose  — ESM bundle from esm.sh (cose-base bundled in)
#   cytoscape-expand-collapse — ESM bundle from esm.sh
#
# Usage:
#   ./scripts/vendor_cytoscape.sh           # download + verify + install
#   ./scripts/vendor_cytoscape.sh --verify  # verify existing vendor files only
#
# To upgrade a dependency:
#   1. Update the URL and version in SOURCES below
#   2. Run with --dry-run to see the new checksum
#   3. Update CHECKSUMS with the new value
#   4. Run normally to install

set -euo pipefail

VENDOR_DIR="$(cd "$(dirname "$0")/.." && pwd)/src/play_launch/src/web/assets/js/vendor"

# ─── Source URLs (pinned versions) ───────────────────────────────────────────

declare -A SOURCES=(
  [cytoscape.esm.min.js]="https://cdn.jsdelivr.net/npm/cytoscape@3.33.1/dist/cytoscape.esm.min.mjs"
  [cytoscape-fcose.js]="https://esm.sh/cytoscape-fcose@2.2.0/es2022/cytoscape-fcose.bundle.mjs"
  [cytoscape-expand-collapse.js]="https://esm.sh/cytoscape-expand-collapse@4.1.1/es2022/cytoscape-expand-collapse.bundle.mjs"
)

# ─── SHA-256 checksums of vendored files ─────────────────────────────────────

declare -A CHECKSUMS=(
  [cytoscape.esm.min.js]="9c30e9956c6e068bdd740f8e2d92be46ec9b6f89525fcd78208cd26a852e96c4"
  [cytoscape-fcose.js]="f06bce2fb9c23c429dc21f353a007f6a8fb4f16c7d44e575fcfded5c949e24ee"
  [cytoscape-expand-collapse.js]="a6d6252a52179e77352870256d2c353618d3a785dd55acb89cf3bfd48df87ff7"
)

# ─── Functions ───────────────────────────────────────────────────────────────

verify_checksum() {
  local file="$1" expected="$2"
  local actual
  actual=$(sha256sum "$file" | cut -d' ' -f1)
  if [ "$actual" != "$expected" ]; then
    echo "FAIL  $(basename "$file")"
    echo "  expected: $expected"
    echo "  actual:   $actual"
    return 1
  fi
  return 0
}

# ─── Verify mode ─────────────────────────────────────────────────────────────

if [ "${1:-}" = "--verify" ]; then
  echo "Verifying Cytoscape vendor checksums..."
  fail=0
  for name in "${!CHECKSUMS[@]}"; do
    file="$VENDOR_DIR/$name"
    if [ ! -f "$file" ]; then
      echo "MISS  $name (not found)"
      fail=1
      continue
    fi
    if verify_checksum "$file" "${CHECKSUMS[$name]}"; then
      echo "OK    $name"
    else
      fail=1
    fi
  done
  if [ "$fail" -ne 0 ]; then
    echo "Verification FAILED"
    exit 1
  fi
  echo "All checksums verified."
  exit 0
fi

# ─── Dry-run mode ────────────────────────────────────────────────────────────

if [ "${1:-}" = "--dry-run" ]; then
  echo "Downloading to compute checksums (dry run)..."
  for name in "${!SOURCES[@]}"; do
    url="${SOURCES[$name]}"
    tmpfile=$(mktemp)
    curl -sL -o "$tmpfile" "$url"
    actual=$(sha256sum "$tmpfile" | cut -d' ' -f1)
    echo "  [$name]=\"$actual\""
    rm -f "$tmpfile"
  done
  exit 0
fi

# ─── Download mode ───────────────────────────────────────────────────────────

mkdir -p "$VENDOR_DIR"

echo "Vendoring Cytoscape.js libraries..."
for name in "${!SOURCES[@]}"; do
  url="${SOURCES[$name]}"
  dest="$VENDOR_DIR/$name"
  expected="${CHECKSUMS[$name]}"

  # Skip if file already exists with valid checksum
  if [ -f "$dest" ]; then
    if verify_checksum "$dest" "$expected" 2>/dev/null; then
      echo "  SKIP  $name (checksum OK)"
      continue
    fi
    echo "  STALE $name (checksum mismatch, re-downloading)"
  fi

  echo "  GET   $name"
  curl -sL -o "$dest" "$url"

  if ! verify_checksum "$dest" "$expected"; then
    echo "Checksum mismatch for $name — aborting."
    rm -f "$dest"
    exit 1
  fi
done

echo "Done. Vendored Cytoscape files:"
ls -lh "$VENDOR_DIR"/cytoscape*.js
