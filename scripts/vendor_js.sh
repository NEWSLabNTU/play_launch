#!/usr/bin/env bash
# Download and vendor Preact JS dependencies for the web UI.
#
# Downloads ESM bundles from jsdelivr, verifies SHA-256 checksums against
# pinned values, rewrites internal import specifiers to relative paths,
# and writes the result to src/play_launch/src/web/assets/js/vendor/.
#
# Usage:
#   ./scripts/vendor_js.sh           # download + verify + install
#   ./scripts/vendor_js.sh --verify  # verify existing vendor files only
#
# To upgrade a dependency:
#   1. Update the URL and version in SOURCES below
#   2. Run with --dry-run to see the new checksum
#   3. Update CHECKSUMS_AFTER_REWRITE with the new value
#   4. Run normally to install

set -euo pipefail

VENDOR_DIR="$(cd "$(dirname "$0")/.." && pwd)/src/play_launch/src/web/assets/js/vendor"

# ─── Source URLs (pinned versions) ───────────────────────────────────────────

declare -A SOURCES=(
  [preact.module.js]="https://cdn.jsdelivr.net/npm/preact@10.25.4/dist/preact.module.js"
  [hooks.module.js]="https://cdn.jsdelivr.net/npm/preact@10.25.4/hooks/dist/hooks.module.js"
  [signals-core.module.js]="https://cdn.jsdelivr.net/npm/@preact/signals-core@1.8.0/dist/signals-core.module.js"
  [signals.module.js]="https://cdn.jsdelivr.net/npm/@preact/signals@1.3.1/dist/signals.module.js"
  [htm.module.js]="https://cdn.jsdelivr.net/npm/htm@3.1.1/dist/htm.module.js"
)

# ─── Checksums of upstream originals (before import rewriting) ───────────────

declare -A CHECKSUMS_UPSTREAM=(
  [preact.module.js]="1a1af6db5b7549506c0247211860a322db7145cd254cd7f1781daf3ece7a54ab"
  [hooks.module.js]="2cb84210e9d5f865f7533eee5679005fad485761a847e2461e43638b9466479b"
  [signals-core.module.js]="04524816f4a89856fe221b9c1debb03711a7c2d32ca6d79f741869b961b330da"
  [signals.module.js]="0d7bc8db24f5d9a12bf1eda49e7eb54e6e7c9acca3252e6ca5bc9c2cf091f9f0"
  [htm.module.js]="ab33dd3f38059b9be4d5f5350128eefb2356639c4e0bbe9d9e8b3ba75847e9e4"
)

# ─── Checksums of final vendored files (after import rewriting) ──────────────

declare -A CHECKSUMS_AFTER_REWRITE=(
  [preact.module.js]="1a1af6db5b7549506c0247211860a322db7145cd254cd7f1781daf3ece7a54ab"
  [hooks.module.js]="bc44c245deff716e4e2693bad8a32c83c91f1d758e490eb8db10123bacb8fecb"
  [signals-core.module.js]="04524816f4a89856fe221b9c1debb03711a7c2d32ca6d79f741869b961b330da"
  [signals.module.js]="ebdebcbe59c9c72f0ba5c63683ef38d676f815ac295d62a3373e6c66cde0b7d6"
  [htm.module.js]="ab33dd3f38059b9be4d5f5350128eefb2356639c4e0bbe9d9e8b3ba75847e9e4"
)

# ─── Functions ───────────────────────────────────────────────────────────────

verify_checksum() {
  local file="$1" expected="$2"
  local actual
  actual=$(sha256sum "$file" | cut -d' ' -f1)
  if [ "$actual" != "$expected" ]; then
    echo "FAIL  $file"
    echo "  expected: $expected"
    echo "  actual:   $actual"
    return 1
  fi
  return 0
}

rewrite_imports() {
  local file="$1"
  case "$(basename "$file")" in
    hooks.module.js)
      sed -i 's|from"preact"|from"./preact.module.js"|g' "$file"
      ;;
    signals.module.js)
      sed -i \
        -e 's|from"preact"|from"./preact.module.js"|g' \
        -e 's|from"preact/hooks"|from"./hooks.module.js"|g' \
        -e 's|from"@preact/signals-core"|from"./signals-core.module.js"|g' \
        "$file"
      ;;
  esac
}

# ─── Verify mode ─────────────────────────────────────────────────────────────

if [ "${1:-}" = "--verify" ]; then
  echo "Verifying vendor checksums..."
  fail=0
  for name in "${!CHECKSUMS_AFTER_REWRITE[@]}"; do
    file="$VENDOR_DIR/$name"
    if [ ! -f "$file" ]; then
      echo "MISS  $file (not found)"
      fail=1
      continue
    fi
    if verify_checksum "$file" "${CHECKSUMS_AFTER_REWRITE[$name]}"; then
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

# ─── Download mode ───────────────────────────────────────────────────────────

TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

echo "Downloading vendor libraries..."
for name in "${!SOURCES[@]}"; do
  url="${SOURCES[$name]}"
  dest="$TMPDIR/$name"
  echo "  $name"
  curl -sL -o "$dest" "$url"

  # Verify upstream checksum
  if ! verify_checksum "$dest" "${CHECKSUMS_UPSTREAM[$name]}"; then
    echo "Upstream checksum mismatch for $name — aborting."
    exit 1
  fi
done

echo "Rewriting import specifiers..."
for name in "${!SOURCES[@]}"; do
  rewrite_imports "$TMPDIR/$name"
done

echo "Verifying final checksums..."
fail=0
for name in "${!CHECKSUMS_AFTER_REWRITE[@]}"; do
  if ! verify_checksum "$TMPDIR/$name" "${CHECKSUMS_AFTER_REWRITE[$name]}"; then
    fail=1
  fi
done
if [ "$fail" -ne 0 ]; then
  echo "Post-rewrite checksum mismatch — aborting."
  exit 1
fi

echo "Installing to $VENDOR_DIR/"
mkdir -p "$VENDOR_DIR"
for name in "${!SOURCES[@]}"; do
  cp "$TMPDIR/$name" "$VENDOR_DIR/$name"
done

echo "Done. Vendored files:"
ls -lh "$VENDOR_DIR"/*.js
