#!/usr/bin/env bash
# Compile typst diagram sources to PNG and SVG.
# Usage: ./compile-images.sh
# Requires: typst (cargo install typst-cli)

set -euo pipefail
cd "$(dirname "$0")/img"

for src in *.typ; do
    name="${src%.typ}"
    echo "Compiling $src ..."
    typst compile "$src" "${name}.png" --format png --ppi 200
    typst compile "$src" "${name}.svg" --format svg
done

echo "Done. Generated:"
ls -1 *.png *.svg 2>/dev/null
