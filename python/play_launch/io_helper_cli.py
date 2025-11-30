"""CLI entry point for play_launch_io_helper that dispatches to Rust binary."""

import os
import shutil
import subprocess
import sys
from pathlib import Path


def _find_binary() -> str:
    """Find bundled play_launch_io_helper binary with fallback to PATH.

    Resolution order:
    1. Package bin directory (pip install layout)
    2. PATH search (fallback)
    """
    name = "play_launch_io_helper"

    # 1. Check package bin directory (pip install)
    pkg_dir = Path(__file__).parent
    bundled = pkg_dir / "bin" / name
    if bundled.exists() and os.access(bundled, os.X_OK):
        return str(bundled)

    # 2. Fall back to PATH
    path_binary = shutil.which(name)
    if path_binary:
        return path_binary

    raise FileNotFoundError(
        f"{name} not found. Ensure play_launch is properly installed.\n"
        f"Checked locations:\n"
        f"  - {bundled}\n"
        f"  - PATH"
    )


def main():
    """Main entry point - delegates to Rust binary."""
    # Handle --binary-path flag to print the actual binary location
    if len(sys.argv) == 2 and sys.argv[1] == "--binary-path":
        try:
            binary = _find_binary()
            print(binary)
            sys.exit(0)
        except FileNotFoundError as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)

    try:
        binary = _find_binary()
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    # Pass through all arguments to Rust binary
    result = subprocess.run([binary] + sys.argv[1:])
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
