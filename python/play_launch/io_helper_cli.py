"""CLI entry point for play_launch_io_helper that dispatches to Rust binary."""

import subprocess
import sys

from play_launch import cli, payload


def _find_binary() -> str:
    """Find the play_launch_io_helper binary: the wheel's payload, then PATH."""
    return cli._find_binary("play_launch_io_helper")


def main():
    """Main entry point - delegates to Rust binary."""
    # Handle --binary-path flag to print the actual binary location
    if len(sys.argv) == 2 and sys.argv[1] == "--binary-path":
        try:
            binary = _find_binary()
            print(binary)
            sys.exit(0)
        except (FileNotFoundError, payload.PayloadError) as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)

    try:
        binary = _find_binary()
    except (FileNotFoundError, payload.PayloadError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    env = cli.payload_env(binary)

    # Pass through all arguments to Rust binary
    result = subprocess.run([binary] + sys.argv[1:], env=env)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
