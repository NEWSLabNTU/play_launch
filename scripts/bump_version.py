#!/usr/bin/env python3
"""
Version bump script for play_launch.

Single source of truth: pyproject.toml
Updates all version locations consistently.
"""

import argparse
import re
import sys
from pathlib import Path

# Project root (relative to this script)
PROJECT_ROOT = Path(__file__).parent.parent

# Files containing version information (relative to project root)
VERSION_FILES = {
    "pyproject.toml": {
        "pattern": r'^version = "[^"]+"',
        "replacement": 'version = "{version}"',
    },
    "src/play_launch/Cargo.toml": {
        "pattern": r'^version = "[^"]+"',
        "replacement": 'version = "{version}"',
    },
    "src/play_launch/package.xml": {
        "pattern": r"<version>[^<]+</version>",
        "replacement": "<version>{version}</version>",
    },
    "python/play_launch/__init__.py": {
        "pattern": r'^__version__ = "[^"]+"',
        "replacement": '__version__ = "{version}"',
    },
}


def get_current_version() -> str:
    """Read current version from pyproject.toml (source of truth)."""
    pyproject = PROJECT_ROOT / "pyproject.toml"
    content = pyproject.read_text()
    match = re.search(r'^version = "([^"]+)"', content, re.MULTILINE)
    if not match:
        raise ValueError("Could not find version in pyproject.toml")
    return match.group(1)


def parse_version(version: str) -> tuple[int, int, int]:
    """Parse semver version string."""
    parts = version.split(".")
    if len(parts) != 3:
        raise ValueError(f"Invalid version format: {version} (expected X.Y.Z)")
    return int(parts[0]), int(parts[1]), int(parts[2])


def bump_version(current: str, bump_type: str) -> str:
    """Bump version by type (major, minor, patch)."""
    major, minor, patch = parse_version(current)
    if bump_type == "major":
        return f"{major + 1}.0.0"
    elif bump_type == "minor":
        return f"{major}.{minor + 1}.0"
    elif bump_type == "patch":
        return f"{major}.{minor}.{patch + 1}"
    else:
        raise ValueError(f"Invalid bump type: {bump_type}")


def update_file(filepath: Path, pattern: str, replacement: str, new_version: str, dry_run: bool) -> bool:
    """Update version in a single file. Returns True if changed."""
    if not filepath.exists():
        print(f"  SKIP: {filepath} (not found)")
        return False

    content = filepath.read_text()
    new_replacement = replacement.format(version=new_version)

    # Find current value
    match = re.search(pattern, content, re.MULTILINE)
    if not match:
        print(f"  SKIP: {filepath} (pattern not found)")
        return False

    old_value = match.group(0)
    if old_value == new_replacement:
        print(f"  OK:   {filepath.relative_to(PROJECT_ROOT)} (already {new_version})")
        return False

    new_content = re.sub(pattern, new_replacement, content, count=1, flags=re.MULTILINE)

    if dry_run:
        print(f"  WILL: {filepath.relative_to(PROJECT_ROOT)}: {old_value} -> {new_replacement}")
    else:
        filepath.write_text(new_content)
        print(f"  DONE: {filepath.relative_to(PROJECT_ROOT)}: {old_value} -> {new_replacement}")

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Bump version across all project files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s patch          # 0.3.2 -> 0.3.3
  %(prog)s minor          # 0.3.2 -> 0.4.0
  %(prog)s major          # 0.3.2 -> 1.0.0
  %(prog)s --set 1.0.0    # Set explicit version
  %(prog)s --check        # Show current versions in all files
""",
    )
    parser.add_argument(
        "bump_type",
        nargs="?",
        choices=["major", "minor", "patch"],
        help="Version component to bump",
    )
    parser.add_argument(
        "--set",
        metavar="VERSION",
        help="Set explicit version (X.Y.Z format)",
    )
    parser.add_argument(
        "--dry-run", "-n",
        action="store_true",
        help="Show what would be changed without modifying files",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Show current version in all files",
    )

    args = parser.parse_args()

    current_version = get_current_version()

    if args.check:
        print(f"Source of truth (pyproject.toml): {current_version}\n")
        print("All version locations:")
        for rel_path, config in VERSION_FILES.items():
            filepath = PROJECT_ROOT / rel_path
            if not filepath.exists():
                print(f"  {rel_path}: NOT FOUND")
                continue
            content = filepath.read_text()
            match = re.search(config["pattern"], content, re.MULTILINE)
            if match:
                # Extract just the version number
                full_match = match.group(0)
                ver_match = re.search(r'[\d]+\.[\d]+\.[\d]+', full_match)
                ver = ver_match.group(0) if ver_match else "?"
                status = "✓" if ver == current_version else f"✗ (out of sync)"
                print(f"  {rel_path}: {ver} {status}")
            else:
                print(f"  {rel_path}: pattern not found")
        return 0

    if args.set:
        # Validate format
        parse_version(args.set)
        new_version = args.set
    elif args.bump_type:
        new_version = bump_version(current_version, args.bump_type)
    else:
        parser.print_help()
        return 1

    print(f"Version: {current_version} -> {new_version}")
    if args.dry_run:
        print("\nDry run - no files will be modified:\n")
    else:
        print()

    changed = 0
    for rel_path, config in VERSION_FILES.items():
        filepath = PROJECT_ROOT / rel_path
        if update_file(filepath, config["pattern"], config["replacement"], new_version, args.dry_run):
            changed += 1

    print()
    if args.dry_run:
        print(f"Would update {changed} file(s)")
    else:
        print(f"Updated {changed} file(s)")
        if changed > 0:
            print("\nNext steps:")
            print(f"  git add -u && git commit -m 'Bump to v{new_version}'")
            print(f"  git tag v{new_version}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
