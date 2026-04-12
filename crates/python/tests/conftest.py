"""Shared fixtures for play_launch_parser tests."""

import os
from pathlib import Path

import pytest

# Fixtures live in the parser crate's test directory
FIXTURES_DIR = (
    Path(__file__).resolve().parent.parent.parent
    / "play_launch_parser"
    / "tests"
    / "fixtures"
    / "launch"
)


@pytest.fixture
def fixtures_dir():
    """Path to the launch file fixtures directory."""
    assert FIXTURES_DIR.is_dir(), f"Fixtures dir not found: {FIXTURES_DIR}"
    return FIXTURES_DIR


@pytest.fixture
def fixture(fixtures_dir):
    """Return a fixture path by name."""

    def _fixture(name: str) -> str:
        path = fixtures_dir / name
        assert path.exists(), f"Fixture not found: {path}"
        return str(path)

    return _fixture
