#!/usr/bin/env python3
"""Build script for pyurrr wheel package.

This script handles:
- Version bumping (optional)
- Building the wheel

Usage:
    # Build current version
    python build_wheel.py

    # Bump version and build
    python build_wheel.py --version 0.2.0

    # Bump patch version (0.1.0 -> 0.1.1)
    python build_wheel.py --bump patch

    # Bump minor version (0.1.0 -> 0.2.0)
    python build_wheel.py --bump minor

    # Bump major version (0.1.0 -> 1.0.0)
    python build_wheel.py --bump major

    # Build without cleaning
    python build_wheel.py --no-clean
"""

import re
import subprocess
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional

import tyro


class VersionPart(str, Enum):
    """Version part to bump."""

    MAJOR = "major"
    MINOR = "minor"
    PATCH = "patch"


@dataclass
class BuildConfig:
    """Configuration for building pyurrr wheel package.

    Attributes:
        version: Set specific version (e.g., "0.2.0"). If None, uses current version.
        bump: Auto-bump version by part (major/minor/patch). Overrides --version.
        no_clean: Skip cleaning build directories before building.
    """

    version: Optional[str] = None
    bump: Optional[VersionPart] = None
    no_clean: bool = False


def get_current_version() -> str:
    """Get current version from pyproject.toml."""
    pyproject_path = Path(__file__).parent / "pyproject.toml"
    content = pyproject_path.read_text()

    match = re.search(r'version\s*=\s*"([^"]+)"', content)
    if not match:
        raise ValueError("Could not find version in pyproject.toml")

    return match.group(1)


def bump_version(version: str, part: VersionPart) -> str:
    """Bump version by specified part.

    Args:
        version: Current version string (e.g., "0.1.0")
        part: Which part to bump (major, minor, or patch)

    Returns:
        New version string
    """
    major, minor, patch = map(int, version.split("."))

    if part == VersionPart.MAJOR:
        major += 1
        minor = 0
        patch = 0
    elif part == VersionPart.MINOR:
        minor += 1
        patch = 0
    elif part == VersionPart.PATCH:
        patch += 1

    return f"{major}.{minor}.{patch}"


def update_version(new_version: str) -> None:
    """Update version in both pyproject.toml and __init__.py.

    Args:
        new_version: New version string to set
    """
    project_root = Path(__file__).parent

    # Update pyproject.toml
    pyproject_path = project_root / "pyproject.toml"
    content = pyproject_path.read_text()
    content = re.sub(
        r'version\s*=\s*"[^"]+"',
        f'version = "{new_version}"',
        content,
    )
    pyproject_path.write_text(content)
    print(f"✓ Updated pyproject.toml -> {new_version}")

    # Update __init__.py
    init_path = project_root / "src" / "pyurrr" / "__init__.py"
    content = init_path.read_text()
    content = re.sub(
        r'__version__\s*=\s*"[^"]+"',
        f'__version__ = "{new_version}"',
        content,
    )
    init_path.write_text(content)
    print(f"✓ Updated src/pyurrr/__init__.py -> {new_version}")


def clean_build_dirs() -> None:
    """Clean build and dist directories."""
    project_root = Path(__file__).parent

    for dir_name in ["build", "dist"]:
        dir_path = project_root / dir_name
        if dir_path.exists():
            subprocess.run(["rm", "-rf", str(dir_path)], check=True)
            print(f"✓ Cleaned {dir_name}/")


def build_wheel() -> Path:
    """Build the wheel package.

    Returns:
        Path to the built wheel file
    """
    print("\nBuilding wheel package...")

    # Build the package using uv
    result = subprocess.run(
        ["uv", "build"],
        check=True,
        capture_output=True,
        text=True,
    )

    print(result.stdout)

    # Find the built wheel
    dist_dir = Path(__file__).parent / "dist"
    wheels = list(dist_dir.glob("*.whl"))

    if not wheels:
        raise FileNotFoundError("No wheel file found in dist/")

    wheel_path = wheels[0]
    print(f"\n✓ Built wheel: {wheel_path.name}")

    return wheel_path


def main(config: BuildConfig) -> None:
    """Main entry point for building the wheel.

    Args:
        config: Build configuration from CLI args
    """
    # Get current version
    current_version = get_current_version()
    print(f"Current version: {current_version}")

    # Determine new version
    if config.bump:
        new_version = bump_version(current_version, config.bump)
        print(
            f"Bumping {config.bump.value} version: {current_version} -> {new_version}"
        )
    elif config.version:
        new_version = config.version
        print(f"Setting version to: {new_version}")
    else:
        new_version = None

    # Update version if needed
    if new_version:
        update_version(new_version)

    # Clean build directories
    if not config.no_clean:
        clean_build_dirs()

    # Build wheel
    try:
        wheel_path = build_wheel()

        print("\n" + "=" * 60)
        print("Build complete!")
        if new_version:
            print(f"Version: {new_version}")
        print(f"Wheel: {wheel_path}")
        print(f"Size: {wheel_path.stat().st_size / 1024:.1f} KB")
        print("=" * 60)

        print("\nTo install this wheel, run:")
        print(f"  pip install {wheel_path.name}")

    except subprocess.CalledProcessError as e:
        print(f"\n✗ Error during build: {e}", file=sys.stderr)
        if e.stdout:
            print(f"STDOUT:\n{e.stdout}", file=sys.stderr)
        if e.stderr:
            print(f"STDERR:\n{e.stderr}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main(tyro.cli(BuildConfig))
