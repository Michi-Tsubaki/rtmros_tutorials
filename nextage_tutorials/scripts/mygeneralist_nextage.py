#!/usr/bin/env python3
"""Run MyGeneralist with its uv environment while preserving the sourced ROS graph."""

from __future__ import annotations

import os
from pathlib import Path
import sys


def main() -> None:
    root = Path(os.environ.get("MYGENERALIST_ROOT", Path.home() / "mygeneralist")).expanduser()
    interpreter = root / ".venv" / "bin" / "python"
    if not interpreter.is_file():
        raise SystemExit(
            f"MyGeneralist environment not found at {interpreter}. "
            f"Run `cd {root} && uv sync --dev`, or set MYGENERALIST_ROOT."
        )
    environment = os.environ.copy()
    source = str(root / "src")
    environment["PYTHONPATH"] = source + os.pathsep + environment.get("PYTHONPATH", "")
    os.execve(
        str(interpreter),
        [str(interpreter), "-m", "mygeneralist.ros.nextage", *sys.argv[1:]],
        environment,
    )


if __name__ == "__main__":
    main()
