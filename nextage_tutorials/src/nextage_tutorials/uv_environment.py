from __future__ import annotations

import os
import site
import sys
from pathlib import Path


PACKAGE_NAME = "nextage_tutorials"
PROJECT_DIR_ENV = "NEXTAGE_TUTORIALS_PROJECT_DIR"


def candidate_project_roots() -> list[Path]:
    roots: list[Path] = []

    configured_root = os.environ.get(PROJECT_DIR_ENV)
    if configured_root:
        roots.append(Path(configured_root).expanduser())

    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for parent in (start, *start.parents):
            if parent.name == PACKAGE_NAME and (parent / "pyproject.toml").is_file():
                roots.append(parent)

    workspaces = set()
    for env_name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        for entry in os.environ.get(env_name, "").split(os.pathsep):
            if not entry:
                continue
            prefix = Path(entry).expanduser()
            if prefix.parent.name == "install":
                workspaces.add(prefix.parent.parent)
            elif prefix.name == "install":
                workspaces.add(prefix.parent)
            elif prefix.name == PACKAGE_NAME and prefix.parent.parent.name == "install":
                workspaces.add(prefix.parent.parent.parent)

    for workspace in workspaces:
        src_dir = workspace / "src"
        if not src_dir.is_dir():
            continue
        direct = src_dir / PACKAGE_NAME
        if (direct / "pyproject.toml").is_file():
            roots.append(direct)
        for candidate in src_dir.glob(f"**/{PACKAGE_NAME}"):
            if (candidate / "pyproject.toml").is_file():
                roots.append(candidate)

    roots.append(Path.home() / "colcon_ws" / "src" / "tork-a" / "rtmros_tutorials" / PACKAGE_NAME)
    roots.append(Path.home() / "colcon_ws" / "src" / PACKAGE_NAME)
    return list(dict.fromkeys(root.resolve() for root in roots))


def add_uv_site_packages() -> None:
    roots = candidate_project_roots()
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        root / ".venv" / "lib" / version / "site-packages"
        for root in roots
    ]

    virtual_env = os.environ.get("VIRTUAL_ENV")
    if virtual_env:
        candidates.append(Path(virtual_env) / "lib" / version / "site-packages")

    for site_packages in reversed(candidates):
        if site_packages.is_dir():
            site_path = str(site_packages)
            site.addsitedir(site_path)
            sys.path[:] = [path for path in sys.path if path != site_path]
            sys.path.insert(0, site_path)

    source_package_candidates = [root / "src" / PACKAGE_NAME for root in roots]
    for package_dir in reversed(source_package_candidates):
        if (package_dir / "__init__.py").is_file():
            source_parent = str(package_dir.parent)
            sys.path[:] = [path for path in sys.path if path != source_parent]
            sys.path.insert(0, source_parent)

            package = sys.modules.get(PACKAGE_NAME)
            if package is not None and hasattr(package, "__path__"):
                package_path = str(package_dir)
                package.__path__[:] = [
                    path for path in package.__path__ if path != package_path
                ]
                package.__path__.insert(0, package_path)

    source_candidates = [root / "src" / "scikit-robot" for root in roots]
    for source_dir in reversed(source_candidates):
        if (source_dir / "skrobot" / "__init__.py").is_file():
            source_path = str(source_dir)
            sys.path[:] = [path for path in sys.path if path != source_path]
            sys.path.insert(0, source_path)
