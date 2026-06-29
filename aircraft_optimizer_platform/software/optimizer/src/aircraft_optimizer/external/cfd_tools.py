from __future__ import annotations

import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class CfdToolStatus:
    available: bool
    su2: str | None
    gmsh: str | None
    meshio: str | None
    raw_output: str
    error: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "available": self.available,
            "su2": self.su2,
            "gmsh": self.gmsh,
            "meshio": self.meshio,
            "raw_output": self.raw_output,
            "error": self.error,
        }


def check_wsl_cfd_tools(timeout_seconds: int = 30) -> CfdToolStatus:
    command = [
        "wsl",
        "bash",
        "-lc",
        "export MAMBA_ROOT_PREFIX=~/.local/aircraft_optimizer_platform/mamba_root; "
        "~/.local/aircraft_optimizer_platform/micromamba/bin/micromamba run -n aop-cfd SU2_CFD --help | head -1; "
        "~/.local/aircraft_optimizer_platform/micromamba/bin/micromamba run -n aop-cfd gmsh -version; "
        "~/.local/aircraft_optimizer_platform/micromamba/bin/micromamba run -n aop-cfd meshio --version | head -1",
    ]
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return CfdToolStatus(
            available=False,
            su2=None,
            gmsh=None,
            meshio=None,
            raw_output="",
            error=str(exc),
        )

    output = completed.stdout.strip()
    parsed = parse_cfd_tool_versions(output)
    if completed.returncode != 0:
        return CfdToolStatus(
            available=False,
            su2=parsed.su2,
            gmsh=parsed.gmsh,
            meshio=parsed.meshio,
            raw_output=output,
            error=completed.stderr.strip() or f"command returned {completed.returncode}",
        )
    return parsed


def parse_cfd_tool_versions(output: str) -> CfdToolStatus:
    lines = [line.strip() for line in output.splitlines() if line.strip()]
    su2 = lines[0] if len(lines) >= 1 and lines[0].startswith("SU2 ") else None
    gmsh = lines[1] if len(lines) >= 2 else None
    meshio = lines[2] if len(lines) >= 3 and lines[2].startswith("meshio ") else None
    return CfdToolStatus(
        available=bool(su2 and gmsh and meshio),
        su2=su2,
        gmsh=gmsh,
        meshio=meshio,
        raw_output=output,
    )
