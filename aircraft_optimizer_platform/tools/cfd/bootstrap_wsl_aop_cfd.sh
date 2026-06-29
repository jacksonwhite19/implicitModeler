#!/usr/bin/env bash
set -euo pipefail

MICROMAMBA_HOME="${HOME}/.local/aircraft_optimizer_platform/micromamba"
MAMBA_ROOT_PREFIX="${HOME}/.local/aircraft_optimizer_platform/mamba_root"
ENV_NAME="aop-cfd"

mkdir -p "${MICROMAMBA_HOME}"
cd "${MICROMAMBA_HOME}"

if [ ! -x "${MICROMAMBA_HOME}/bin/micromamba" ]; then
  curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest -o micromamba.tar.bz2
  tar -xjf micromamba.tar.bz2
fi

export MAMBA_ROOT_PREFIX
"${MICROMAMBA_HOME}/bin/micromamba" create -y \
  -n "${ENV_NAME}" \
  -c conda-forge \
  python=3.11 \
  su2=8.5.0 \
  gmsh \
  meshio

"${MICROMAMBA_HOME}/bin/micromamba" run -n "${ENV_NAME}" SU2_CFD --help >/dev/null
"${MICROMAMBA_HOME}/bin/micromamba" run -n "${ENV_NAME}" gmsh -version
"${MICROMAMBA_HOME}/bin/micromamba" run -n "${ENV_NAME}" meshio --version
