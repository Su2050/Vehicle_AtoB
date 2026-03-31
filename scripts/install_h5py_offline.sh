#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_DIR="$ROOT_DIR/packages"

if ! ls "$PACKAGE_DIR"/h5py-*.whl >/dev/null 2>&1; then
  echo "No h5py wheel found in $PACKAGE_DIR" >&2
  exit 1
fi

python3 -m pip install --no-index --find-links "$PACKAGE_DIR" h5py
python3 -c 'import h5py; print(f"h5py {h5py.__version__} installed")'
