This directory stores offline wheels for Codex cloud sandboxes.

Current target:
- Python 3.12.12
- Linux x86_64

Included wheel:
- `h5py-3.16.0-cp312-cp312-manylinux_2_28_x86_64.whl`

Install from the repo root with:

```bash
bash scripts/install_h5py_offline.sh
```

If pip reports a missing dependency, add the matching dependency wheel into this
directory and rerun the same script.
