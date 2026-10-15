"""Generate CTF ordering fixtures and H5OINA ordering fixtures without patterns.

Run from the repository root:
    /opt/local/anaconda3/envs/nxpython/bin/python -I Data/EbsdTestFiles/make_out_of_order_test_files.py Data/EbsdTestFiles

The shuffle uses a fixed seed so the output is reproducible.
"""
import random
import sys
from pathlib import Path


def split_ctf(path: Path):
    lines = path.read_text(encoding="latin-1").splitlines(keepends=True)
    for i, line in enumerate(lines):
        if line.startswith("Phase\tX\tY"):
            return lines[: i + 1], lines[i + 1 :]
    raise RuntimeError(f"{path}: column header line not found")


import h5py
import numpy as np

SOURCE_H5OINA = Path("/Users/mjackson/Workspace1/DREAM3D_Data/H5Oina_Test_Data/H5Oina_Test_Data.h5oina")
PATTERN_NAMES = {"Processed Patterns", "Unprocessed Patterns"}


def copy_without_patterns(src: h5py.File, dst: h5py.File, permutation=None) -> None:
    """Copy scan metadata and EBSD measurements, omitting patterns and auxiliary images."""

    def visit(name, obj):
        if name.startswith(("1/Electron Image", "1/Layered Image")):
            return
        if name == "1/EBSD/Header/Processed Static Background":
            return
        if isinstance(obj, h5py.Group):
            g = dst.require_group(name)
            g.attrs.update(obj.attrs)
            return
        if obj.name.split("/")[-1] in PATTERN_NAMES:
            return
        data = obj[()]
        if permutation is not None and "/EBSD/Data/" in obj.name and obj.shape and obj.shape[0] == permutation.size:
            data = data[permutation]
        d = dst.create_dataset(name, data=data, dtype=obj.dtype)
        d.attrs.update(obj.attrs)

    dst.attrs.update(src.attrs)
    src.visititems(visit)


def make_h5oina(data_dir: Path) -> None:
    with h5py.File(SOURCE_H5OINA, "r") as src:
        n = src["1/EBSD/Data/Phase"].shape[0]
        with h5py.File(data_dir / "H5OINA_Ordered.h5oina", "w") as dst:
            copy_without_patterns(src, dst)
        perm = np.arange(n)
        np.random.default_rng(1804).shuffle(perm)
        with h5py.File(data_dir / "H5OINA_OutOfOrder.h5oina", "w") as dst:
            copy_without_patterns(src, dst, perm)


def main(data_dir: Path) -> None:
    header, rows = split_ctf(data_dir / "Test_US_2.ctf")
    rows = [r for r in rows if r.strip()]

    shuffled = list(rows)
    random.Random(1804).shuffle(shuffled)
    (data_dir / "Test_US_2_OutOfOrder.ctf").write_text("".join(header + shuffled), encoding="latin-1")

    duplicate = list(rows)
    duplicate[10] = duplicate[11]  # point 11 now appears twice, point 10 is missing
    (data_dir / "Test_US_2_DuplicatePoint.ctf").write_text("".join(header + duplicate), encoding="latin-1")

    make_h5oina(data_dir)


if __name__ == "__main__":
    main(Path(sys.argv[1]))
