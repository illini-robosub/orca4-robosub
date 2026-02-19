#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path

def convert(in_path: Path, out_path: Path, fmt: str | None = "collada"):
    out_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = ["assimp", "export", str(in_path), str(out_path)]
    if fmt:
        cmd += ["--format", fmt]  # e.g. "collada"

    subprocess.run(cmd, check=True)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("input_fbx", type=Path)
    p.add_argument("output_dae", type=Path)
    p.add_argument("--format", default="collada")
    args = p.parse_args()
    convert(args.input_fbx, args.output_dae, args.format)

if __name__ == "__main__":
    main()
