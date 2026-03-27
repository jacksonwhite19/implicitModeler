import argparse
from pathlib import Path
import numpy as np


def load_pts(path):
    pts = np.genfromtxt(path, delimiter=',', names=True)
    xs = pts[pts.dtype.names[0]]
    ys = pts[pts.dtype.names[1]]
    arr = np.column_stack((xs, ys))
    out = [arr[0]]
    for p in arr[1:]:
        if np.linalg.norm(p - out[-1]) > 1e-9:
            out.append(p)
    if np.linalg.norm(out[0] - out[-1]) < 1e-9:
        out.pop()
    return np.array(out)


def emit(name, pts):
    lines = [f"let {name} = custom_profile(["]
    for p in pts:
        lines.append(f"    [{p[0]:.6f}, {p[1]:.6f}],")
    lines.append("]);\n")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--outer', type=Path, required=True)
    ap.add_argument('--inner', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    outer = load_pts(args.outer)
    inner = load_pts(args.inner)
    text = emit('measured_outer_start', outer) + emit('measured_inner_start', inner)
    args.out.write_text(text)
    print(f'saved={args.out}')

if __name__ == '__main__':
    main()
