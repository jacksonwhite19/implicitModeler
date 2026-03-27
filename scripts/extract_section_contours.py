import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_grid(path: Path):
    data = np.genfromtxt(path, delimiter=',', names=True)
    a_name, b_name = data.dtype.names[:2]
    xs = np.unique(data[a_name])
    zs = np.unique(data[b_name])
    d = data['d'].reshape((len(zs), len(xs)))
    return a_name, b_name, xs, zs, d


def signed_area(poly):
    x = poly[:, 0]
    y = poly[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', type=Path)
    ap.add_argument('--out-prefix', type=Path, required=True)
    ap.add_argument('--center-y', type=float, default=0.0)
    ap.add_argument('--center-z', type=float, default=0.0)
    args = ap.parse_args()

    a_name, b_name, xs, zs, d = load_grid(args.csv)
    X, Z = np.meshgrid(xs, zs)
    fig, ax = plt.subplots()
    cs = ax.contour(X, Z, d, levels=[0.0])
    segs = cs.allsegs[0]
    plt.close(fig)
    segs = [np.asarray(seg) for seg in segs if len(seg) >= 8]
    if not segs:
        raise SystemExit('no contours')

    segs.sort(key=lambda s: abs(signed_area(s)), reverse=True)
    for idx, seg in enumerate(segs[:4]):
        np.savetxt(args.out_prefix.with_name(f"{args.out_prefix.name}_{idx}.csv"), seg, delimiter=',', header='y,z', comments='')
        centered = np.column_stack((seg[:,0] - args.center_y, seg[:,1] - args.center_z))
        np.savetxt(args.out_prefix.with_name(f"{args.out_prefix.name}_{idx}_centered.csv"), centered, delimiter=',', header='y,z_local', comments='')
        print(f"contour_{idx}_points={len(seg)} area={signed_area(seg)}")

if __name__ == '__main__':
    main()
