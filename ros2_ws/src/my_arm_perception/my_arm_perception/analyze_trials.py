#!/usr/bin/env python3
"""Summarize a trial_runner CSV.  Usage: python3 analyze_trials.py trials.csv [--plot]"""
import csv
import statistics as st
import sys
from collections import Counter, defaultdict

SLOTS = ["top_left", "top_center", "top_right",
         "middle_left", "middle_center", "middle_right",
         "bottom_left", "bottom_center", "bottom_right"]


def pct(a, b):
    return f"{100 * a / b:5.1f}%  ({a}/{b})" if b else "  n/a"


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    rows = list(csv.DictReader(open(sys.argv[1])))
    n = len(rows)
    det = [r for r in rows if r["detected"] == "True"]
    ok = [r for r in rows if r["slot_correct"] == "True"]

    print(f"\n=== {n} trials ===")
    print(f"Detection rate        {pct(len(det), n)}")
    print(f"Slot accuracy (all)   {pct(len(ok), n)}")
    print(f"Slot accuracy | det.  {pct(len(ok), len(det))}")
    wrong_tray = [r for r in det if r["slot_correct"] != "True"]
    print(f"Wrong-tray rate       {pct(len(wrong_tray), n)}")

    lat = [float(r["latency_s"]) for r in det if r["latency_s"]]
    if lat:
        lat.sort()
        p95 = lat[min(len(lat) - 1, int(0.95 * len(lat)))]
        print(f"Latency (s)           mean {st.mean(lat):.2f}  median {st.median(lat):.2f}  p95 {p95:.2f}")

    print("\n--- Accuracy per requested supply ---")
    by_cat = defaultdict(list)
    for r in rows:
        by_cat[r["requested"]].append(r["slot_correct"] == "True")
    for cat, v in sorted(by_cat.items()):
        print(f"  {cat:14s} {pct(sum(v), len(v))}")

    print("\n--- Accuracy per true shelf slot (3x3, as seen from the table) ---")
    by_slot = defaultdict(list)
    for r in rows:
        by_slot[r["gt_slot"]].append(r["slot_correct"] == "True")
    for row in range(3):
        cells = []
        for col in range(3):
            v = by_slot[SLOTS[row * 3 + col]]
            cells.append(f"{100 * sum(v) / len(v):5.0f}% n={len(v):<3d}" if v else "   -       ")
        print("  " + " | ".join(cells))

    print("\n--- Confusions (requested -> tray actually found) ---")
    conf = Counter((r["requested"], r["found_category"]) for r in wrong_tray)
    if not conf:
        print("  none")
    for (a, b), c in conf.most_common():
        print(f"  {a:14s} -> {b:14s} x{c}")

    man = [r for r in rows if r["manip_status"] not in ("", "not_run")]
    if man:
        succ = [r for r in man if r["manip_status"] == "SUCCEEDED"]
        print("\n--- Manipulation ---")
        print(f"Success rate          {pct(len(succ), len(man))}")
        print(f"End-to-end success    {pct(sum(1 for r in succ if r['slot_correct'] == 'True'), n)}")
        times = [float(r["manip_time_s"]) for r in man if r["manip_time_s"]]
        if times:
            print(f"Manip time (s)        mean {st.mean(times):.2f}")
        rms = [float(r["joint_rms_err"]) for r in man if r["joint_rms_err"]]
        mx = [float(r["joint_max_err"]) for r in man if r["joint_max_err"]]
        if rms:
            print(f"Joint RMS err (rad)   mean {st.mean(rms):.4f}   worst max {max(mx):.4f}")
        print("Outcomes:", dict(Counter(r["manip_status"] for r in man)))

    if "--plot" in sys.argv:
        import matplotlib.pyplot as plt
        grid = [[(100 * sum(by_slot[SLOTS[r * 3 + c]]) / len(by_slot[SLOTS[r * 3 + c]]))
                 if by_slot[SLOTS[r * 3 + c]] else float("nan") for c in range(3)] for r in range(3)]
        fig, ax = plt.subplots()
        im = ax.imshow(grid, vmin=0, vmax=100, cmap="viridis")
        ax.set_xticks(range(3), ["left", "center", "right"])
        ax.set_yticks(range(3), ["top", "middle", "bottom"])
        for r in range(3):
            for c in range(3):
                ax.text(c, r, f"{grid[r][c]:.0f}%", ha="center", va="center", color="white")
        ax.set_title("Slot accuracy by shelf position")
        fig.colorbar(im)
        fig.savefig("slot_accuracy.png", dpi=150, bbox_inches="tight")
        print("\nSaved slot_accuracy.png")


if __name__ == "__main__":
    main()
