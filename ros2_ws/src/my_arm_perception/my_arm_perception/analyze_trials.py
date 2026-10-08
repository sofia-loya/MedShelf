#!/usr/bin/env python3
"""Summarize a trial_runner CSV.  Usage: python3 analyze_trials.py trials.csv [--plot]"""
import csv
import re
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

    man = [r for r in rows if r.get("manip_status", "") not in ("", "not_run")]
    if man:
        succ = [r for r in man if r["manip_status"] == "SUCCEEDED"]
        print("\n--- Manipulation ---")
        print(f"Node reported success {pct(len(succ), len(man))}")
        if "delivered_correct" in man[0]:
            dc = [r for r in man if r["delivered_correct"] == "True"]
            e2e = [r for r in man if r["end_to_end"] == "True"]
            print(f"Right tray delivered  {pct(len(dc), len(man))}   (Gazebo ground truth)")
            print(f"End-to-end success    {pct(len(e2e), n)}   (right cubby AND right tray delivered)")
            wrong = [r for r in man if r["delivered_bin"] not in ("", "none", r["requested"])]
            print(f"Wrong tray delivered  {pct(len(wrong), len(man))}")
        times = [float(r["manip_time_s"]) for r in succ if r["manip_time_s"]]
        if times:
            print(f"Pick time (s)         mean {st.mean(times):.1f}  min {min(times):.1f}  max {max(times):.1f}")
        ge = [float(r["grasp_err_cm"]) for r in man if r.get("grasp_err_cm")]
        if ge:
            print(f"Grasp error (cm)      mean {st.mean(ge):.2f}  max {max(ge):.2f}   (gripper vs real handle)")
        rms = [float(r["joint_rms_err"]) for r in man if r["joint_rms_err"]]
        mx = [float(r["joint_max_err"]) for r in man if r["joint_max_err"]]
        if rms:
            print(f"Joint RMS err (rad)   mean {st.mean(rms):.4f}   worst max {max(mx):.4f}")

        print("\n  Manipulation success per cubby (3x3):")
        by_m = defaultdict(list)
        for r in man:
            ok_ = r.get("delivered_correct", r["manip_status"] == "SUCCEEDED")
            by_m[r["det_slot"]].append(ok_ in (True, "True"))
        for row in range(3):
            cells = []
            for col in range(3):
                v = by_m[SLOTS[row * 3 + col]]
                cells.append(f"{100 * sum(v) / len(v):5.0f}% n={len(v):<3d}" if v else "   -       ")
            print("  " + " | ".join(cells))

        fails = Counter()
        for r in man:
            if r["manip_status"] != "SUCCEEDED":
                d = r.get("manip_detail", "") or r["manip_status"]
                fails[re.sub(r"\(.*?\)|[\d.]+ ?cm", "", d).strip()[:70]] += 1
        if fails:
            print("\n  Failure reasons:")
            for reason, c in fails.most_common():
                print(f"    x{c:<3d} {reason}")

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
