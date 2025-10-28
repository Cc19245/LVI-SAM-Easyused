#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Tkinter GUI wrapper for evaluate_with_evo.py:
- Select KITTI raw dir, estimated/GT bag, topics, and output dir
- One-click to export TUM and run evo APE/RPE

Notes:
- Reading bag requires ROS (noetic) Python environment (`import rosbag`).
- GUI requires a graphical environment; on headless servers/containers use the CLI instead.
"""

import os
import sys
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# Ensure we can import evaluate_with_evo.py in the same directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    from evaluate_with_evo import (
        kitti_oxts_to_tum,
        bag_to_tum,
        run_evo_ape,
        run_evo_rpe,
        ensure_dir,
        write_tum,
        normalize_timebases,
    )
except Exception as e:
    # Defer import error handling to runtime
    pass

import shutil


class EvoGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("evo Evaluation GUI (KITTI/ROS bag)")
        self.geometry("780x560")
        self.resizable(True, True)

        # Variables
        self.kitti_dir = tk.StringVar()
        self.bag_est = tk.StringVar()
        self.est_topic = tk.StringVar(value="/lvi_sam/lidar/mapping/odometry")
        self.bag_gt = tk.StringVar()
        self.gt_topic = tk.StringVar()
        self.out_dir = tk.StringVar(value=os.path.join(SCRIPT_DIR, "..", "bags", "evals"))
        self.eval_type = tk.StringVar(value="both")
        self.align_type = tk.StringVar(value="se3")
        self.save_table = tk.BooleanVar(value=True)

        self._build_ui()

    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}

        frm = ttk.Frame(self)
        frm.pack(fill=tk.BOTH, expand=True)

        # Row: KITTI raw directory
        row = 0
        ttk.Label(frm, text="KITTI raw directory (optional)").grid(row=row, column=0, sticky=tk.W, **pad)
        ttk.Entry(frm, textvariable=self.kitti_dir, width=70).grid(row=row, column=1, sticky=tk.EW, **pad)
        ttk.Button(frm, text="Browse", command=self._choose_kitti_dir).grid(row=row, column=2, **pad)

        # Row: estimated bag
        row += 1
        ttk.Label(frm, text="Estimated bag path").grid(row=row, column=0, sticky=tk.W, **pad)
        ttk.Entry(frm, textvariable=self.bag_est, width=70).grid(row=row, column=1, sticky=tk.EW, **pad)
        ttk.Button(frm, text="Browse", command=self._choose_bag_est).grid(row=row, column=2, **pad)

        # Row: estimated topic
        row += 1
        ttk.Label(frm, text="Estimated topic").grid(row=row, column=0, sticky=tk.W, **pad)
        ttk.Entry(frm, textvariable=self.est_topic, width=70).grid(row=row, column=1, sticky=tk.EW, **pad)
        ttk.Button(frm, text="List bag topics", command=self._list_topics_est).grid(row=row, column=2, **pad)

        # Row: ground-truth bag (optional)
        row += 1
        ttk.Label(frm, text="Ground-truth bag path (optional)").grid(row=row, column=0, sticky=tk.W, **pad)
        ttk.Entry(frm, textvariable=self.bag_gt, width=70).grid(row=row, column=1, sticky=tk.EW, **pad)
        ttk.Button(frm, text="Browse", command=self._choose_bag_gt).grid(row=row, column=2, **pad)

        # Row: ground-truth topic (optional)
        row += 1
        ttk.Label(frm, text="Ground-truth topic (optional)").grid(row=row, column=0, sticky=tk.W, **pad)
        ttk.Entry(frm, textvariable=self.gt_topic, width=70).grid(row=row, column=1, sticky=tk.EW, **pad)
        ttk.Button(frm, text="List bag topics", command=self._list_topics_gt).grid(row=row, column=2, **pad)

        # Row: output directory
        row += 1
        ttk.Label(frm, text="Output directory").grid(row=row, column=0, sticky=tk.W, **pad)
        ttk.Entry(frm, textvariable=self.out_dir, width=70).grid(row=row, column=1, sticky=tk.EW, **pad)
        ttk.Button(frm, text="Browse", command=self._choose_out_dir).grid(row=row, column=2, **pad)

        # Row: evaluation options
        row += 1
        ttk.Label(frm, text="Evaluation type").grid(row=row, column=0, sticky=tk.W, **pad)
        ttk.OptionMenu(frm, self.eval_type, self.eval_type.get(), "ape", "rpe", "both").grid(row=row, column=1, sticky=tk.W, **pad)
        ttk.Label(frm, text="Alignment").grid(row=row, column=2, sticky=tk.W, **pad)
        ttk.OptionMenu(frm, self.align_type, self.align_type.get(), "none", "scale", "se3").grid(row=row, column=3, sticky=tk.W, **pad)
        row += 1
        ttk.Checkbutton(frm, text="Save CSV tables", variable=self.save_table).grid(row=row, column=0, sticky=tk.W, **pad)

        # Buttons row
        row += 1
        ttk.Button(frm, text="Generate TUM only", command=self._run_generate_only).grid(row=row, column=0, **pad)
        ttk.Button(frm, text="Run evaluation", command=self._run_all).grid(row=row, column=1, **pad)
        ttk.Button(frm, text="Run APE+RPE (CSV)", command=self._run_both_csv).grid(row=row, column=2, **pad)
        ttk.Button(frm, text="Open output dir", command=self._open_out_dir).grid(row=row, column=3, **pad)
        ttk.Button(frm, text="Quit", command=self.destroy).grid(row=row, column=4, **pad)

        # Log box
        row += 1
        ttk.Label(frm, text="Logs").grid(row=row, column=0, sticky=tk.W, **pad)
        self.txt = tk.Text(frm, height=18)
        self.txt.grid(row=row, column=0, columnspan=4, sticky=tk.NSEW, padx=6, pady=4)

        # Layout weights
        for c in range(0, 4):
            frm.grid_columnconfigure(c, weight=1)
        frm.grid_rowconfigure(row, weight=1)

    # Pickers
    def _choose_kitti_dir(self):
        d = filedialog.askdirectory(title="Select KITTI *_sync directory")
        if d:
            self.kitti_dir.set(d)

    def _choose_bag_est(self):
        f = filedialog.askopenfilename(title="Select estimated bag", filetypes=[["ROS bag", "*.bag"], ["All", "*.*"]])
        if f:
            self.bag_est.set(f)

    def _choose_bag_gt(self):
        f = filedialog.askopenfilename(title="Select ground-truth bag", filetypes=[["ROS bag", "*.bag"], ["All", "*.*"]])
        if f:
            self.bag_gt.set(f)

    def _choose_out_dir(self):
        d = filedialog.askdirectory(title="Select output directory")
        if d:
            self.out_dir.set(d)

    # Logging helpers
    def log(self, s: str):
        self.txt.insert(tk.END, s + "\n")
        self.txt.see(tk.END)
        self.update_idletasks()

    def _list_topics(self, bag_path: str):
        if not bag_path or not os.path.isfile(bag_path):
            messagebox.showwarning("Tip", "Please choose a valid bag file")
            return
        try:
            import rosbag
        except Exception as e:
            messagebox.showerror("Error", "Failed to import rosbag. Source your ROS environment or install ROS Python deps.")
            return
        try:
            from collections import Counter
            types = {}
            with rosbag.Bag(bag_path, 'r') as bag:
                for conn in bag._get_connections():
                    types[conn.topic] = conn.datatype
            self.log("Bag topics:")
            for t, ty in sorted(types.items()):
                self.log(f"  {t} : {ty}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read bag: {e}")

    def _list_topics_est(self):
        self._list_topics(self.bag_est.get())

    def _list_topics_gt(self):
        self._list_topics(self.bag_gt.get())

    # 执行逻辑
    def _run_thread(self, do_eval: bool, force_eval_type: str = None, force_save_table: bool = None):
        try:
            out_dir = self.out_dir.get().strip()
            ensure_dir(out_dir)
            gt_tum = os.path.join(out_dir, 'gt.tum')
            est_tum = os.path.join(out_dir, 'est.tum')

            # Ground truth source
            if self.bag_gt.get().strip() and self.gt_topic.get().strip():
                self.log(f"Extract GT from bag: {self.bag_gt.get()} [{self.gt_topic.get()}]")
                gt = bag_to_tum(self.bag_gt.get().strip(), self.gt_topic.get().strip())
                self.log(f"GT frames: {len(gt)}")
            elif self.kitti_dir.get().strip():
                self.log(f"Generate GT from KITTI raw: {self.kitti_dir.get()}")
                gt = kitti_oxts_to_tum(self.kitti_dir.get().strip())
                self.log(f"GT frames: {len(gt)}")
            else:
                messagebox.showerror("Error", "Please provide GT source: KITTI dir or (bag + topic)")
                return

            # Estimated trajectory
            if not self.bag_est.get().strip():
                messagebox.showerror("Error", "Please provide estimated bag path")
                return
            self.log(f"Extract estimated trajectory from bag: {self.bag_est.get()} [{self.est_topic.get()}]")
            est = bag_to_tum(self.bag_est.get().strip(), self.est_topic.get().strip())
            self.log(f"EST frames: {len(est)}")

            gt, est, gt_offset, est_offset = normalize_timebases(gt, est)
            if abs(gt_offset) > 1e-6 or abs(est_offset) > 1e-6:
                self.log(
                    f"Applied timestamp offsets: gt {gt_offset:+.6f} s, est {est_offset:+.6f} s"
                )

            write_tum(gt_tum, gt)
            write_tum(est_tum, est)
            self.log(f"Saved TUM files to: {gt_tum} and {est_tum}")

            if not do_eval:
                self.log("TUM export done. Skipping evo evaluation.")
                return

            # evo check
            have_ape = shutil.which('evo_ape') is not None
            have_rpe = shutil.which('evo_rpe') is not None
            if not (have_ape and have_rpe):
                self.log("evo not detected. Install via `pip3 install evo`. Skipping evaluation.")
                return

            # evaluation
            et = force_eval_type if force_eval_type else self.eval_type.get()
            align = self.align_type.get()
            save_table = self.save_table.get() if force_save_table is None else force_save_table
            if et in ("ape", "both"):
                self.log("[evo_ape] start...")
                run_evo_ape(gt_tum, est_tum, out_dir, align=align, save_table=save_table)
                self.log("[evo_ape] done.")
            if et in ("rpe", "both"):
                self.log("[evo_rpe] start...")
                run_evo_rpe(gt_tum, est_tum, out_dir, align=align, save_table=save_table)
                self.log("[evo_rpe] done.")

            self.log("All done.")
            messagebox.showinfo("Done", f"Output dir: {out_dir}")

        except Exception as e:
            messagebox.showerror("Error", str(e))
            self.log(f"Error: {e}")

    def _run_all(self):
        threading.Thread(target=self._run_thread, args=(True,), daemon=True).start()

    def _run_generate_only(self):
        threading.Thread(target=self._run_thread, args=(False,), daemon=True).start()

    def _run_both_csv(self):
        threading.Thread(target=self._run_thread, args=(True, 'both', True), daemon=True).start()

    def _open_out_dir(self):
        d = self.out_dir.get().strip()
        if not d:
            messagebox.showwarning("Tip", "Please set the output directory first")
            return
        os.makedirs(d, exist_ok=True)
        # Try to open in host browser or file manager
        try:
            if sys.platform.startswith('linux'):
                # 在 devcontainer 中可尝试 xdg-open；若失败请手动打开
                os.system(f'xdg-open "{d}" >/dev/null 2>&1 &')
            elif sys.platform == 'darwin':
                os.system(f'open "{d}"')
            elif os.name == 'nt':
                os.startfile(d)  # type: ignore
        except Exception:
            pass


def main():
    app = EvoGUI()
    app.mainloop()


if __name__ == '__main__':
    main()
