#!/usr/bin/env python3
"""
Navigation Benchmark Analyzer – tkinter GUI (v3).

Reads JSON benchmark reports from ~/ros2_ws/reports/, displays them in
a sortable, filterable table, computes per-algorithm summaries, and
offers statistical tests plus matplotlib plots.

New in v3:
- Additional derived metrics (straight-line distance, path optimality ratio).
- Richer statistics: descriptive block (min/max/median/Q1/Q3/CV/95% CI),
  success-rate section, rank-biserial effect size for Mann-Whitney U.
- Help → Metrics & Tests Legend window explaining every column and test.
- Window columns auto-redistribute on horizontal resize.
- Route filter, Group-by-Route summary, Route Matrix, Plot dialog (unchanged
  from v2 but now works with extended metric list).

Typography: edit the ``UI_FONT_*`` constants near the top of this file
(see comment block) to scale text without editing widget code.

Run standalone:
    python3 compare_reports_gui.py
"""

import csv
import json
import math
import os
import unicodedata
import tkinter as tk
import tkinter.font as tkfont
from collections import defaultdict
from itertools import combinations
from tkinter import filedialog, messagebox, ttk

REPORTS_DIR = os.path.expanduser("~/ros2_ws/reports")

# ---------------------------------------------------------------------------
# UI typography — tweak these constants to change font sizes app-wide
#
#   UI_FONT_FAMILY       None = use the same family as Tk's default UI font
#                        (on Linux often DejaVu Sans / Ubuntu; on Windows Segoe UI).
#                        Or set a string, e.g. "Segoe UI", "Ubuntu", "DejaVu Sans".
#   UI_FONT_SIZE         Point size for tables, filter bar, buttons, comboboxes,
#                        menus, and dialog labels (default Tk is ~9–10).
#   UI_TREE_ROW_HEIGHT   Pixel height of each Treeview row; raise if text looks
#                        clipped after increasing UI_FONT_SIZE.
#   UI_FONT_STATS        (family, size) for Analysis → Run Statistics (monospace).
#   UI_LEGEND_BODY / H1 / H2 / CODE — Help → Metrics & Tests Legend text sizes.
# ---------------------------------------------------------------------------
UI_FONT_FAMILY = None
UI_FONT_SIZE = 11
UI_TREE_ROW_HEIGHT = 28
UI_FONT_STATS = ("Courier", 15)
UI_LEGEND_BODY = 14
UI_LEGEND_H1 = 16
UI_LEGEND_H2 = 14
UI_LEGEND_CODE = 12

# ---------------------------------------------------------------------------
# Metric definitions
# ---------------------------------------------------------------------------

# Keys that come directly from the JSON "metrics" dict (or are derived and
# injected into it by _compute_derived_metrics before anything else runs).
METRIC_KEYS = [
    # ── Path quality ──────────────────────────────────────────────────────
    "path_length_planned_m",    # total length of the planned path
    "distance_traveled_m",      # actual distance the robot traveled
    "straight_line_dist_m",     # Euclidean start→goal distance (derived)
    "path_optimality_ratio",    # path_length / straight_line_dist  (derived)
    "path_smoothness_rad",      # cumulative heading-change along path
    # ── Timing ────────────────────────────────────────────────────────────
    "planning_time_s",          # time spent in the global planner
    "execution_time_s",         # time spent following the path
    "total_time_s",             # planning + execution
    # ── Performance ───────────────────────────────────────────────────────
    "completeness",             # 1 = goal reached, 0 = failed
    "avg_velocity_ms",          # mean speed while moving
    "energy_efficiency",        # distance_traveled / total_time  (or custom)
    "num_recoveries",           # number of Nav2 recovery behaviours triggered
]

METRIC_LABELS = {
    "path_length_planned_m":  "Path Length (m)",
    "distance_traveled_m":    "Dist Traveled (m)",
    "straight_line_dist_m":   "Straight-Line (m)",
    "path_optimality_ratio":  "Optimality Ratio",
    "path_smoothness_rad":    "Smoothness (rad)",
    "planning_time_s":        "Planning Time (s)",
    "execution_time_s":       "Exec Time (s)",
    "total_time_s":           "Total Time (s)",
    "completeness":           "Completeness",
    "avg_velocity_ms":        "Avg Vel (m/s)",
    "energy_efficiency":      "Energy Eff.",
    "num_recoveries":         "Recoveries",
}

# "lower" = lower value is better, "higher" = higher is better, "info" = neutral
METRIC_BETTER = {
    "path_length_planned_m": "lower",
    "distance_traveled_m":   "lower",
    "straight_line_dist_m":  "info",
    "path_optimality_ratio": "lower",
    "path_smoothness_rad":   "lower",
    "planning_time_s":       "lower",
    "execution_time_s":      "lower",
    "total_time_s":          "lower",
    "completeness":          "higher",
    "avg_velocity_ms":       "higher",
    "energy_efficiency":     "higher",
    "num_recoveries":        "lower",
}

METRIC_DESCRIPTIONS = {
    "path_length_planned_m": (
        "Total arc-length (metres) of the path returned by the global planner "
        "before execution begins.  Lower ⟹ more direct path."),
    "distance_traveled_m": (
        "Actual odometry distance (metres) the robot traveled during execution. "
        "Includes detours from recoveries.  Lower ⟹ less waste."),
    "straight_line_dist_m": (
        "Euclidean (straight-line) distance from start pose to goal pose. "
        "Computed automatically from the JSON start/goal coordinates. "
        "Used as a denominator for the Optimality Ratio.  Informational only."),
    "path_optimality_ratio": (
        "path_length_planned_m ÷ straight_line_dist_m.  A value of 1.0 means "
        "the planner found a perfectly straight path. Higher values indicate "
        "longer, less direct routes.  Closer to 1.0 is better."),
    "path_smoothness_rad": (
        "Sum of absolute heading changes (radians) between consecutive waypoints. "
        "Smaller values mean a smoother, less jerky path – important for drones "
        "because sharp turns cost energy and time."),
    "planning_time_s": (
        "Wall-clock time (seconds) the global planner spent computing the path. "
        "Critical for real-time applications.  Lower ⟹ faster planner."),
    "execution_time_s": (
        "Wall-clock time (seconds) from when the robot started following the "
        "path until it either reached the goal or aborted."),
    "total_time_s": (
        "planning_time_s + execution_time_s.  The end-to-end mission time; "
        "the most practically relevant timing metric."),
    "completeness": (
        "Binary outcome: 1 if the robot reached the goal (SUCCEEDED), 0 "
        "otherwise.  The algorithm summary shows the mean, which equals the "
        "success rate across repeated trials."),
    "avg_velocity_ms": (
        "Mean translational speed (m/s) while the robot was in motion. "
        "Higher speed generally means more efficient execution, but may "
        "increase collision risk near obstacles."),
    "energy_efficiency": (
        "A composite metric – often defined as distance_traveled ÷ total_time "
        "or a custom energy model.  Higher ⟹ more efficient."),
    "num_recoveries": (
        "Number of Nav2 recovery behaviours (spin, backup, wait) triggered "
        "during the run.  Ideally zero for a clean environment."),
}

STAT_DESCRIPTIONS = {
    "Shapiro-Wilk normality test": (
        "PURPOSE\n"
        "  Tests whether a set of values could plausibly come from a normal\n"
        "  (Gaussian) distribution.\n\n"
        "HYPOTHESES\n"
        "  H₀ (null)        : the data IS normally distributed.\n"
        "  H₁ (alternative) : the data is NOT normally distributed.\n\n"
        "OUTPUT\n"
        "  W statistic (0–1, closer to 1 = more normal)\n"
        "  p-value: if p < 0.05 we REJECT H₀ → data is NOT normal.\n\n"
        "WHY IT MATTERS\n"
        "  Many classical tests (t-test, ANOVA) assume normality.\n"
        "  If data is not normal, use non-parametric tests instead\n"
        "  (Kruskal-Wallis, Mann-Whitney U) – which this tool does.\n\n"
        "CAUTION\n"
        "  Needs n ≥ 3 samples.  With very small n, it has low power.\n"
        "  With very large n, even tiny, irrelevant deviations are flagged."
    ),
    "Kruskal-Wallis H-test": (
        "PURPOSE\n"
        "  Non-parametric test: checks whether ≥2 independent groups have the\n"
        "  same median (rank distribution).  It is the non-parametric analogue\n"
        "  of one-way ANOVA.\n\n"
        "HYPOTHESES\n"
        "  H₀: all algorithm groups have the same population distribution.\n"
        "  H₁: at least one algorithm group is stochastically different.\n\n"
        "OUTPUT\n"
        "  H statistic (approximately χ² distributed)\n"
        "  p-value: p < 0.05 → SIGNIFICANT → at least one group differs.\n\n"
        "WHY IT MATTERS\n"
        "  Unlike ANOVA, it does not assume normality, making it ideal\n"
        "  for small robotics benchmark datasets.\n\n"
        "NEXT STEP\n"
        "  When significant, run pairwise Mann-Whitney U to find which\n"
        "  specific algorithms differ."
    ),
    "Mann-Whitney U test (pairwise)": (
        "PURPOSE\n"
        "  Pairwise non-parametric test: compares two independent groups to\n"
        "  determine whether one tends to produce larger values than the other.\n\n"
        "HYPOTHESES\n"
        "  H₀: the two algorithms have the same distribution.\n"
        "  H₁: one algorithm stochastically dominates the other.\n\n"
        "OUTPUT\n"
        "  U statistic, p-value (two-sided), and rank-biserial r effect size.\n\n"
        "EFFECT SIZE (rank-biserial r)\n"
        "  r = 1 − (2U) / (n₁ × n₂)   ranges −1 to +1\n"
        "  |r| < 0.10 : negligible\n"
        "  |r| 0.10–0.30 : small\n"
        "  |r| 0.30–0.50 : medium\n"
        "  |r| > 0.50 : large\n\n"
        "WHY IT MATTERS\n"
        "  p < 0.05 means the difference is unlikely to be random noise.\n"
        "  The effect size tells you whether it is practically meaningful."
    ),
    "Descriptive statistics": (
        "For each algorithm the following are reported per metric:\n\n"
        "  n       : number of runs\n"
        "  mean    : arithmetic average\n"
        "  std     : sample standard deviation (spread)\n"
        "  CV%     : coefficient of variation = std/mean × 100.\n"
        "            Measures consistency – lower CV = more repeatable.\n"
        "  min/max : range of observed values\n"
        "  median  : middle value (50th percentile) – robust to outliers\n"
        "  Q1/Q3   : 25th / 75th percentiles (IQR = Q3 − Q1)\n"
        "  95% CI  : 95% confidence interval for the mean\n"
        "            (t-distribution based when scipy is available)"
    ),
    "Success rate": (
        "Percentage of runs where status == 'SUCCEEDED'.\n"
        "Computed per algorithm from the filtered dataset.\n"
        "This is the most direct measure of algorithm reliability:\n"
        "  100% = always reaches the goal\n"
        "    0% = never reaches the goal\n"
        "Compare alongside other metrics – a fast algorithm that often\n"
        "fails may be worse than a slower but reliable one."
    ),
}

COLUMNS = [
    ("run_id",    "Run ID"),
    ("algorithm", "Algorithm"),
    ("map_file",  "Map"),
    ("start",     "Start (x,y)"),
    ("goal",      "Goal (x,y)"),
    ("status",    "Status"),
] + [(k, METRIC_LABELS[k]) for k in METRIC_KEYS]

COL_IDS      = [c[0] for c in COLUMNS]
COL_HEADINGS = [c[1] for c in COLUMNS]
NUMERIC_COLS = set(METRIC_KEYS)

STATUSES = ["All", "SUCCEEDED", "ABORTED", "FAILED", "TIMEOUT"]
PALETTE  = ["#4C72B0", "#DD8452", "#55A868", "#C44E52",
            "#8172B3", "#937860", "#DA8BC3", "#CCB974"]

PLOT_TYPES = [
    ("boxplot",   "Boxplot by Algorithm"),
    ("bar_route", "Bar Chart: Algorithm × Route"),
    ("scatter",   "Multi-Scatter Dashboard"),
    ("multi_box", "Multi-metric Boxplots (6 metrics)"),
]

# Default column widths (px)
_COL_WIDTHS = {
    "run_id": 80, "algorithm": 100, "map_file": 130,
    "start": 120, "goal": 120, "status": 90,
}
for _k in METRIC_KEYS:
    _COL_WIDTHS[_k] = 115


# ---------------------------------------------------------------------------
# Data helpers
# ---------------------------------------------------------------------------

def _fmt_coord(val):
    try:
        return f"{float(val):.2f}"
    except (TypeError, ValueError):
        return str(val)


def _route_label(r):
    sp = r.get("start_pose", {})
    gp = r.get("goal_pose",  {})
    sx, sy = _fmt_coord(sp.get("x", 0)), _fmt_coord(sp.get("y", 0))
    gx, gy = _fmt_coord(gp.get("x", 0)), _fmt_coord(gp.get("y", 0))
    return f"({sx},{sy})\u2192({gx},{gy})"


def _compute_derived_metrics(r):
    """Inject computed metrics into the report's 'metrics' dict if absent."""
    m  = r.setdefault("metrics", {})
    sp = r.get("start_pose", {})
    gp = r.get("goal_pose",  {})
    sx, sy = float(sp.get("x", 0)), float(sp.get("y", 0))
    gx, gy = float(gp.get("x", 0)), float(gp.get("y", 0))
    sld = math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)
    if "straight_line_dist_m" not in m:
        m["straight_line_dist_m"] = round(sld, 4)
    if "path_optimality_ratio" not in m:
        plen = m.get("path_length_planned_m", 0)
        m["path_optimality_ratio"] = round(plen / sld, 4) if sld > 0.01 and plen > 0 else 0.0


def load_reports_from_dir(directory):
    reports = []
    if not os.path.isdir(directory):
        return reports
    for fname in sorted(os.listdir(directory)):
        if not fname.endswith(".json"):
            continue
        fpath = os.path.join(directory, fname)
        try:
            with open(fpath) as f:
                data = json.load(f)
            data["_file"]  = fname
            _compute_derived_metrics(data)
            data["_route"] = _route_label(data)
            reports.append(data)
        except (json.JSONDecodeError, OSError):
            pass
    return reports


def report_to_row(r):
    m  = r.get("metrics",    {})
    sp = r.get("start_pose", {})
    gp = r.get("goal_pose",  {})
    start = f"({sp.get('x', 0):.1f}, {sp.get('y', 0):.1f})"
    goal  = f"({gp.get('x', 0):.1f}, {gp.get('y', 0):.1f})"
    row = {
        "run_id":    r.get("run_id",    "?"),
        "algorithm": r.get("algorithm", "?"),
        "map_file":  r.get("map_file",  "?"),
        "start":     start,
        "goal":      goal,
        "status":    r.get("status",    "?"),
    }
    for k in METRIC_KEYS:
        row[k] = m.get(k, 0)
    return row


def _mean_std(values):
    n = len(values)
    if n == 0:
        return 0.0, 0.0
    mean = sum(values) / n
    std  = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    return mean, std


def _quartiles(sorted_vals):
    """Return (Q1, median, Q3) using linear interpolation."""
    n = len(sorted_vals)
    if n == 0:
        return 0.0, 0.0, 0.0

    def _interp(idx):
        lo = int(idx)
        hi = min(lo + 1, n - 1)
        frac = idx - lo
        return sorted_vals[lo] + frac * (sorted_vals[hi] - sorted_vals[lo])

    return _interp((n - 1) * 0.25), _interp((n - 1) * 0.5), _interp((n - 1) * 0.75)


def _fmt_metric(val, key):
    fmts = {
        "path_length_planned_m": "{:.3f}",
        "distance_traveled_m":   "{:.3f}",
        "straight_line_dist_m":  "{:.3f}",
        "path_optimality_ratio": "{:.4f}",
        "path_smoothness_rad":   "{:.4f}",
        "planning_time_s":       "{:.4f}",
        "execution_time_s":      "{:.2f}",
        "total_time_s":          "{:.2f}",
        "completeness":          "{:.0f}",
        "avg_velocity_ms":       "{:.3f}",
        "energy_efficiency":     "{:.3f}",
        "num_recoveries":        "{:.0f}",
    }
    return fmts.get(key, "{:.4f}").format(val)


def _better_text(metric_key):
    tag = METRIC_BETTER.get(metric_key, "info")
    if tag == "higher":
        return "higher is better"
    if tag == "lower":
        return "lower is better"
    return "info"


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------

class BenchmarkApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Navigation Benchmark Analyzer  v3")
        self.geometry("1600x900")
        self.minsize(900, 560)

        self.reports_dir      = REPORTS_DIR
        self.all_reports      = []
        self.filtered_reports = []
        self.sort_col         = None
        self.sort_reverse     = False
        self.group_by_route   = tk.BooleanVar(value=False)
        self._resize_after_id = None

        self._apply_ui_fonts()
        self._build_menu()
        self._build_filter_panel()
        self._build_main_table()
        self._build_summary_panel()

        self._load_reports()

    def _apply_ui_fonts(self):
        """Apply UI_FONT_* settings to ttk widgets (tables, filters, dialogs)."""
        if UI_FONT_FAMILY:
            fam = UI_FONT_FAMILY
        else:
            try:
                fam = tkfont.nametofont("TkDefaultFont").actual()["family"]
            except tk.TclError:
                fam = "sans-serif"
        self._ui_font = (fam, UI_FONT_SIZE)

        style = ttk.Style(self)
        # Main UI
        style.configure("Treeview", font=self._ui_font, rowheight=UI_TREE_ROW_HEIGHT)
        style.configure("Treeview.Heading", font=(fam, UI_FONT_SIZE, "bold"))
        style.configure("TLabel", font=self._ui_font)
        style.configure("TButton", font=self._ui_font)
        style.configure("TCheckbutton", font=self._ui_font)
        style.configure("TLabelframe.Label", font=self._ui_font)
        style.configure("TCombobox", font=self._ui_font)
        style.configure("TNotebook.Tab", font=self._ui_font)
        style.configure("TMenubutton", font=self._ui_font)

    # ---- menu -----------------------------------------------------------

    def _build_menu(self):
        mf = getattr(self, "_ui_font", ("TkDefaultFont", UI_FONT_SIZE))
        menubar = tk.Menu(self, font=mf)

        file_menu = tk.Menu(menubar, tearoff=0, font=mf)
        file_menu.add_command(label="Open Directory…", command=self._open_directory)
        file_menu.add_command(label="Export CSV…",     command=self._export_csv)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.quit)
        menubar.add_cascade(label="File", menu=file_menu)

        analysis_menu = tk.Menu(menubar, tearoff=0, font=mf)
        analysis_menu.add_command(label="Run Statistics",  command=self._run_statistics)
        analysis_menu.add_command(label="Generate Plots…", command=self._show_plot_dialog)
        analysis_menu.add_command(label="Route Matrix…",   command=self._show_route_matrix)
        menubar.add_cascade(label="Analysis", menu=analysis_menu)

        help_menu = tk.Menu(menubar, tearoff=0, font=mf)
        help_menu.add_command(label="Metrics & Tests Legend", command=self._show_legend)
        menubar.add_cascade(label="Help", menu=help_menu)

        self.config(menu=menubar)

    # ---- filter panel ---------------------------------------------------

    def _build_filter_panel(self):
        frame = ttk.LabelFrame(self, text="Filters", padding=6)
        frame.pack(fill=tk.X, padx=8, pady=(8, 0))

        ttk.Label(frame, text="Algorithm:").pack(side=tk.LEFT, padx=(0, 3))
        self.algo_var = tk.StringVar(value="All")
        self.algo_cb  = ttk.Combobox(frame, textvariable=self.algo_var,
                                     values=["All"], state="readonly", width=13)
        self.algo_cb.pack(side=tk.LEFT, padx=(0, 8))
        self.algo_cb.bind("<<ComboboxSelected>>", lambda _: self._apply_filters())

        ttk.Label(frame, text="Map:").pack(side=tk.LEFT, padx=(0, 3))
        self.map_var = tk.StringVar(value="All")
        self.map_cb  = ttk.Combobox(frame, textvariable=self.map_var,
                                    values=["All"], state="readonly", width=18)
        self.map_cb.pack(side=tk.LEFT, padx=(0, 8))
        self.map_cb.bind("<<ComboboxSelected>>", lambda _: self._apply_filters())

        ttk.Label(frame, text="Status:").pack(side=tk.LEFT, padx=(0, 3))
        self.status_var = tk.StringVar(value="All")
        self.status_cb  = ttk.Combobox(frame, textvariable=self.status_var,
                                       values=STATUSES, state="readonly", width=11)
        self.status_cb.pack(side=tk.LEFT, padx=(0, 8))
        self.status_cb.bind("<<ComboboxSelected>>", lambda _: self._apply_filters())

        ttk.Label(frame, text="Route \u2192:").pack(side=tk.LEFT, padx=(0, 3))
        self.route_var = tk.StringVar(value="All")
        self.route_cb  = ttk.Combobox(frame, textvariable=self.route_var,
                                      values=["All"], state="readonly", width=28)
        self.route_cb.pack(side=tk.LEFT, padx=(0, 8))
        self.route_cb.bind("<<ComboboxSelected>>", lambda _: self._apply_filters())

        ttk.Button(frame, text="Reset",   command=self._reset_filters).pack(side=tk.LEFT, padx=(2, 2))
        ttk.Button(frame, text="Refresh", command=self._load_reports).pack(side=tk.LEFT)

        self.count_label = ttk.Label(frame, text="", foreground="#555")
        self.count_label.pack(side=tk.RIGHT, padx=(8, 0))

    # ---- main treeview --------------------------------------------------

    def _build_main_table(self):
        self._table_container = ttk.Frame(self)
        self._table_container.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        xscroll = ttk.Scrollbar(self._table_container, orient=tk.HORIZONTAL)
        yscroll = ttk.Scrollbar(self._table_container, orient=tk.VERTICAL)

        self.tree = ttk.Treeview(
            self._table_container, columns=COL_IDS, show="headings",
            xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)

        xscroll.config(command=self.tree.xview)
        yscroll.config(command=self.tree.yview)

        for cid, heading in COLUMNS:
            w = _COL_WIDTHS.get(cid, 115)
            self.tree.heading(cid, text=heading,
                              command=lambda c=cid: self._sort_by_column(c))
            self.tree.column(cid, width=w, minwidth=45, stretch=True,
                             anchor=tk.E if cid in NUMERIC_COLS else tk.W)

        self.tree.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        self._table_container.rowconfigure(0, weight=1)
        self._table_container.columnconfigure(0, weight=1)

        # Auto-redistribute column widths when window is resized
        self._table_container.bind("<Configure>", self._on_table_resize)

    def _on_table_resize(self, event):
        """Spread extra horizontal space across all columns proportionally."""
        if self._resize_after_id:
            self.after_cancel(self._resize_after_id)
        self._resize_after_id = self.after(80, lambda w=event.width: self._do_resize(w))

    def _do_resize(self, available_px):
        total_defined = sum(_COL_WIDTHS.get(c, 115) for c in COL_IDS)
        avail = max(available_px - 20, total_defined)   # -20 for yscroll
        if avail <= total_defined:
            return
        factor = avail / total_defined
        for cid in COL_IDS:
            base = _COL_WIDTHS.get(cid, 115)
            self.tree.column(cid, width=max(45, int(base * factor)))

    # ---- summary panel --------------------------------------------------

    def _build_summary_panel(self):
        self._summary_lf = ttk.LabelFrame(
            self, text="Algorithm Summary – reflects current filter", padding=6)
        self._summary_lf.pack(fill=tk.X, padx=8, pady=(0, 8))

        ctrl = ttk.Frame(self._summary_lf)
        ctrl.pack(fill=tk.X, pady=(0, 4))
        ttk.Checkbutton(ctrl, text="Group by Route",
                        variable=self.group_by_route,
                        command=self._populate_summary).pack(side=tk.LEFT)

        summary_cols     = ["algorithm", "route", "runs"] + METRIC_KEYS
        summary_headings = ["Algorithm", "Route", "Runs"] + [METRIC_LABELS[k] for k in METRIC_KEYS]

        xscroll = ttk.Scrollbar(self._summary_lf, orient=tk.HORIZONTAL)
        self.summary_tree = ttk.Treeview(
            self._summary_lf, columns=summary_cols, show="headings", height=5,
            xscrollcommand=xscroll.set)
        xscroll.config(command=self.summary_tree.xview)

        for cid, heading in zip(summary_cols, summary_headings):
            w = 115 if cid in NUMERIC_COLS else 90
            if cid == "route": w = 210
            self.summary_tree.heading(cid, text=heading)
            self.summary_tree.column(cid, width=w, minwidth=45,
                                     anchor=tk.E if cid in NUMERIC_COLS else tk.W)

        self.summary_tree.pack(fill=tk.X, expand=False)
        xscroll.pack(fill=tk.X)

    # ---- data loading / filtering ---------------------------------------

    def _load_reports(self):
        self.all_reports = load_reports_from_dir(self.reports_dir)
        algos  = sorted({r.get("algorithm", "?") for r in self.all_reports})
        maps   = sorted({r.get("map_file",   "?") for r in self.all_reports})
        routes = sorted({r["_route"]              for r in self.all_reports})
        self.algo_cb["values"]  = ["All"] + algos
        self.map_cb["values"]   = ["All"] + maps
        self.route_cb["values"] = ["All"] + routes
        self._apply_filters()

    def _reset_filters(self):
        self.algo_var.set("All")
        self.map_var.set("All")
        self.status_var.set("All")
        self.route_var.set("All")
        self._apply_filters()

    def _apply_filters(self):
        algo   = self.algo_var.get()
        map_f  = self.map_var.get()
        status = self.status_var.get()
        route  = self.route_var.get()

        filtered = self.all_reports
        if algo   != "All": filtered = [r for r in filtered if r.get("algorithm") == algo]
        if map_f  != "All": filtered = [r for r in filtered if r.get("map_file")  == map_f]
        if status != "All": filtered = [r for r in filtered if r.get("status")    == status]
        if route  != "All": filtered = [r for r in filtered if r["_route"]        == route]

        self.filtered_reports = filtered
        self._populate_main_table()
        self._populate_summary()

        active = []
        if algo   != "All": active.append(f"Algo={algo}")
        if map_f  != "All": active.append(f"Map={map_f}")
        if status != "All": active.append(f"Status={status}")
        if route  != "All": active.append(f"Route={route}")
        fs = ", ".join(active) if active else "no filters"
        self.count_label.config(text=f"{len(filtered)} run(s)  [{fs}]")

    def _populate_main_table(self):
        self.tree.delete(*self.tree.get_children())
        rows = [report_to_row(r) for r in self.filtered_reports]

        if self.sort_col:
            try:
                if self.sort_col in NUMERIC_COLS:
                    rows.sort(key=lambda r: float(r.get(self.sort_col, 0)),
                              reverse=self.sort_reverse)
                else:
                    rows.sort(key=lambda r: str(r.get(self.sort_col, "")),
                              reverse=self.sort_reverse)
            except (ValueError, TypeError):
                pass

        for row in rows:
            vals = []
            for cid in COL_IDS:
                v = row[cid]
                if cid in NUMERIC_COLS:
                    v = _fmt_metric(v, cid)
                vals.append(v)
            self.tree.insert("", tk.END, values=vals)

    def _populate_summary(self):
        self.summary_tree.delete(*self.summary_tree.get_children())

        parts = []
        if self.algo_var.get()  != "All": parts.append(f"Algo={self.algo_var.get()}")
        if self.route_var.get() != "All": parts.append(f"Route={self.route_var.get()}")
        suffix = f"  [{', '.join(parts)}]" if parts else ""
        self._summary_lf.config(
            text=f"Algorithm Summary – reflects current filter{suffix}")

        if self.group_by_route.get():
            groups = defaultdict(list)
            for r in self.filtered_reports:
                groups[(r.get("algorithm", "?"), r["_route"])].append(r)
            for (algo, route) in sorted(groups):
                self._insert_summary_row(algo, route, groups[(algo, route)])
        else:
            by_algo = defaultdict(list)
            for r in self.filtered_reports:
                by_algo[r.get("algorithm", "?")].append(r)
            for algo in sorted(by_algo):
                runs      = by_algo[algo]
                all_rts   = sorted({r["_route"] for r in runs})
                route_str = all_rts[0] if len(all_rts) == 1 else f"{len(all_rts)} routes"
                self._insert_summary_row(algo, route_str, runs)

    def _insert_summary_row(self, algo, route_str, runs):
        n    = len(runs)
        vals = [algo, route_str, str(n)]
        for k in METRIC_KEYS:
            mv          = [r.get("metrics", {}).get(k, 0) for r in runs]
            mean, std   = _mean_std(mv)
            vals.append(f"{mean:.3f} \u00b1 {std:.3f}" if n > 1 else _fmt_metric(mean, k))
        self.summary_tree.insert("", tk.END, values=vals)

    # ---- column sorting -------------------------------------------------

    def _sort_by_column(self, col):
        if self.sort_col == col:
            self.sort_reverse = not self.sort_reverse
        else:
            self.sort_col     = col
            self.sort_reverse = False
        self._populate_main_table()

    # ---- file actions ---------------------------------------------------

    def _open_directory(self):
        d = filedialog.askdirectory(
            title="Select Reports Directory", initialdir=self.reports_dir)
        if d:
            self.reports_dir = d
            self._load_reports()

    def _export_csv(self):
        if not self.filtered_reports:
            messagebox.showinfo("Export", "No data to export.")
            return
        path = filedialog.asksaveasfilename(
            title="Export CSV", defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        fieldnames = (["run_id", "algorithm", "map_file", "route",
                       "start_x", "start_y", "goal_x", "goal_y", "status"]
                      + METRIC_KEYS)
        try:
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for r in self.filtered_reports:
                    m  = r.get("metrics",    {})
                    sp = r.get("start_pose", {})
                    gp = r.get("goal_pose",  {})
                    row = {
                        "run_id":    r.get("run_id",    ""),
                        "algorithm": r.get("algorithm", ""),
                        "map_file":  r.get("map_file",  ""),
                        "route":     r.get("_route",    ""),
                        "start_x":   sp.get("x", ""),
                        "start_y":   sp.get("y", ""),
                        "goal_x":    gp.get("x", ""),
                        "goal_y":    gp.get("y", ""),
                        "status":    r.get("status", ""),
                    }
                    for k in METRIC_KEYS:
                        row[k] = m.get(k, 0)
                    writer.writerow(row)
            messagebox.showinfo("Export", f"CSV saved to:\n{path}")
        except OSError as e:
            messagebox.showerror("Export Error", str(e))

    # ---- statistical analysis -------------------------------------------

    def _run_statistics(self):
        try:
            from scipy.stats import kruskal, mannwhitneyu, shapiro
            _has_scipy = True
        except ImportError:
            _has_scipy = False
            messagebox.showwarning(
                "scipy not found",
                "scipy is not installed – Shapiro-Wilk and Kruskal-Wallis will be skipped.\n"
                "Install with:  pip install scipy\n\n"
                "Descriptive statistics will still be shown.")

        reports = self.filtered_reports
        if not reports:
            messagebox.showinfo("Statistics", "No data loaded.")
            return

        by_algo = defaultdict(list)
        for r in reports:
            by_algo[r.get("algorithm", "?")].append(r)

        win    = tk.Toplevel(self)
        win.title("Statistical Analysis – current filter")
        win.geometry("900x700")

        toolbar = ttk.Frame(win)
        toolbar.pack(fill=tk.X, padx=4, pady=2)

        text   = tk.Text(win, wrap=tk.NONE, font=UI_FONT_STATS)
        xsc    = ttk.Scrollbar(win, orient=tk.HORIZONTAL, command=text.xview)
        ysc    = ttk.Scrollbar(win, orient=tk.VERTICAL,   command=text.yview)
        text.config(xscrollcommand=xsc.set, yscrollcommand=ysc.set)
        ysc.pack(side=tk.RIGHT,  fill=tk.Y)
        xsc.pack(side=tk.BOTTOM, fill=tk.X)
        text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        def _plain_text(s):
            """
            Convert unicode-heavy report text into plain ASCII for safer
            paste into Word/Docs/Excel.
            """
            replacements = {
                "→": "->",
                "–": "-",
                "—": "-",
                "−": "-",
                "×": "x",
                "★": "*",
                "✓": "PASS",
                "✘": "FAIL",
                "₀": "0",
                "₁": "1",
                "₂": "2",
                "₃": "3",
                "₄": "4",
                "₅": "5",
                "₆": "6",
                "₇": "7",
                "₈": "8",
                "₉": "9",
                "│": "|",
                "█": "#",
            }
            for old, new in replacements.items():
                s = s.replace(old, new)
            # Drop any other non-ASCII combining characters after replacement.
            s = unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode("ascii")
            # Normalize line endings for consistent cross-app paste behavior.
            return s.replace("\r\n", "\n").replace("\r", "\n")

        def _copy():
            payload = text.get("1.0", tk.END).strip("\n")
            win.clipboard_clear()
            win.clipboard_append(payload)
            win.update_idletasks()
            messagebox.showinfo("Copied", "Statistics copied to clipboard.", parent=win)

        def _copy_plain():
            payload = _plain_text(text.get("1.0", tk.END)).strip("\n")
            win.clipboard_clear()
            win.clipboard_append(payload)
            win.update_idletasks()
            messagebox.showinfo(
                "Copied",
                "Plain-text statistics copied (best for report paste).",
                parent=win
            )

        def _save_plain():
            path = filedialog.asksaveasfilename(
                title="Save statistics as text",
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
            )
            if not path:
                return
            try:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(_plain_text(text.get("1.0", tk.END)).strip("\n") + "\n")
                messagebox.showinfo("Saved", f"Statistics saved to:\n{path}", parent=win)
            except OSError as e:
                messagebox.showerror("Save Error", str(e), parent=win)

        ttk.Button(toolbar, text="Copy to Clipboard", command=_copy).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Copy Plain Text", command=_copy_plain).pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="Save .txt", command=_save_plain).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Help: Test Explanations",
                   command=lambda: self._show_legend(section="stats")).pack(side=tk.LEFT, padx=6)

        algo_names  = sorted(by_algo.keys())
        route_names = sorted({r["_route"] for r in reports})

        af = []
        if self.algo_var.get()   != "All": af.append(f"Algorithm={self.algo_var.get()}")
        if self.map_var.get()    != "All": af.append(f"Map={self.map_var.get()}")
        if self.status_var.get() != "All": af.append(f"Status={self.status_var.get()}")
        if self.route_var.get()  != "All": af.append(f"Route={self.route_var.get()}")

        out = []
        W   = 76
        out.append("=" * W)
        out.append("  STATISTICAL ANALYSIS  –  NAVIGATION BENCHMARK")
        out.append("=" * W)
        out.append(f"\nFilters  : {', '.join(af) or 'none'}")
        out.append(f"Algos    : {', '.join(algo_names)}")
        out.append(f"Routes   : {', '.join(route_names)}")
        out.append(f"Runs     : {len(reports)}")
        for a in algo_names:
            n   = len(by_algo[a])
            suc = sum(1 for r in by_algo[a] if r.get("status") == "SUCCEEDED")
            out.append(f"  {a:16s}: n={n:3d},  success={suc}/{n} ({100*suc/n:.0f}%)"
                       + ("  \u26a0 n<10" if n < 10 else ""))
        out.append("")

        # ── Success rate section ─────────────────────────────────────────
        out.append("─" * W)
        out.append("  SUCCESS RATE  (runs where status == 'SUCCEEDED')")
        out.append("─" * W)
        for a in algo_names:
            runs = by_algo[a]
            suc  = sum(1 for r in runs if r.get("status") == "SUCCEEDED")
            bar  = "\u2588" * int(suc / len(runs) * 20) if runs else ""
            out.append(f"  {a:16s}: {suc:3d}/{len(runs)}  {100*suc/len(runs):5.1f}%  |{bar:<20}|")
        out.append("")

        # ── Per-metric sections ──────────────────────────────────────────
        for key in METRIC_KEYS:
            label  = METRIC_LABELS[key]
            better = METRIC_BETTER.get(key, "info")
            out.append("=" * W)
            out.append(f"  {label}   [{better} = better]")
            out.append("=" * W)

            algo_vals = {a: [r.get("metrics", {}).get(key, 0) for r in by_algo[a]]
                         for a in algo_names}

            # ── Descriptive statistics ───────────────────────────────────
            out.append("\n  DESCRIPTIVE STATISTICS")
            out.append(f"  {'Algorithm':16s}  {'n':>3}  {'mean':>9}  "
                       f"{'std':>9}  {'CV%':>6}  {'min':>9}  "
                       f"{'Q1':>9}  {'median':>9}  {'Q3':>9}  {'max':>9}  95% CI")
            out.append("  " + "-" * (W - 2))
            for a in algo_names:
                v = sorted(algo_vals[a])
                n = len(v)
                if n == 0:
                    out.append(f"  {a:16s}  {'0':>3}  (no data)")
                    continue
                mean, std = _mean_std(v)
                cv        = (std / mean * 100) if mean != 0 else 0.0
                vmin, vmax = v[0], v[-1]
                q1, med, q3 = _quartiles(v)
                if n >= 2:
                    se = std / math.sqrt(n)
                    try:
                        from scipy.stats import t as t_dist
                        t_crit = t_dist.ppf(0.975, df=n - 1)
                    except Exception:
                        t_crit = 1.96
                    ci = f"[{mean - t_crit*se:.3f}, {mean + t_crit*se:.3f}]"
                else:
                    ci = "n/a"
                out.append(
                    f"  {a:16s}  {n:>3d}  {mean:>9.4f}  {std:>9.4f}  "
                    f"{cv:>5.1f}%  {vmin:>9.4f}  {q1:>9.4f}  "
                    f"{med:>9.4f}  {q3:>9.4f}  {vmax:>9.4f}  {ci}")

            if not _has_scipy:
                out.append("\n  (scipy not installed – hypothesis tests skipped)")
                out.append("")
                continue

            # ── Shapiro-Wilk ─────────────────────────────────────────────
            out.append("\n  SHAPIRO-WILK NORMALITY  (H\u2080: data is normal)")
            for a in algo_names:
                v = algo_vals[a]
                if len(v) < 3:
                    out.append(f"  {a:16s}: skipped (n < 3)")
                    continue
                if len(set(v)) == 1:
                    out.append(f"  {a:16s}: all values identical, skipped")
                    continue
                stat, p = shapiro(v)
                tag = "NORMAL \u2713" if p >= 0.05 else "NOT normal \u2718"
                out.append(f"  {a:16s}: W={stat:.4f}  p={p:.4f}  [{tag}]")

            # ── Kruskal-Wallis ────────────────────────────────────────────
            groups = [algo_vals[a] for a in algo_names if algo_vals[a]]
            if len(groups) >= 2:
                try:
                    h_stat, h_p = kruskal(*groups)
                    sig = "SIGNIFICANT \u2605" if h_p < 0.05 else "not significant"
                    out.append(f"\n  KRUSKAL-WALLIS H-TEST: H={h_stat:.4f}  p={h_p:.4f}  [{sig}]")
                    if h_p < 0.05 and len(algo_names) > 1:
                        out.append("\n  MANN-WHITNEY U  pairwise (with rank-biserial effect size r)")
                        out.append(f"  {'Pair':34s}  {'U':>8}  {'p':>7}  {'r':>7}  effect  sig")
                        out.append("  " + "-" * 64)
                        for a1, a2 in combinations(algo_names, 2):
                            v1, v2 = algo_vals[a1], algo_vals[a2]
                            if not v1 or not v2:
                                continue
                            u_s, u_p = mannwhitneyu(v1, v2, alternative="two-sided")
                            n1n2     = len(v1) * len(v2)
                            r_rb     = 1 - (2 * u_s) / n1n2 if n1n2 > 0 else 0
                            abs_r    = abs(r_rb)
                            if   abs_r < 0.10: eff = "negligible"
                            elif abs_r < 0.30: eff = "small"
                            elif abs_r < 0.50: eff = "medium"
                            else:              eff = "large"
                            sig = "*" if u_p < 0.05 else " "
                            pair = f"{a1} vs {a2}"
                            out.append(
                                f"  {pair:34s}  {u_s:>8.1f}  {u_p:>7.4f}  {r_rb:>+7.4f}  {eff:<10}  {sig}")
                except ValueError:
                    out.append("\n  Kruskal-Wallis: skipped (insufficient data)")
            else:
                out.append("\n  Kruskal-Wallis: skipped (\u2265 2 groups needed)")
            out.append("")

        text.insert(tk.END, "\n".join(out))
        text.config(state=tk.DISABLED)

    # ---- plot dialog ----------------------------------------------------

    def _show_plot_dialog(self):
        if not self.filtered_reports:
            messagebox.showinfo("Plots", "No data loaded.")
            return
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            messagebox.showerror(
                "Missing dependency",
                "matplotlib is required.\nInstall with:  pip install matplotlib")
            return

        dlg = tk.Toplevel(self)
        dlg.title("Generate Plot")
        dlg.geometry("500x360")
        dlg.resizable(False, False)

        ttk.Label(dlg, text="Metric:").grid(
            row=0, column=0, sticky=tk.W, padx=14, pady=10)
        metric_var = tk.StringVar(value="total_time_s")
        ttk.Combobox(dlg, textvariable=metric_var, values=METRIC_KEYS,
                     state="readonly", width=28).grid(
            row=0, column=1, padx=8, pady=10, sticky=tk.W)

        ttk.Label(dlg, text="Plot type:").grid(
            row=1, column=0, sticky=tk.NW, padx=14, pady=4)
        plot_type_var = tk.StringVar(value="boxplot")
        ttk.Label(dlg, text="Uses selected Metric:").grid(
            row=2, column=1, sticky=tk.W, padx=8, pady=(2, 2))
        ttk.Radiobutton(dlg, text="Boxplot by Algorithm",
                        variable=plot_type_var, value="boxplot").grid(
            row=3, column=1, sticky=tk.W, padx=8, pady=2)
        ttk.Radiobutton(dlg, text="Bar Chart: Algorithm × Route",
                        variable=plot_type_var, value="bar_route").grid(
            row=4, column=1, sticky=tk.W, padx=8, pady=2)

        ttk.Separator(dlg, orient=tk.HORIZONTAL).grid(
            row=5, column=0, columnspan=2, sticky="ew", padx=10, pady=(8, 8))

        ttk.Label(dlg, text="Fixed metric dashboards (ignore dropdown):").grid(
            row=6, column=1, sticky=tk.W, padx=8, pady=(0, 2))
        ttk.Radiobutton(dlg, text="Multi-Scatter Dashboard",
                        variable=plot_type_var, value="scatter").grid(
            row=7, column=1, sticky=tk.W, padx=8, pady=2)
        ttk.Radiobutton(dlg, text="Multi-metric Boxplots (6 metrics)",
                        variable=plot_type_var, value="multi_box").grid(
            row=8, column=1, sticky=tk.W, padx=8, pady=2)

        ttk.Button(
            dlg, text="Generate",
            command=lambda: [
                dlg.destroy(),
                self._generate_plots(metric_var.get(), plot_type_var.get())
            ]
        ).grid(row=9, column=0, columnspan=2, pady=14)

    def _generate_plots(self, metric_key="total_time_s", plot_type="boxplot"):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            return

        reports = self.filtered_reports
        if not reports:
            return

        by_algo    = defaultdict(list)
        for r in reports:
            by_algo[r.get("algorithm", "?")].append(r)
        algo_names = sorted(by_algo.keys())
        colors_map = {a: PALETTE[i % len(PALETTE)] for i, a in enumerate(algo_names)}

        plt.style.use("seaborn-v0_8-whitegrid")

        if plot_type == "multi_box":
            box_metrics = [
                ("planning_time_s",       "Planning Time (s)"),
                ("execution_time_s",      "Execution Time (s)"),
                ("path_length_planned_m", "Path Length (m)"),
                ("total_time_s",          "Total Time (s)"),
                ("path_optimality_ratio", "Optimality Ratio"),
                ("path_smoothness_rad",   "Path Smoothness (rad)"),
            ]
            fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
            fig.suptitle("Metric Distributions by Algorithm (filtered)",
                         fontsize=14, fontweight="bold")
            for ax, (key, lbl) in zip(axes.flatten(), box_metrics):
                data, labs = [], []
                for a in algo_names:
                    vals = [r.get("metrics", {}).get(key, 0) for r in by_algo[a]]
                    if vals:
                        data.append(vals); labs.append(a)
                if data:
                    bp = ax.boxplot(data, labels=labs, patch_artist=True)
                    for patch, a in zip(bp["boxes"], labs):
                        patch.set_facecolor(colors_map[a]); patch.set_alpha(0.75)
                ax.set_title(f"{lbl} ({_better_text(key)})")
                ax.set_ylabel(lbl); ax.set_xlabel("Algorithm")
            fig.tight_layout(rect=[0, 0, 1, 0.95])

        elif plot_type == "boxplot":
            lbl = METRIC_LABELS.get(metric_key, metric_key)
            fig, ax = plt.subplots(figsize=(8, 5))
            fig.suptitle(f"{lbl} by Algorithm (filtered)", fontsize=13, fontweight="bold")
            data, labs = [], []
            for a in algo_names:
                vals = [r.get("metrics", {}).get(metric_key, 0) for r in by_algo[a]]
                if vals:
                    data.append(vals); labs.append(a)
            if data:
                bp = ax.boxplot(data, labels=labs, patch_artist=True)
                for patch, a in zip(bp["boxes"], labs):
                    patch.set_facecolor(colors_map[a]); patch.set_alpha(0.75)
            ax.set_ylabel(lbl); ax.set_xlabel("Algorithm")
            fig.tight_layout(rect=[0, 0, 1, 0.95])

        elif plot_type == "bar_route":
            all_routes = sorted({r["_route"] for r in reports})
            lbl        = METRIC_LABELS.get(metric_key, metric_key)
            fig, ax    = plt.subplots(figsize=(max(10, len(all_routes) * 2.4), 6))
            fig.suptitle(f"{lbl}: Algorithm \u00d7 Route (filtered)",
                         fontsize=13, fontweight="bold")
            n_algo    = len(algo_names)
            bar_width = 0.8 / max(n_algo, 1)
            x         = list(range(len(all_routes)))
            for i, a in enumerate(algo_names):
                means = []
                for route in all_routes:
                    vals = [r.get("metrics", {}).get(metric_key, 0)
                            for r in reports
                            if r.get("algorithm") == a and r["_route"] == route]
                    means.append(sum(vals) / len(vals) if vals else 0)
                offsets = [xi + (i - n_algo / 2 + 0.5) * bar_width for xi in x]
                bars    = ax.bar(offsets, means, width=bar_width * 0.9,
                                 label=a, color=colors_map[a], alpha=0.82)
                for bar, v in zip(bars, means):
                    if v > 0:
                        ax.text(bar.get_x() + bar.get_width() / 2,
                                bar.get_height() + 0.01,
                                f"{v:.2f}", ha="center", va="bottom", fontsize=7)
            ax.set_xticks(x)
            ax.set_xticklabels(all_routes, rotation=22, ha="right", fontsize=8)
            ax.set_ylabel(lbl); ax.set_xlabel("Route (Start \u2192 Goal)")
            ax.legend(title="Algorithm")
            fig.tight_layout(rect=[0, 0, 1, 0.95])

        elif plot_type == "scatter":
            pairs = [
                ("planning_time_s", "path_length_planned_m",
                 "Planning Time (s)", "Path Length (m)"),
                ("planning_time_s", "path_optimality_ratio",
                 "Planning Time (s)", "Optimality Ratio"),
                ("execution_time_s", "distance_traveled_m",
                 "Execution Time (s)", "Distance Traveled (m)"),
                ("avg_velocity_ms", "energy_efficiency",
                 "Avg Velocity (m/s)", "Energy Efficiency"),
            ]
            fig, axes = plt.subplots(2, 2, figsize=(12.5, 9))
            fig.suptitle("Multi-Scatter Dashboard (filtered)", fontsize=14, fontweight="bold")
            for ax, (xk, yk, xl, yl) in zip(axes.flatten(), pairs):
                for a in algo_names:
                    runs = by_algo[a]
                    xs = [r.get("metrics", {}).get(xk, 0) for r in runs]
                    ys = [r.get("metrics", {}).get(yk, 0) for r in runs]
                    ax.scatter(xs, ys, label=a, color=colors_map[a],
                               s=56, alpha=0.82, edgecolors="white", linewidth=0.45)
                ax.set_xlabel(xl)
                ax.set_ylabel(yl)
                xbetter = _better_text(xk)
                ybetter = _better_text(yk)
                ax.set_title(f"{yl} ({ybetter}) vs {xl} ({xbetter})")
            handles, labels = axes[0][0].get_legend_handles_labels()
            if handles:
                fig.legend(handles, labels, loc="upper center", ncol=max(1, len(labels)))
            fig.tight_layout(rect=[0, 0, 1, 0.93])

        plt.show()

    # ---- route matrix ---------------------------------------------------

    def _show_route_matrix(self):
        reports = self.filtered_reports
        if not reports:
            messagebox.showinfo("Route Matrix", "No data loaded.")
            return

        win = tk.Toplevel(self)
        win.title("Route Matrix (Algorithm \u00d7 Route)")
        win.geometry("980x500")

        ctrl = ttk.Frame(win)
        ctrl.pack(fill=tk.X, padx=8, pady=6)
        ttk.Label(ctrl, text="Metric:").pack(side=tk.LEFT)
        metric_var = tk.StringVar(value="total_time_s")
        metric_cb  = ttk.Combobox(ctrl, textvariable=metric_var,
                                  values=METRIC_KEYS, state="readonly", width=26)
        metric_cb.pack(side=tk.LEFT, padx=(4, 0))

        all_routes = sorted({r["_route"] for r in reports})
        algo_names = sorted({r.get("algorithm", "?") for r in reports})

        mf = ttk.Frame(win)
        mf.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        mc = ["algorithm"] + all_routes
        mh = ["Algorithm"] + all_routes

        xsc = ttk.Scrollbar(mf, orient=tk.HORIZONTAL)
        ysc = ttk.Scrollbar(mf, orient=tk.VERTICAL)
        mt  = ttk.Treeview(mf, columns=mc, show="headings",
                           xscrollcommand=xsc.set, yscrollcommand=ysc.set)
        xsc.config(command=mt.xview); ysc.config(command=mt.yview)

        for cid, hdr in zip(mc, mh):
            w = 100 if cid == "algorithm" else 175
            mt.heading(cid, text=hdr)
            mt.column(cid, width=w, minwidth=50,
                      anchor=tk.W if cid == "algorithm" else tk.CENTER)

        mt.grid(row=0, column=0, sticky="nsew")
        ysc.grid(row=0, column=1, sticky="ns")
        xsc.grid(row=1, column=0, sticky="ew")
        mf.rowconfigure(0, weight=1); mf.columnconfigure(0, weight=1)

        def _refresh(*_):
            key = metric_var.get()
            mt.delete(*mt.get_children())
            for algo in algo_names:
                row = [algo]
                for route in all_routes:
                    vals = [r.get("metrics", {}).get(key, 0)
                            for r in reports
                            if r.get("algorithm") == algo and r["_route"] == route]
                    if not vals:
                        row.append("\u2014")
                    elif len(vals) == 1:
                        row.append(_fmt_metric(vals[0], key))
                    else:
                        m, s = _mean_std(vals)
                        row.append(f"{m:.3f} \u00b1 {s:.3f}")
                mt.insert("", tk.END, values=row)

        metric_cb.bind("<<ComboboxSelected>>", _refresh)
        _refresh()

        ttk.Label(win,
                  text="Cells: single value, or mean \u00b1 std for multiple runs.",
                  foreground="#555").pack(pady=(0, 6))

    # ---- Legend / Help window -------------------------------------------

    def _show_legend(self, section="all"):
        win = tk.Toplevel(self)
        win.title("Metrics & Tests Legend")
        win.geometry("820x680")

        nb = ttk.Notebook(win)
        nb.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        def _tab(title, content_lines):
            frame = ttk.Frame(nb)
            nb.add(frame, text=title)
            try:
                leg_fam = UI_FONT_FAMILY or tkfont.nametofont("TkDefaultFont").actual()["family"]
            except tk.TclError:
                leg_fam = "sans-serif"
            text = tk.Text(
                frame, wrap=tk.WORD,
                font=(leg_fam, UI_LEGEND_BODY),
                padx=10, pady=8, spacing2=2)
            sc   = ttk.Scrollbar(frame, command=text.yview)
            text.config(yscrollcommand=sc.set)
            sc.pack(side=tk.RIGHT, fill=tk.Y)
            text.pack(fill=tk.BOTH, expand=True)

            text.tag_config("h1",    font=(leg_fam, UI_LEGEND_H1, "bold"), spacing1=10, spacing3=4)
            text.tag_config("h2",    font=(leg_fam, UI_LEGEND_H2, "bold"), spacing1=6)
            text.tag_config("good",  foreground="#2a7a2a")
            text.tag_config("info",  foreground="#2255aa")
            text.tag_config("code",  font=(UI_FONT_STATS[0], UI_LEGEND_CODE))
            text.tag_config("note",  foreground="#666666")

            for line in content_lines:
                if line.startswith("##"):
                    text.insert(tk.END, line[2:].strip() + "\n", "h1")
                elif line.startswith("#"):
                    text.insert(tk.END, line[1:].strip() + "\n", "h2")
                elif line.startswith(">>>"):
                    text.insert(tk.END, line[3:].strip() + "\n", "code")
                elif line.startswith("NOTE:"):
                    text.insert(tk.END, line + "\n", "note")
                else:
                    text.insert(tk.END, line + "\n")

            text.config(state=tk.DISABLED)
            return frame

        # ── Tab 1: Metrics ───────────────────────────────────────────────
        metric_lines = ["## Metric Definitions", ""]
        groups = [
            ("Path Quality Metrics", [
                "path_length_planned_m", "distance_traveled_m",
                "straight_line_dist_m",  "path_optimality_ratio",
                "path_smoothness_rad"]),
            ("Timing Metrics", [
                "planning_time_s", "execution_time_s", "total_time_s"]),
            ("Performance Metrics", [
                "completeness", "avg_velocity_ms", "energy_efficiency",
                "num_recoveries"]),
        ]
        for grp_name, keys in groups:
            metric_lines += [f"# {grp_name}", ""]
            for k in keys:
                lbl     = METRIC_LABELS.get(k, k)
                better  = METRIC_BETTER.get(k, "info")
                desc    = METRIC_DESCRIPTIONS.get(k, "(no description)")
                arrow   = "\u2193 lower = better" if better == "lower" else \
                          "\u2191 higher = better" if better == "higher" else "\u2192 informational"
                metric_lines += [
                    f"  [{lbl}]  —  {arrow}",
                    f"  Key: {k}",
                ]
                for dline in desc.split(". "):
                    if dline.strip():
                        metric_lines.append(f"  {dline.strip()}.")
                metric_lines.append("")
        _tab("Metrics", metric_lines)

        # ── Tab 2: Statistical Tests ──────────────────────────────────────
        stat_lines = ["## Statistical Tests Explained", ""]
        for name, desc in STAT_DESCRIPTIONS.items():
            stat_lines.append(f"# {name}")
            stat_lines.append("")
            for dl in desc.splitlines():
                stat_lines.append(f"  {dl}")
            stat_lines.append("")

        stat_lines += [
            "## Interpreting p-values", "",
            "  p < 0.05  →  result is statistically significant at the 5% level.",
            "               The observed difference is unlikely (< 5% chance) to be",
            "               due to random variation alone.",
            "",
            "  p ≥ 0.05  →  not significant.  We cannot reject the null hypothesis.",
            "               This does NOT prove the algorithms are identical —",
            "               it may just mean we need more data.",
            "",
            "  A significant p-value tells you the difference is REAL.",
            "  The effect size tells you whether it is MEANINGFUL.",
            "",
            "## Recommended workflow", "",
            "  1. Check success rate  —  filter out failed runs if needed.",
            "  2. Check Shapiro-Wilk  —  understand data distribution.",
            "  3. Run Kruskal-Wallis  —  is there any difference across algorithms?",
            "  4. If significant, run Mann-Whitney U pairwise comparisons.",
            "  5. Check effect size (r) to gauge practical importance.",
            "  6. Report: median ± IQR (not just mean ± std) for robustness.",
            "",
            "NOTE: With small n (< 10 per group) all tests have limited power.",
            "NOTE: Collect ≥ 10 repeated runs per algorithm per route for reliable results.",
        ]
        _tab("Statistical Tests", stat_lines)

        # ── Tab 3: Quick Reference ────────────────────────────────────────
        quick_lines = [
            "## Quick Reference Card", "",
            "# Column → Metric mapping",
            "",
        ]
        for k in METRIC_KEYS:
            lbl    = METRIC_LABELS[k]
            better = METRIC_BETTER[k]
            arrow  = "\u2193" if better == "lower" else "\u2191" if better == "higher" else "\u2194"
            quick_lines.append(f"  {lbl:<22s}  {arrow}  key: {k}")
        quick_lines += [
            "",
            "# Effect size thresholds (rank-biserial r)", "",
            "  |r| < 0.10   negligible — practically no difference",
            "  |r| 0.10–0.30  small",
            "  |r| 0.30–0.50  medium",
            "  |r| > 0.50   large — practically important",
            "",
            "# Coefficient of Variation (CV%)", "",
            "  CV% = std / mean × 100",
            "  CV% < 15%   low variability  — algorithm is consistent",
            "  CV% 15–30%  moderate variability",
            "  CV% > 30%   high variability — results vary run to run",
            "",
            "# Path Optimality Ratio", "",
            "  1.0  = perfect straight-line path",
            "  1.2  = path is 20% longer than straight line",
            "  Lower is always better (path is more direct)",
        ]
        _tab("Quick Reference", quick_lines)

        # Jump to stats tab if called from stats window
        if section == "stats":
            nb.select(1)

    # ---------------------------------------------------------------------------


def main():
    app = BenchmarkApp()
    app.mainloop()


if __name__ == "__main__":
    main()
