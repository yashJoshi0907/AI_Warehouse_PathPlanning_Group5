"""
gui.py
======
Interactive simulator & real-time animation for the warehouse delivery robot.

Features
--------
- Pick any of the four search algorithms and watch the planned route.
- Play / pause / single-step the robot with an adjustable speed.
- Click a cell to cycle it: open → shelf → crowded → trolley.
- Toggle the hidden-trolley mode: the robot detects the obstacle one cell
  ahead, updates its map and replans online with A*.
- Live telemetry: position, accumulated and planned cost, nodes, replans.
- Side-by-side comparison table for the four algorithms on the current map.

Run with:  python gui.py      (or: python main.py --gui)
"""

from __future__ import annotations
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Dict, List, Optional, Tuple, Set
import time

from warehouse_grid import (
    WarehouseGrid, make_default_grid, DEFAULT_LAYOUT,
    OPEN, CROWDED, SHELF, RESTRICTED, START, GOAL, TROLLEY, IMPASSABLE,
)
from search_algorithms import ALGORITHMS, astar, SearchResult
from trolley_replanning import TrolleySimulator

# ---------------------------------------------------------------------------
# Palette (shared with the figures so the app and the report feel related)
# ---------------------------------------------------------------------------
C = {
    'app_bg':     '#F2F4F9',
    'card':       '#FFFFFF',
    'ink':        '#1B1F2A',
    'muted':      '#6B7385',
    'rule':       '#DDE2EC',
    'accent':     '#2D6CDF',
    'accent_dark':'#2456B4',
    'success':    '#1FA97F',
    'warning':    '#E8A33D',
    'danger':     '#E0414A',
    'violet':     '#6C5CE7',
    'teal':       '#0CA678',

    'open':       '#EEF1F8',
    'crowded':    '#FFD9A0',
    'shelf':      '#2E3550',
    'restricted': '#F2A7B3',
    'start':      '#1FA97F',
    'goal':       '#2D6CDF',
    'trolley':    '#E0414A',
    'visited':    '#DCE6FF',
    'path':       '#6C5CE7',
    'replan':     '#E8590C',
    'robot':      '#2D6CDF',
}

CELL_COLOUR = {
    OPEN: C['open'],
    CROWDED: C['crowded'],
    SHELF: C['shelf'],
    RESTRICTED: C['restricted'],
    START: C['start'],
    GOAL: C['goal'],
    TROLLEY: C['trolley'],
}
CELL_LABEL = {START: 'S', GOAL: 'G', CROWDED: '4', TROLLEY: 'T'}
DARK_LABEL = {SHELF, RESTRICTED, TROLLEY, START, GOAL}


class WarehouseGUI:
    """Tkinter front-end for the warehouse search problem."""

    CANVAS = 600

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Warehouse Delivery Robot — Route Planning Simulator")
        self.root.geometry("1240x880")
        self.root.minsize(1160, 820)
        self.root.configure(bg=C['app_bg'])

        # Model state
        self.layout: List[str] = list(DEFAULT_LAYOUT)
        self.grid = WarehouseGrid(self.layout)
        self.current_algo = tk.StringVar(value='A*')
        self.speed_ms = tk.IntVar(value=160)
        self.trolley_check_var = tk.BooleanVar(value=False)

        # Animation state
        self.anim_running = False
        self.step_index = 0
        self.planned_path: List[Tuple[int, int]] = []
        self.visited_cells: Set[Tuple[int, int]] = set()
        self.accumulated_cost = 0.0
        self.replan_count = 0
        self.trolley_mode = False
        self.trolley_pos: Optional[Tuple[int, int]] = None
        self.sim_result = None
        self.status = 'ready'

        self._setup_styles()
        self._build_header()
        self._build_body()

        self._bind_shortcuts()
        self._compute_search()
        self._draw_grid()

    # ===================================================================
    # Styling helpers
    # ===================================================================

    def _setup_styles(self) -> None:
        style = ttk.Style()
        style.theme_use('clam')

        style.configure('TCombobox', fieldbackground=C['card'], background=C['card'],
                        foreground=C['ink'], arrowcolor=C['muted'], borderwidth=1,
                        relief='flat', padding=4)
        style.map('TCombobox', fieldbackground=[('readonly', C['card'])])
        style.configure('Tiny.TCheckbutton', background=C['card'], foreground=C['ink'])

        # Comparison dialog table
        style.configure('Compare.Treeview', background=C['card'], fieldbackground=C['card'],
                        foreground=C['ink'], rowheight=30, borderwidth=0, font=('Segoe UI', 10))
        style.configure('Compare.Treeview.Heading', background=C['app_bg'],
                        foreground=C['muted'], font=('Segoe UI', 8, 'bold'), relief='flat')
        style.map('Compare.Treeview', background=[('selected', C['visited'])])
        style.layout('Compare.Treeview', [('Compare.Treeview.treearea', {'sticky': 'nswe'})])

    def _card(self, parent: tk.Widget, **kw) -> tk.Frame:
        frame = tk.Frame(parent, bg=C['card'], highlightthickness=1,
                         highlightbackground=C['rule'], **kw)
        return frame

    def _button(self, parent: tk.Widget, text: str, command,
                colour: str = C['accent'], wide: bool = False) -> tk.Button:
        btn = tk.Button(parent, text=text, command=command, bg=colour, fg='#FFFFFF',
                        activebackground=self._shade(colour, 0.85), activeforeground='#FFFFFF',
                        font=('Segoe UI', 9, 'bold'), relief='flat', bd=0,
                        padx=14, pady=8, cursor='hand2', highlightthickness=0)
        btn.bind('<Enter>', lambda _e: btn.config(bg=self._shade(colour, 0.9)))
        btn.bind('<Leave>', lambda _e: btn.config(bg=colour))
        if wide:
            btn.pack(fill=tk.X, pady=(0, 0))
        return btn

    @staticmethod
    def _shade(hex_colour: str, factor: float) -> str:
        """Darken a #rrggbb colour by *factor* (0..1)."""
        hex_colour = hex_colour.lstrip('#')
        r, g, b = (int(hex_colour[i:i + 2], 16) for i in (0, 2, 4))
        return '#{:02x}{:02x}{:02x}'.format(int(r * factor), int(g * factor), int(b * factor))

    def _rounded(self, x1: float, y1: float, x2: float, y2: float,
                 radius: float, **kw):
        """Draw a rounded rectangle on the canvas and return its item id."""
        r = min(radius, (x2 - x1) / 2, (y2 - y1) / 2)
        points = [
            x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
            x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
            x1, y2, x1, y2 - r, x1, y1 + r, x1, y1,
        ]
        return self.canvas.create_polygon(points, smooth=True, **kw)

    # ===================================================================
    # Layout
    # ===================================================================

    def _build_header(self) -> None:
        header = tk.Frame(self.root, bg=C['app_bg'], padx=22, pady=16)
        header.pack(side=tk.TOP, fill=tk.X)

        left = tk.Frame(header, bg=C['app_bg'])
        left.pack(side=tk.LEFT)

        tk.Label(left, text="Warehouse Delivery Robot", font=('Segoe UI', 17, 'bold'),
                 fg=C['ink'], bg=C['app_bg']).pack(anchor='w')
        tk.Label(left, text="Route planning and online replanning with BFS, DFS, "
                            "Greedy Best-First and A*",
                 font=('Segoe UI', 9), fg=C['muted'], bg=C['app_bg']).pack(anchor='w')

        tk.Label(header, text="Group 5  ·  BT24CSD024 / 028 / 029 / 031 / 032 / 033",
                 font=('Segoe UI', 9), fg=C['muted'], bg=C['app_bg']).pack(side=tk.RIGHT, pady=8)

        tk.Frame(self.root, bg=C['rule'], height=1).pack(fill=tk.X, padx=22)

    def _build_body(self) -> None:
        body = tk.Frame(self.root, bg=C['app_bg'], padx=22, pady=14)
        body.pack(fill=tk.BOTH, expand=True)

        # ---- left: map card -------------------------------------------
        map_card = self._card(body)
        map_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        inner = tk.Frame(map_card, bg=C['card'], padx=16, pady=14)
        inner.pack(fill=tk.BOTH, expand=True)

        title_row = tk.Frame(inner, bg=C['card'])
        title_row.pack(fill=tk.X)
        tk.Label(title_row, text="Warehouse map", font=('Segoe UI', 11, 'bold'),
                 fg=C['ink'], bg=C['card']).pack(side=tk.LEFT)
        tk.Label(title_row, text="click a cell to cycle: open → shelf → crowded → trolley",
                 font=('Segoe UI', 8), fg=C['muted'], bg=C['card']).pack(side=tk.RIGHT)

        self._build_legend(inner)

        self.canvas_size = self.CANVAS
        self.canvas = tk.Canvas(inner, width=self.CANVAS, height=self.CANVAS,
                                bg=C['card'], highlightthickness=0)
        self.canvas.pack(pady=(10, 4))
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        self.lbl_hint = tk.Label(
            inner, text="Press Space to run, → to step, R to reset.",
            font=('Segoe UI', 8), fg=C['muted'], bg=C['card'])
        self.lbl_hint.pack(anchor='w')

        # ---- right: controls & telemetry ------------------------------
        side = tk.Frame(body, bg=C['app_bg'], width=380)
        side.pack(side=tk.RIGHT, fill=tk.Y, padx=(16, 0))
        side.pack_propagate(False)

        self._build_controls(side)
        self._build_telemetry(side)

    def _build_legend(self, parent: tk.Widget) -> None:
        bar = tk.Frame(parent, bg=C['card'])
        bar.pack(fill=tk.X, pady=(10, 0))
        items = [
            ("Start", C['start'], '#FFFFFF'),
            ("Goal", C['goal'], '#FFFFFF'),
            ("Open · 1", C['open'], C['ink']),
            ("Crowded · 4", C['crowded'], '#7A4500'),
            ("Shelf", C['shelf'], '#FFFFFF'),
            ("Restricted", C['restricted'], '#7A1F2B'),
            ("Trolley", C['trolley'], '#FFFFFF'),
        ]
        for name, colour, text_colour in items:
            chip = tk.Label(bar, text=f" {name} ", bg=colour, fg=text_colour,
                            font=('Segoe UI', 8, 'bold'), padx=4, pady=2)
            chip.pack(side=tk.LEFT, padx=(0, 6))

    def _build_controls(self, parent: tk.Widget) -> None:
        card = self._card(parent)
        card.pack(fill=tk.X)
        pad = tk.Frame(card, bg=C['card'], padx=14, pady=14)
        pad.pack(fill=tk.X)

        tk.Label(pad, text="PLAN", font=('Segoe UI', 8, 'bold'),
                 fg=C['muted'], bg=C['card']).pack(anchor='w')

        algo_row = tk.Frame(pad, bg=C['card'])
        algo_row.pack(fill=tk.X, pady=(6, 10))
        tk.Label(algo_row, text="Algorithm", font=('Segoe UI', 9),
                 fg=C['ink'], bg=C['card']).pack(side=tk.LEFT)
        combo = ttk.Combobox(algo_row, textvariable=self.current_algo,
                             values=list(ALGORITHMS.keys()), state='readonly', width=14)
        combo.pack(side=tk.RIGHT)
        combo.bind("<<ComboboxSelected>>", self._on_algo_change)

        chk = tk.Checkbutton(
            pad, text="Hidden trolley obstacle (auto-replan)",
            variable=self.trolley_check_var, command=self._on_trolley_check_toggle,
            bg=C['card'], fg=C['ink'], activebackground=C['card'],
            activeforeground=C['ink'], selectcolor=C['card'], bd=0,
            highlightthickness=0, font=('Segoe UI', 9), anchor='w')
        chk.pack(fill=tk.X, pady=(0, 10))

        buttons = tk.Frame(pad, bg=C['card'])
        buttons.pack(fill=tk.X)
        self.btn_run = self._button(buttons, "▶  Run", self._toggle_run, C['success'])
        self.btn_run.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self.btn_step = self._button(buttons, "⏭  Step", self._step_once, C['accent'])
        self.btn_step.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        self.btn_reset = self._button(buttons, "↺", self._reset_sim, C['warning'])
        self.btn_reset.configure(width=3, padx=8)
        self.btn_reset.pack(side=tk.LEFT, padx=(4, 0))

        speed = tk.Frame(pad, bg=C['card'])
        speed.pack(fill=tk.X, pady=(12, 6))
        tk.Label(speed, text="Speed", font=('Segoe UI', 8),
                 fg=C['muted'], bg=C['card']).pack(side=tk.LEFT)
        slider = tk.Scale(speed, from_=40, to=500, orient=tk.HORIZONTAL,
                          variable=self.speed_ms, showvalue=False, length=200,
                          bg=C['card'], highlightthickness=0, bd=0,
                          troughcolor=C['open'], activebackground=C['accent'])
        slider.pack(side=tk.RIGHT)

        self._button(pad, "Compare all four algorithms", self._open_comparison_dialog,
                     C['violet'], wide=True).pack(fill=tk.X, pady=(6, 0))
        self._button(pad, "Restore default warehouse", self._reset_grid_layout,
                     C['muted'], wide=True).pack(fill=tk.X, pady=(6, 0))

    def _build_telemetry(self, parent: tk.Widget) -> None:
        card = self._card(parent)
        card.pack(fill=tk.X, pady=(14, 0))
        pad = tk.Frame(card, bg=C['card'], padx=14, pady=14)
        pad.pack(fill=tk.X)

        tk.Label(pad, text="LIVE TELEMETRY", font=('Segoe UI', 8, 'bold'),
                 fg=C['muted'], bg=C['card']).pack(anchor='w')

        self.lbl_status = tk.Label(pad, text="Ready", font=('Segoe UI', 9, 'bold'),
                                   fg='#FFFFFF', bg=C['accent'], padx=10, pady=5)
        self.lbl_status.pack(fill=tk.X, pady=(8, 12))

        grid = tk.Frame(pad, bg=C['card'])
        grid.pack(fill=tk.X)

        cards = [
            ("Robot", "lbl_pos", str(self.grid.start)),
            ("Goal", "lbl_goal", str(self.grid.goal)),
            ("Remaining moves", "lbl_moves", "0"),
            ("Cost so far", "lbl_cost", "0.0"),
            ("Plan cost", "lbl_total_cost", "0.0"),
            ("Nodes expanded", "lbl_nodes", "0"),
            ("Search time", "lbl_time", "0.000 ms"),
            ("Replans", "lbl_replans", "0"),
        ]
        self.labels: Dict[str, tk.Label] = {}
        for i, (title, key, value) in enumerate(cards):
            row, col = divmod(i, 2)
            cell = tk.Frame(grid, bg=C['card'], highlightthickness=1,
                            highlightbackground=C['rule'])
            cell.grid(row=row, column=col, padx=(0, 6), pady=(0, 6), sticky='nsew')
            grid.grid_columnconfigure(col, weight=1)
            tk.Label(cell, text=title.upper(), font=('Segoe UI', 7), fg=C['muted'],
                     bg=C['card']).pack(anchor='w', padx=8, pady=(6, 0))
            value_lbl = tk.Label(cell, text=value, font=('Segoe UI', 13, 'bold'),
                                 fg=C['ink'], bg=C['card'])
            value_lbl.pack(anchor='w', padx=8, pady=(0, 6))
            self.labels[key] = value_lbl

        self.replan_info = tk.Label(pad, text="No trolley on the map.",
                                    font=('Segoe UI', 8), fg=C['muted'], bg=C['card'],
                                    wraplength=320, justify='left', anchor='w')
        self.replan_info.pack(fill=tk.X, pady=(10, 0))

    def _bind_shortcuts(self) -> None:
        self.root.bind('<space>', lambda _e: self._toggle_run())
        self.root.bind('<Right>', lambda _e: self._step_once())
        self.root.bind('<r>', lambda _e: self._reset_sim())
        self.root.bind('<R>', lambda _e: self._reset_sim())

    # ===================================================================
    # Search
    # ===================================================================

    def _set_status(self, text: str, colour: str) -> None:
        self.status = text
        self.lbl_status.config(text=text, bg=colour)

    def _compute_search(self) -> None:
        if self.trolley_check_var.get():
            self.trolley_mode = True
            try:
                sim = TrolleySimulator(self.grid)
                self.sim_result = sim.run()
                self.planned_path = list(sim.initial_path)
                self.trolley_pos = sim.trolley_positions[0] if sim.trolley_positions else None
                search_res = sim.initial_result
            except Exception:
                self.sim_result = None
                self.trolley_pos = None
                self.planned_path = []
                search_res = SearchResult('Trolley', None, 0, 0.0, 0.0)
        else:
            self.trolley_mode = False
            self.trolley_pos = None
            self.sim_result = None
            fn = ALGORITHMS.get(self.current_algo.get(), astar)
            search_res = fn(self.grid)
            self.planned_path = list(search_res.path or [])

        self.step_index = 0
        self.accumulated_cost = 0.0
        self.replan_count = 0
        self.visited_cells.clear()

        self.labels['lbl_pos'].config(text=str(self.grid.start))
        self.labels['lbl_goal'].config(text=str(self.grid.goal))
        self.labels['lbl_moves'].config(text=str(max(0, len(self.planned_path) - 1)))
        self.labels['lbl_cost'].config(text="0.0")
        self.labels['lbl_total_cost'].config(
            text=f"{search_res.path_cost:.1f}" if search_res.goal_reached else "—")
        self.labels['lbl_nodes'].config(text=str(search_res.nodes_expanded))
        self.labels['lbl_time'].config(text=f"{search_res.elapsed_sec * 1000:.2f} ms")
        self.labels['lbl_replans'].config(text="0")

        if self.trolley_mode and self.trolley_pos:
            self.replan_info.config(
                text=f"Hidden trolley at {self.trolley_pos} — unknown until the sensor sees it.")
        else:
            self.replan_info.config(text="No trolley on the map.")

        if not self.planned_path:
            self._set_status("No route to the goal", C['danger'])
        elif self.trolley_mode:
            self._set_status(
                f"Plan ready · {len(self.planned_path) - 1} moves · hidden obstacle armed",
                C['warning'])
        else:
            self._set_status(
                f"Plan ready · {len(self.planned_path) - 1} moves", C['success'])

    # ===================================================================
    # Rendering
    # ===================================================================

    def _draw_grid(self) -> None:
        self.canvas.delete('all')
        rows, cols = self.grid.rows, self.grid.cols
        pad = 10
        size = (self.canvas_size - 2 * pad) / max(rows, cols)
        self.cell_px = size
        self.pad = pad
        radius = size * 0.18
        gap = max(1.5, size * 0.045)

        for r in range(rows):
            for c in range(cols):
                sym = self.grid.cell(r, c)
                x1 = pad + c * size + gap
                y1 = pad + r * size + gap
                x2 = pad + (c + 1) * size - gap
                y2 = pad + (r + 1) * size - gap

                colour = CELL_COLOUR.get(sym, C['open'])
                if (r, c) in self.visited_cells and sym == OPEN:
                    colour = C['visited']
                self._rounded(x1, y1, x2, y2, radius, fill=colour, outline='')

                label = CELL_LABEL.get(sym, '')
                if label:
                    fill = '#FFFFFF' if sym in DARK_LABEL else '#7A4500'
                    self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text=label,
                                            font=('Segoe UI', int(size * 0.30), 'bold'),
                                            fill=fill)

        # Route under the tiles drawn as a soft line + solid core
        if len(self.planned_path) > 1:
            travelled = self.planned_path[:self.step_index + 1]
            remaining = self.planned_path[self.step_index:]

            def coords(points):
                out = []
                for r, c in points:
                    out += [pad + (c + 0.5) * size, pad + (r + 0.5) * size]
                return out

            route_colour = C['replan'] if self.replan_count else C['path']

            if len(travelled) > 1:
                self.canvas.create_line(*coords(travelled), fill=C['teal'],
                                        width=max(3, size * 0.10), capstyle='round',
                                        joinstyle='round', smooth=False)
            if len(remaining) > 1:
                self.canvas.create_line(*coords(remaining), fill='#E4E1FB',
                                        width=max(6, size * 0.22), capstyle='round',
                                        joinstyle='round')
                self.canvas.create_line(*coords(remaining), fill=route_colour,
                                        width=max(2, size * 0.075), capstyle='round',
                                        joinstyle='round', dash=(4, 3))

            for r, c in self.planned_path[1:-1]:
                cx, cy = pad + (c + 0.5) * size, pad + (r + 0.5) * size
                self.canvas.create_oval(cx - 2.2, cy - 2.2, cx + 2.2, cy + 2.2,
                                        fill=C['card'], outline='')

        # Hidden trolley marker
        if self.trolley_mode and self.trolley_pos:
            tr, tc = self.trolley_pos
            x1 = pad + tc * size + gap
            y1 = pad + tr * size + gap
            x2 = pad + (tc + 1) * size - gap
            y2 = pad + (tr + 1) * size - gap
            self._rounded(x1, y1, x2, y2, radius, outline=C['danger'], width=2, fill='')
            self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text="!",
                                    font=('Segoe UI', int(size * 0.34), 'bold'),
                                    fill=C['danger'])

        # Robot
        if self.planned_path and self.step_index < len(self.planned_path):
            rr, rc = self.planned_path[self.step_index]
            cx = pad + (rc + 0.5) * size
            cy = pad + (rr + 0.5) * size
            rad = size * 0.30
            self.canvas.create_oval(cx - rad - 4, cy - rad - 4, cx + rad + 4, cy + rad + 4,
                                    outline='#DCE6FF', width=3)
            self.canvas.create_oval(cx - rad, cy - rad, cx + rad, cy + rad,
                                    fill=C['robot'], outline='#FFFFFF', width=2)
            eye = max(1.6, rad * 0.22)
            for dx in (-rad * 0.34, rad * 0.34):
                self.canvas.create_oval(cx + dx - eye, cy - eye * 0.4,
                                        cx + dx + eye, cy + eye * 1.6,
                                        fill='#FFFFFF', outline='')

    # ===================================================================
    # Interaction
    # ===================================================================

    def _on_canvas_click(self, event: tk.Event) -> None:
        if self.anim_running:
            return
        size = getattr(self, 'cell_px', self.canvas_size / self.grid.cols)
        pad = getattr(self, 'pad', 0)
        col = int((event.x - pad) // size)
        row = int((event.y - pad) // size)
        if not self.grid.in_bounds(row, col):
            return

        current = self.grid.cell(row, col)
        if current in (START, GOAL):
            return

        cycle = {OPEN: SHELF, SHELF: CROWDED, CROWDED: TROLLEY, TROLLEY: OPEN}
        new_sym = cycle.get(current, OPEN)

        self.grid._grid[row][col] = new_sym
        row_chars = list(self.layout[row])
        row_chars[col] = new_sym
        self.layout[row] = ''.join(row_chars)
        self._reset_sim()

    def _toggle_run(self) -> None:
        if self.anim_running:
            self.anim_running = False
            self.btn_run.config(text="▶  Resume")
            self._set_status("Paused", C['warning'])
            return

        if not self.planned_path:
            messagebox.showwarning("No route", "No valid route is available on this map.")
            return
        if self.step_index >= len(self.planned_path) - 1:
            self._reset_sim()

        self.anim_running = True
        self.btn_run.config(text="⏸  Pause")
        self._set_status("Robot moving…", C['accent'])
        self._animate()

    def _animate(self) -> None:
        if not self.anim_running:
            return
        if self._step_forward():
            self.root.after(max(20, self.speed_ms.get()), self._animate)
        else:
            self.anim_running = False
            self.btn_run.config(text="▶  Run")

    def _step_once(self) -> None:
        if self.anim_running:
            self.anim_running = False
            self.btn_run.config(text="▶  Run")
        self._step_forward()

    def _step_forward(self) -> bool:
        if not self.planned_path or self.step_index >= len(self.planned_path) - 1:
            if self.planned_path:
                self._set_status("Destination reached", C['success'])
            return False

        next_pos = self.planned_path[self.step_index + 1]

        if self.trolley_mode and self.trolley_pos and next_pos == self.trolley_pos:
            current_pos = self.planned_path[self.step_index]
            self._set_status(f"Obstacle detected at {next_pos}", C['danger'])
            self.grid.mark_impassable(*self.trolley_pos)
            self.replan_count += 1
            self.labels['lbl_replans'].config(text=str(self.replan_count))

            replan = astar(self.grid, start_override=current_pos)
            if replan.path:
                self.planned_path = list(replan.path)
                self.step_index = 0
                self.trolley_pos = None
                self.replan_info.config(
                    text=f"Replanned from {current_pos}: {len(replan.path) - 1} moves, "
                         f"cost {replan.path_cost:.1f} ({replan.nodes_expanded} nodes).")
                self.labels['lbl_total_cost'].config(text=f"{replan.path_cost:.1f}")
                self.labels['lbl_moves'].config(text=f"{len(replan.path) - 1} (remaining)")
                self.labels['lbl_nodes'].config(text=str(replan.nodes_expanded))
                self.labels['lbl_time'].config(text=f"{replan.elapsed_sec * 1000:.2f} ms")
                self._set_status("Map updated · rerouted with A*", C['replan'])
                self._draw_grid()
                return True

            self._set_status("No alternative route — destination unreachable", C['danger'])
            self.replan_info.config(text="The robot halts instead of entering the blocked cell.")
            self.anim_running = False
            self.btn_run.config(text="▶  Run")
            self._draw_grid()
            return False

        self.step_index += 1
        current = self.planned_path[self.step_index]
        self.visited_cells.add(current)
        self.accumulated_cost += self.grid.entry_cost(*current)
        self.labels['lbl_pos'].config(text=str(current))
        self.labels['lbl_cost'].config(text=f"{self.accumulated_cost:.1f}")
        self.labels['lbl_moves'].config(
            text=f"{max(0, len(self.planned_path) - 1 - self.step_index)}")
        self._draw_grid()

        if current == self.grid.goal:
            self._set_status("Delivered — goal reached", C['success'])
            return False
        return True

    def _reset_sim(self) -> None:
        self.anim_running = False
        self.btn_run.config(text="▶  Run")
        # Rebuild from the pristine layout so trolleys detected in an earlier
        # run do not stay marked on the map.
        self.grid = WarehouseGrid(self.layout)
        self._compute_search()
        self._draw_grid()

    def _reset_grid_layout(self) -> None:
        self.layout = list(DEFAULT_LAYOUT)
        self._reset_sim()

    def _on_algo_change(self, event=None) -> None:
        self._reset_sim()

    def _on_trolley_check_toggle(self) -> None:
        self._reset_sim()

    # ===================================================================
    # Comparison dialog
    # ===================================================================

    def _open_comparison_dialog(self) -> None:
        dlg = tk.Toplevel(self.root)
        dlg.title("Algorithm comparison")
        dlg.geometry("720x420")
        dlg.configure(bg=C['app_bg'])
        dlg.transient(self.root)
        dlg.grab_set()

        tk.Label(dlg, text="Four algorithms on the current map",
                 font=('Segoe UI', 13, 'bold'), fg=C['ink'],
                 bg=C['app_bg']).pack(anchor='w', padx=18, pady=(16, 2))
        tk.Label(dlg, text="Same start, same goal, same obstacles — only the strategy changes.",
                 font=('Segoe UI', 9), fg=C['muted'], bg=C['app_bg']).pack(anchor='w', padx=18)

        holder = tk.Frame(dlg, bg=C['card'], highlightthickness=1,
                          highlightbackground=C['rule'])
        holder.pack(fill=tk.BOTH, expand=True, padx=18, pady=14)

        columns = ("Algorithm", "Goal", "Moves", "Cost", "Nodes", "Time (ms)")
        tree = ttk.Treeview(holder, columns=columns, show='headings', height=6,
                            style='Compare.Treeview')
        widths = (190, 70, 80, 90, 90, 110)
        for col, width in zip(columns, widths):
            tree.heading(col, text=col.upper())
            tree.column(col, anchor=tk.CENTER, width=width)
        tree.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        for fn in ALGORITHMS.values():
            res: SearchResult = fn(self.grid)
            tree.insert('', tk.END, values=(
                res.algorithm,
                "Yes" if res.goal_reached else "No",
                res.path_length if res.goal_reached else "—",
                f"{res.path_cost:.1f}" if res.goal_reached else "—",
                res.nodes_expanded,
                f"{res.elapsed_sec * 1000:.3f}",
            ))

        notes = ("A* is the only algorithm that guarantees the lowest-cost route. "
                 "BFS guarantees the fewest moves, Greedy is the cheapest in work but "
                 "unproven, and DFS simply goes deep.")
        tk.Label(dlg, text=notes, font=('Segoe UI', 8), fg=C['muted'], bg=C['app_bg'],
                 wraplength=670, justify='left').pack(anchor='w', padx=18)

        self._button(dlg, "Close", dlg.destroy, C['muted']).pack(pady=12)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    root = tk.Tk()
    WarehouseGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
