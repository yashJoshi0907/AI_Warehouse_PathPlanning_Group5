"""
gui.py
======
Interactive GUI & Real-Time Animation for Warehouse Delivery Robot
Author   : Sayan Majhi (BT24CSD028) & Dev Goyal (BT24CSD029)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot

Features:
- Full graphical interface with interactive warehouse grid.
- Visual animation of robot traversal, step-by-step search, and real-time replanning.
- Interactive obstacle placement: click any grid cell to place/remove Shelves, Crowded cells, or Trolleys.
- Live HUD tracking position, step count, accumulated path cost, nodes expanded, and replans.
- Speed control slider, Play/Pause/Step/Reset controls.
- Dynamic Trolley Obstacle simulation with sensor lookahead and live A* replanning.
- Multi-algorithm comparison dialog with real-time metrics.
"""

from __future__ import annotations
import tkinter as tk
from tkinter import ttk, messagebox
import time
from typing import List, Tuple, Optional, Dict, Set

from warehouse_grid import (
    WarehouseGrid, make_default_grid, DEFAULT_LAYOUT,
    OPEN, CROWDED, SHELF, RESTRICTED, START, GOAL, TROLLEY,
    CELL_COST, IMPASSABLE
)
from search_algorithms import bfs, dfs, greedy, astar, SearchResult, manhattan
from trolley_replanning import TrolleySimulator, ReplanEvent

# ---------------------------------------------------------------------------
# Color Palette
# ---------------------------------------------------------------------------
COLORS = {
    'bg':             '#1E1E2E',  # Dark modern background
    'panel_bg':       '#252538',  # Side panel background
    'card_bg':        '#2D2D44',  # Metric cards
    'text':           '#CDD6F4',  # Off-white primary text
    'subtext':        '#A6ADC8',  # Gray secondary text
    'accent':         '#89B4FA',  # Soft blue
    'border':         '#45475A',  # Border line
    
    # Grid Cell Colors
    'cell_open':      '#FFFFFF',  # Open aisle
    'cell_crowded':   '#FAB387',  # Amber/peach crowded (cost 4)
    'cell_shelf':     '#313244',  # Dark slate shelf
    'cell_restricted':'#F38BA8',  # Soft red restricted
    'cell_start':     '#A6E3A1',  # Soft green start
    'cell_goal':      '#89B4FA',  # Soft blue goal
    'cell_trolley':   '#E78284',  # Crimson trolley
    
    # Animation Highlights
    'path_line':      '#CBA6F7',  # Soft purple planned path
    'robot':          '#F9E2AF',  # Golden yellow robot
    'robot_glow':     '#F5C2E7',  # Pink glow
    'visited':        '#45475A',  # Explored cells
    'replan_path':    '#94E2D5',  # Teal replanned path
    'detected':       '#E78284',  # Alert red
}


class WarehouseGUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("AI Warehouse Delivery Robot — Route Planning & Replanning")
        self.root.geometry("1100x750")
        self.root.minsize(980, 680)
        self.root.configure(bg=COLORS['bg'])

        # Data state
        self.layout = [row for row in DEFAULT_LAYOUT]
        self.grid = WarehouseGrid(self.layout)
        self.current_algo = tk.StringVar(value="A* Search")
        self.speed_ms = tk.IntVar(value=180)  # milliseconds per step

        # Animation state
        self.anim_running = False
        self.anim_paused = False
        self.step_index = 0
        self.planned_path: List[Tuple[int, int]] = []
        self.visited_cells: Set[Tuple[int, int]] = set()
        self.accumulated_cost = 0.0
        self.replan_count = 0
        self.trolley_mode = False
        self.trolley_pos: Optional[Tuple[int, int]] = None
        self.sim_result = None

        # Build UI layout
        self._setup_styles()
        self._create_widgets()
        
        # Initial search run
        self._compute_search()
        self._draw_grid()

    # -----------------------------------------------------------------------
    # Setup & Layout
    # -----------------------------------------------------------------------
    def _setup_styles(self) -> None:
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background=COLORS['bg'])
        style.configure('Panel.TFrame', background=COLORS['panel_bg'])
        style.configure('Card.TFrame', background=COLORS['card_bg'])
        style.configure('TLabel', background=COLORS['card_bg'], foreground=COLORS['text'], font=('Segoe UI', 10))
        style.configure('Header.TLabel', background=COLORS['panel_bg'], foreground=COLORS['accent'], font=('Segoe UI', 12, 'bold'))
        style.configure('Title.TLabel', background=COLORS['bg'], foreground=COLORS['text'], font=('Segoe UI', 15, 'bold'))
        style.configure('Sub.TLabel', background=COLORS['card_bg'], foreground=COLORS['subtext'], font=('Segoe UI', 8))
        style.configure('Value.TLabel', background=COLORS['card_bg'], foreground='#FFFFFF', font=('Segoe UI', 12, 'bold'))
        
        style.configure('Action.TButton', font=('Segoe UI', 9, 'bold'), padding=5)
        style.map('Action.TButton',
                  background=[('active', COLORS['accent']), ('!disabled', COLORS['card_bg'])],
                  foreground=[('!disabled', '#FFFFFF')])

    def _create_widgets(self) -> None:
        # Top banner
        top_bar = tk.Frame(self.root, bg=COLORS['bg'], pady=8, padx=15)
        top_bar.pack(side=tk.TOP, fill=tk.X)
        
        lbl_title = tk.Label(top_bar, text="AI Warehouse Delivery Robot — Route Planning Simulator",
                             font=('Segoe UI', 14, 'bold'), fg=COLORS['text'], bg=COLORS['bg'])
        lbl_title.pack(side=tk.LEFT)
        lbl_sub = tk.Label(top_bar, text="  Group 5 (BT24CSD024, 028, 029, 031, 032, 033)",
                           font=('Segoe UI', 9), fg=COLORS['subtext'], bg=COLORS['bg'])
        lbl_sub.pack(side=tk.LEFT, pady=3)

        # Main horizontal split
        main_body = tk.Frame(self.root, bg=COLORS['bg'])
        main_body.pack(fill=tk.BOTH, expand=True, padx=12, pady=6)

        # Left panel: Canvas (Grid)
        left_box = tk.Frame(main_body, bg=COLORS['bg'])
        left_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Legend Bar
        self._create_legend(left_box)

        # Grid Canvas
        self.canvas_size = 520
        self.canvas = tk.Canvas(left_box, width=self.canvas_size, height=self.canvas_size,
                               bg=COLORS['panel_bg'], highlightthickness=1, highlightbackground=COLORS['border'])
        self.canvas.pack(pady=6)
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        # Click instruction label
        lbl_click = tk.Label(left_box, text="💡 Tip: Click any cell to cycle: [Open .] → [Shelf #] → [Crowded C] → [Trolley T]",
                             font=('Segoe UI', 8, 'italic'), fg=COLORS['subtext'], bg=COLORS['bg'])
        lbl_click.pack(pady=2)

        # Right panel: Controls & Metrics HUD
        right_panel = tk.Frame(main_body, bg=COLORS['panel_bg'], width=380, padx=14, pady=12,
                               highlightthickness=1, highlightbackground=COLORS['border'])
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(10, 0))
        right_panel.pack_propagate(False)

        self._create_controls(right_panel)
        self._create_metrics_hud(right_panel)

    def _create_legend(self, parent: tk.Widget) -> None:
        bar = tk.Frame(parent, bg=COLORS['bg'])
        bar.pack(fill=tk.X, pady=(0, 4))
        
        items = [
            ("Start (S)", COLORS['cell_start'], '#222'),
            ("Goal (G)", COLORS['cell_goal'], '#222'),
            ("Open (1)", COLORS['cell_open'], '#222'),
            ("Crowded (4)", COLORS['cell_crowded'], '#222'),
            ("Shelf (#)", COLORS['cell_shelf'], '#FFF'),
            ("Trolley (T)", COLORS['cell_trolley'], '#FFF'),
            ("Robot", COLORS['robot'], '#222'),
        ]
        for name, col, text_col in items:
            box = tk.Label(bar, text=f" {name} ", bg=col, fg=text_col,
                           font=('Segoe UI', 8, 'bold'), relief=tk.RIDGE, bd=1)
            box.pack(side=tk.LEFT, padx=3)

    def _create_controls(self, parent: tk.Widget) -> None:
        lbl = ttk.Label(parent, text="ALGORITHM & CONTROL", style='Header.TLabel')
        lbl.pack(anchor=tk.W, pady=(0, 6))

        # Algorithm selection dropdown
        algo_frame = tk.Frame(parent, bg=COLORS['panel_bg'])
        algo_frame.pack(fill=tk.X, pady=3)

        tk.Label(algo_frame, text="Algorithm:", font=('Segoe UI', 9),
                 fg=COLORS['text'], bg=COLORS['panel_bg']).pack(side=tk.LEFT)
        
        algos = ["A* Search", "BFS (Breadth-First)", "Greedy Best-First", "DFS (Depth-First)"]
        combo = ttk.Combobox(algo_frame, textvariable=self.current_algo, values=algos, state="readonly", width=18)
        combo.pack(side=tk.RIGHT)
        combo.bind("<<ComboboxSelected>>", self._on_algo_change)

        # Trolley simulation checkbox / mode
        mode_frame = tk.Frame(parent, bg=COLORS['panel_bg'])
        mode_frame.pack(fill=tk.X, pady=6)
        
        self.trolley_check_var = tk.BooleanVar(value=False)
        chk = tk.Checkbutton(mode_frame, text="Simulate Dynamic Trolley (Auto-Obstacle)",
                             variable=self.trolley_check_var, bg=COLORS['panel_bg'], fg=COLORS['accent'],
                             activebackground=COLORS['panel_bg'], activeforeground=COLORS['accent'],
                             selectcolor=COLORS['card_bg'], font=('Segoe UI', 9, 'bold'),
                             command=self._on_trolley_check_toggle)
        chk.pack(anchor=tk.W)

        # Playback buttons
        btn_frame = tk.Frame(parent, bg=COLORS['panel_bg'])
        btn_frame.pack(fill=tk.X, pady=6)

        self.btn_run = tk.Button(btn_frame, text="▶ Run", bg='#27AE60', fg='white', font=('Segoe UI', 9, 'bold'),
                                 relief=tk.FLAT, padx=10, pady=4, cursor='hand2', command=self._toggle_run)
        self.btn_run.pack(side=tk.LEFT, padx=2)

        self.btn_step = tk.Button(btn_frame, text="⏭ Step", bg='#2980B9', fg='white', font=('Segoe UI', 9, 'bold'),
                                  relief=tk.FLAT, padx=8, pady=4, cursor='hand2', command=self._step_once)
        self.btn_step.pack(side=tk.LEFT, padx=2)

        self.btn_reset = tk.Button(btn_frame, text="🔄 Reset", bg='#E67E22', fg='white', font=('Segoe UI', 9, 'bold'),
                                   relief=tk.FLAT, padx=8, pady=4, cursor='hand2', command=self._reset_sim)
        self.btn_reset.pack(side=tk.LEFT, padx=2)

        self.btn_clear = tk.Button(btn_frame, text="🧹 Default", bg='#7F8C8D', fg='white', font=('Segoe UI', 9, 'bold'),
                                   relief=tk.FLAT, padx=6, pady=4, cursor='hand2', command=self._reset_grid_layout)
        self.btn_clear.pack(side=tk.LEFT, padx=2)

        # Speed slider
        speed_frame = tk.Frame(parent, bg=COLORS['panel_bg'])
        speed_frame.pack(fill=tk.X, pady=4)
        tk.Label(speed_frame, text="Anim Speed:", font=('Segoe UI', 8),
                 fg=COLORS['subtext'], bg=COLORS['panel_bg']).pack(side=tk.LEFT)
        scale = tk.Scale(speed_frame, from_=50, to=500, orient=tk.HORIZONTAL,
                         variable=self.speed_ms, bg=COLORS['panel_bg'], fg=COLORS['subtext'],
                         highlightthickness=0, showvalue=False, length=180)
        scale.pack(side=tk.RIGHT)

        # Compare button
        btn_compare = tk.Button(parent, text="📊 Compare All 4 Algorithms Table", bg='#8E44AD', fg='white',
                                font=('Segoe UI', 9, 'bold'), relief=tk.FLAT, pady=4, cursor='hand2',
                                command=self._open_comparison_dialog)
        btn_compare.pack(fill=tk.X, pady=(6, 12))

    def _create_metrics_hud(self, parent: tk.Widget) -> None:
        lbl = ttk.Label(parent, text="LIVE METRICS & STATUS", style='Header.TLabel')
        lbl.pack(anchor=tk.W, pady=(4, 6))

        # Status Banner
        self.lbl_status = tk.Label(parent, text="STATUS: Ready", font=('Segoe UI', 9, 'bold'),
                                   fg='#A6E3A1', bg=COLORS['card_bg'], pady=4, relief=tk.RIDGE, bd=1)
        self.lbl_status.pack(fill=tk.X, pady=(0, 6))

        # HUD Grid (Cards)
        hud = tk.Frame(parent, bg=COLORS['panel_bg'])
        hud.pack(fill=tk.BOTH, expand=True)

        cards = [
            ("Robot Position", "lbl_pos", "(0, 0)"),
            ("Goal Position",  "lbl_goal", f"{self.grid.goal}"),
            ("Path Moves",     "lbl_moves", "0"),
            ("Accumulated Cost","lbl_cost", "0.0"),
            ("Total Plan Cost", "lbl_total_cost", "0.0"),
            ("Nodes Expanded", "lbl_nodes", "0"),
            ("Execution Time", "lbl_time", "0.00 ms"),
            ("Replans Occurred","lbl_replans", "0"),
        ]

        self.labels: Dict[str, tk.Label] = {}
        for i, (title, key, default_val) in enumerate(cards):
            row = i // 2
            col = i % 2
            card = tk.Frame(hud, bg=COLORS['card_bg'], padx=8, pady=5, relief=tk.RIDGE, bd=1)
            card.grid(row=row, column=col, padx=3, pady=3, sticky='nsew')
            hud.grid_columnconfigure(col, weight=1)

            t_lbl = tk.Label(card, text=title, font=('Segoe UI', 7), fg=COLORS['subtext'], bg=COLORS['card_bg'])
            t_lbl.pack(anchor=tk.W)
            v_lbl = tk.Label(card, text=default_val, font=('Segoe UI', 11, 'bold'), fg=COLORS['text'], bg=COLORS['card_bg'])
            v_lbl.pack(anchor=tk.W)
            self.labels[key] = v_lbl

        # Replan info card
        self.replan_info = tk.Label(parent, text="Trolley status: Clear", font=('Segoe UI', 8),
                                    fg=COLORS['subtext'], bg=COLORS['panel_bg'], wraplength=340, justify=tk.LEFT)
        self.replan_info.pack(fill=tk.X, pady=(8, 0))

    # -----------------------------------------------------------------------
    # Search & Computation
    # -----------------------------------------------------------------------
    def _compute_search(self) -> None:
        algo_name = self.current_algo.get()
        start_t = time.perf_counter()

        if self.trolley_check_var.get():
            # Trolley Replanning Simulation
            self.trolley_mode = True
            try:
                sim = TrolleySimulator(self.grid)
                self.sim_result = sim.run()
                self.planned_path = sim.initial_path
                self.trolley_pos = sim.trolley_positions[0] if sim.trolley_positions else None
                search_res = sim.initial_result
            except Exception as e:
                self.planned_path = []
                search_res = SearchResult("Trolley", None, 0, 0, 0)
        else:
            self.trolley_mode = False
            self.trolley_pos = None
            self.sim_result = None

            if "A*" in algo_name:
                search_res = astar(self.grid)
            elif "BFS" in algo_name:
                search_res = bfs(self.grid)
            elif "Greedy" in algo_name:
                search_res = greedy(self.grid)
            elif "DFS" in algo_name:
                search_res = dfs(self.grid)
            else:
                search_res = astar(self.grid)
            
            self.planned_path = search_res.path or []

        # Update HUD Metrics
        self.labels['lbl_pos'].config(text=str(self.grid.start))
        self.labels['lbl_goal'].config(text=str(self.grid.goal))
        self.labels['lbl_moves'].config(text=str(len(self.planned_path) - 1 if self.planned_path else 0))
        self.labels['lbl_total_cost'].config(text=f"{search_res.path_cost:.1f}")
        self.labels['lbl_nodes'].config(text=str(search_res.nodes_expanded))
        self.labels['lbl_time'].config(text=f"{search_res.elapsed_sec * 1000:.2f} ms")
        self.labels['lbl_cost'].config(text="0.0")
        self.labels['lbl_replans'].config(text="0")

        if self.trolley_mode and self.trolley_pos:
            self.replan_info.config(text=f"⚠️ Dynamic Trolley hidden at cell {self.trolley_pos} (unknown to robot until detected).")
        else:
            self.replan_info.config(text="Trolley status: None active.")

        if not self.planned_path:
            self.lbl_status.config(text="STATUS: ✗ No Path Found", fg='#F38BA8')
        else:
            self.lbl_status.config(text=f"STATUS: Plan Ready ({len(self.planned_path)-1} moves)", fg='#A6E3A1')

    # -----------------------------------------------------------------------
    # Canvas Rendering
    # -----------------------------------------------------------------------
    def _draw_grid(self) -> None:
        self.canvas.delete("all")
        rows, cols = self.grid.rows, self.grid.cols
        cw = self.canvas_size / cols
        ch = self.canvas_size / rows

        # Draw Cells
        for r in range(rows):
            for c in range(cols):
                sym = self.grid.cell(r, c)
                x1, y1 = c * cw, r * ch
                x2, y2 = x1 + cw, y1 + ch

                # Cell Color
                if sym == START:
                    col = COLORS['cell_start']
                    text = "S"
                elif sym == GOAL:
                    col = COLORS['cell_goal']
                    text = "G"
                elif sym == SHELF:
                    col = COLORS['cell_shelf']
                    text = "#"
                elif sym == RESTRICTED:
                    col = COLORS['cell_restricted']
                    text = "R"
                elif sym == CROWDED:
                    col = COLORS['cell_crowded']
                    text = "C (4)"
                elif sym == TROLLEY:
                    col = COLORS['cell_trolley']
                    text = "T"
                else:
                    col = COLORS['cell_open']
                    text = ""

                # Draw base rectangle
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=col, outline=COLORS['border'], width=1)

                # Cell label
                if text:
                    text_color = '#FFFFFF' if sym in (SHELF, RESTRICTED, TROLLEY) else '#222222'
                    self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text=text,
                                            font=('Segoe UI', int(cw * 0.26), 'bold'), fill=text_color)

        # Draw Planned Path line and waypoints
        if self.planned_path and len(self.planned_path) > 1:
            points = []
            for r, c in self.planned_path:
                points.append((c + 0.5) * cw)
                points.append((r + 0.5) * ch)

            # Path line
            path_col = COLORS['replan_path'] if self.replan_count > 0 else COLORS['path_line']
            self.canvas.create_line(*points, fill=path_col, width=3, dash=(4, 2), tag="path_line")

            # Waypoint dots
            for idx, (r, c) in enumerate(self.planned_path):
                if (r, c) not in (self.grid.start, self.grid.goal):
                    cx, cy = (c + 0.5) * cw, (r + 0.5) * ch
                    self.canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=path_col, outline="", tag="path_dot")

        # Draw Hidden Trolley indicator if in Trolley mode
        if self.trolley_mode and self.trolley_pos:
            tr, tc = self.trolley_pos
            tx1, ty1 = tc * cw, tr * ch
            tx2, ty2 = tx1 + cw, ty1 + ch
            # Draw hazard stripes border
            self.canvas.create_rectangle(tx1 + 2, ty1 + 2, tx2 - 2, ty2 - 2,
                                        outline=COLORS['cell_trolley'], width=2, dash=(3, 3))
            self.canvas.create_text((tx1 + tx2) / 2, (ty1 + ty2) / 2, text="⚠️ T",
                                    font=('Segoe UI', 9, 'bold'), fill='#D20F39')

        # Draw Robot Avatar at current step
        if self.planned_path and self.step_index < len(self.planned_path):
            rr, rc = self.planned_path[self.step_index]
            rx, ry = (rc + 0.5) * cw, (rr + 0.5) * ch
            rad = min(cw, ch) * 0.32

            # Outer glow
            self.canvas.create_oval(rx - rad - 3, ry - rad - 3, rx + rad + 3, ry + rad + 3,
                                    outline=COLORS['robot_glow'], width=2, tag="robot_glow")
            # Inner circle
            self.canvas.create_oval(rx - rad, ry - rad, rx + rad, ry + rad,
                                    fill=COLORS['robot'], outline='#D20F39', width=2, tag="robot")
            # Robot face/icon
            self.canvas.create_text(rx, ry, text="🤖", font=('Segoe UI', int(rad * 0.95)), tag="robot_icon")

    # -----------------------------------------------------------------------
    # Interactive Click Handling
    # -----------------------------------------------------------------------
    def _on_canvas_click(self, event: tk.Event) -> None:
        if self.anim_running:
            return  # Prevent edits mid-animation

        cw = self.canvas_size / self.grid.cols
        ch = self.canvas_size / self.grid.rows
        col = int(event.x // cw)
        row = int(event.y // ch)

        if not self.grid.in_bounds(row, col):
            return

        current_sym = self.grid.cell(row, col)
        if current_sym in (START, GOAL):
            return  # Don't overwrite Start or Goal

        # Cycle: Open '.' -> Shelf '#' -> Crowded 'C' -> Trolley 'T' -> Open '.'
        cycle_map = {
            OPEN:       SHELF,
            SHELF:      CROWDED,
            CROWDED:    TROLLEY,
            TROLLEY:    OPEN,
        }
        new_sym = cycle_map.get(current_sym, OPEN)
        self.grid._grid[row][col] = new_sym

        self._reset_sim()

    # -----------------------------------------------------------------------
    # Animation & Stepping
    # -----------------------------------------------------------------------
    def _toggle_run(self) -> None:
        if self.anim_running:
            # Pause
            self.anim_running = False
            self.btn_run.config(text="▶ Resume", bg='#27AE60')
            self.lbl_status.config(text="STATUS: ⏸ Paused", fg='#F9E2AF')
        else:
            # Start / Resume
            if not self.planned_path:
                messagebox.showwarning("No Path", "No valid path available to navigate!")
                return
            self.anim_running = True
            self.btn_run.config(text="⏸ Pause", bg='#F39C12')
            self.lbl_status.config(text="STATUS: 🏃 Navigating...", fg='#89B4FA')
            self._animate_loop()

    def _animate_loop(self) -> None:
        if not self.anim_running:
            return

        has_next = self._step_forward()
        if has_next:
            self.root.after(self.speed_ms.get(), self._animate_loop)
        else:
            self.anim_running = False
            self.btn_run.config(text="▶ Run", bg='#27AE60')

    def _step_once(self) -> None:
        if self.anim_running:
            self.anim_running = False
            self.btn_run.config(text="▶ Run", bg='#27AE60')
        self._step_forward()

    def _step_forward(self) -> bool:
        if not self.planned_path or self.step_index >= len(self.planned_path) - 1:
            self.lbl_status.config(text="STATUS: ✓ Goal Reached Successfully!", fg='#A6E3A1')
            return False

        # Next move
        next_pos = self.planned_path[self.step_index + 1]

        # Check for dynamic trolley obstacle in Trolley Mode
        if self.trolley_mode and self.trolley_pos and next_pos == self.trolley_pos:
            curr_pos = self.planned_path[self.step_index]
            # SENSOR LOOKAHEAD: Detected obstacle!
            self.lbl_status.config(text=f"STATUS: ⚠️ Obstacle Detected at {next_pos}!", fg='#E78284')
            self.grid.mark_impassable(*self.trolley_pos)
            self.replan_count += 1
            self.labels['lbl_replans'].config(text=str(self.replan_count))

            # Trigger Online A* Replanning from Current Position
            replan_res = astar(self.grid, start_override=curr_pos)
            if replan_res.path:
                self.planned_path = replan_res.path
                self.step_index = 0
                self.trolley_pos = None  # obstacle registered into map
                self.replan_info.config(text=f"✓ Replanned via A*! New Path Cost: {replan_res.path_cost:.1f} (Expanded {replan_res.nodes_expanded} nodes)")
                self.labels['lbl_moves'].config(text=f"{len(self.planned_path)-1} (remaining)")
                self._draw_grid()
                return True
            else:
                self.lbl_status.config(text="STATUS: ✗ Replan Failed — No Alternate Route!", fg='#F38BA8')
                self.replan_info.config(text="Robot is trapped: Destination unreachable.")
                self.anim_running = False
                self.btn_run.config(text="▶ Run", bg='#27AE60')
                return False

        # Advance one step
        self.step_index += 1
        curr_cell = self.planned_path[self.step_index]
        self.visited_cells.add(curr_cell)

        # Accumulate Cost
        self.accumulated_cost += self.grid.entry_cost(*curr_cell)
        self.labels['lbl_pos'].config(text=str(curr_cell))
        self.labels['lbl_cost'].config(text=f"{self.accumulated_cost:.1f}")

        self._draw_grid()

        if curr_cell == self.grid.goal:
            self.lbl_status.config(text="STATUS: ✓ Goal Reached Successfully!", fg='#A6E3A1')
            return False

        return True

    def _reset_sim(self) -> None:
        self.anim_running = False
        self.step_index = 0
        self.accumulated_cost = 0.0
        self.replan_count = 0
        self.visited_cells.clear()
        self.btn_run.config(text="▶ Run", bg='#27AE60')
        self._compute_search()
        self._draw_grid()

    def _reset_grid_layout(self) -> None:
        self.grid = WarehouseGrid(DEFAULT_LAYOUT)
        self._reset_sim()

    def _on_algo_change(self, event=None) -> None:
        self._reset_sim()

    def _on_trolley_check_toggle(self) -> None:
        self._reset_sim()

    # -----------------------------------------------------------------------
    # Comparison Dialog Table
    # -----------------------------------------------------------------------
    def _open_comparison_dialog(self) -> None:
        dlg = tk.Toplevel(self.root)
        dlg.title("Algorithm Performance Comparison Matrix")
        dlg.geometry("640x340")
        dlg.configure(bg=COLORS['bg'])
        dlg.transient(self.root)
        dlg.grab_set()

        lbl = tk.Label(dlg, text="Four-Algorithm Performance Comparison on Current Grid",
                       font=('Segoe UI', 12, 'bold'), fg=COLORS['accent'], bg=COLORS['bg'], pady=10)
        lbl.pack()

        # Run algorithms
        algos = [
            ("A* Search", astar),
            ("Breadth-First (BFS)", bfs),
            ("Greedy Best-First", greedy),
            ("Depth-First (DFS)", dfs),
        ]

        # Treeview Table
        columns = ("Algorithm", "Goal?", "Moves", "Cost", "Nodes", "Time (ms)")
        tree = ttk.Treeview(dlg, columns=columns, show='headings', height=6)
        for col in columns:
            tree.heading(col, text=col)
            tree.column(col, anchor=tk.CENTER, width=95)

        for name, fn in algos:
            res: SearchResult = fn(self.grid)
            goal_str = "Yes ✓" if res.goal_reached else "No ✗"
            moves_str = str(res.path_length) if res.goal_reached else "—"
            cost_str = f"{res.path_cost:.1f}" if res.goal_reached else "—"
            nodes_str = str(res.nodes_expanded)
            time_str = f"{res.elapsed_sec * 1000:.2f}"
            tree.insert('', tk.END, values=(name, goal_str, moves_str, cost_str, nodes_str, time_str))

        tree.pack(padx=15, pady=8, fill=tk.BOTH, expand=True)

        note_txt = ("• A* guarantees lowest cost by considering both real cost g(n) and heuristic h(n).\n"
                    "• BFS guarantees fewest moves, but may incur higher cost if it enters crowded cells.\n"
                    "• Greedy BFS expands the fewest nodes rapidly, but can be sub-optimal.\n"
                    "• DFS tends to take deep, suboptimal paths with high cost.")
        lbl_note = tk.Label(dlg, text=note_txt, font=('Segoe UI', 8), fg=COLORS['subtext'],
                            bg=COLORS['bg'], justify=tk.LEFT)
        lbl_note.pack(padx=15, pady=(0, 10), anchor=tk.W)

        btn_close = tk.Button(dlg, text="Close", bg=COLORS['card_bg'], fg='white',
                              font=('Segoe UI', 9, 'bold'), padx=15, command=dlg.destroy)
        btn_close.pack(pady=6)


# ---------------------------------------------------------------------------
# Main Launch Entry
# ---------------------------------------------------------------------------
def main() -> None:
    root = tk.Tk()
    app = WarehouseGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
