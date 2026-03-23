# 파일명: robot_gui.py
import tkinter as tk
from tkinter import ttk
import math

SIDEBAR_WIDTH = 300
LOG_HEIGHT = 250
ROI_SIZE = 30.0
DRAW_SAMPLING_DIST = 15.0


class RobotMapGUI:
    def __init__(self, root, state, db_manager, control_func, emergency_func, parking_func):
        self.root = root
        self.state = state
        self.db = db_manager

        self.toggle_robot_func = control_func
        self.emergency_stop_func = emergency_func
        self.trigger_parking_func = parking_func

        self.root.title("Robot Monitor (Safe Park)")
        self.root.attributes('-fullscreen', True)
        self.root.bind("<Escape>", lambda e: self.root.attributes("-fullscreen", False))

        self.last_log_id = -1

        self.root.bind("<Control-plus>", self.zoom_in)
        self.root.bind("<Control-equal>", self.zoom_in)
        self.root.bind("<Control-minus>", self.zoom_out)
        self.root.bind("<Control-MouseWheel>", self.on_mouse_wheel)

        self.root.bind("<Button-3>", self.on_right_click_press)
        self.root.bind("<B3-Motion>", self.on_right_click_drag)

        self.view_center_x, self.view_center_y = 50.0, 50.0
        self.last_mouse_x, self.last_mouse_y = 0, 0

        self.frame_side = tk.Frame(root, width=SIDEBAR_WIDTH, bg="#202020")
        self.frame_side.pack(side="right", fill="y")
        self.frame_side.pack_propagate(False)

        self.frame_left = tk.Frame(root, bg="#F0F0F0")
        self.frame_left.pack(side="left", fill="both", expand=True)

        self.frame_map = tk.Frame(self.frame_left, bg="white")
        self.frame_map.pack(side="top", fill="both", expand=True)

        self.canvas = tk.Canvas(self.frame_map, bg="white", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)

        self.canvas.bind("<Button-1>", self.on_map_click)
        self.canvas.bind("<B1-Motion>", self.on_map_drag)

        self.frame_bottom = tk.Frame(self.frame_left, height=LOG_HEIGHT, bg="#222")
        self.frame_bottom.pack(side="bottom", fill="x")

        self.setup_log_view()
        self.setup_sidebar_ui()
        self.refresh_logs()
        self.update_gui()

    def zoom_in(self, event=None):
        with self.state.lock:
            self.state.view_w = max(40, self.state.view_w - 20)
            self.state.view_h = self.state.view_w / 2

    def zoom_out(self, event=None):
        with self.state.lock:
            self.state.view_w = min(2000, self.state.view_w + 20)
            self.state.view_h = self.state.view_w / 2

    def on_mouse_wheel(self, event):
        self.zoom_in() if event.delta > 0 else self.zoom_out()

    def on_right_click_press(self, event):
        self.last_mouse_x, self.last_mouse_y = event.x, event.y

    def on_right_click_drag(self, event):
        dx, dy = event.x - self.last_mouse_x, event.y - self.last_mouse_y
        s, _, _ = self.get_dynamic_params()
        self.view_center_x -= dx / s
        self.view_center_y += dy / s
        self.last_mouse_x, self.last_mouse_y = event.x, event.y

    def get_dynamic_params(self):
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 100:
            return 1.0, 0, 0
        with self.state.lock:
            cw_v, ch_v = self.state.view_w, self.state.view_h
        scale = min(cw / cw_v, ch / ch_v) * 0.95
        ox = (cw / 2) - (self.view_center_x * scale)
        oy = (ch / 2) + (self.view_center_y * scale)
        return scale, ox, oy

    def cm_to_px(self, cx, cy, s, ox, oy):
        return ox + (cx * s), oy - (cy * s)

    def px_to_cm(self, px, py, s, ox, oy):
        return (px - ox) / s, (oy - py) / s

    def setup_sidebar_ui(self):
        self.fixed_container = tk.Frame(self.frame_side, bg="#202020")
        self.fixed_container.pack(side="top", fill="x")

        tk.Label(self.fixed_container, text="STATUS MONITOR", font=("Impact", 22), bg="#202020", fg="white").pack(
            pady=(40, 10))
        self.radar_canvas = tk.Canvas(self.fixed_container, bg="black", width=260, height=260, highlightthickness=1,
                                      highlightbackground="#444")
        self.radar_canvas.pack(pady=10)

        self.lbl_curr = tk.Label(self.fixed_container, text="X: 50.0 Y: 50.0\nA: 0.00 rad",
                                 font=("Consolas", 13, "bold"), bg="#202020", fg="#00FF00", justify="left")
        self.lbl_curr.pack(pady=5)
        self.lbl_status = tk.Label(self.fixed_container, text="STATUS: IDLE", font=("Consolas", 13, "bold"),
                                   bg="#202020", fg="orange")
        self.lbl_status.pack(pady=5)

        tk.Label(self.fixed_container, text="-------------------", bg="#202020", fg="#444").pack()
        self.lbl_target_wp = tk.Label(self.fixed_container, text="No Target", font=("Consolas", 11, "bold"),
                                      bg="#202020", fg="#87CEEB")
        self.lbl_target_wp.pack(pady=2)
        self.lbl_roi_pts = tk.Label(self.fixed_container, text="Detect: 0 pts", font=("Consolas", 11), bg="#202020",
                                    fg="#00FF00")
        self.lbl_roi_pts.pack(pady=2)
        self.lbl_roi_status = tk.Label(self.fixed_container, text="WAITING", font=("Arial", 9, "italic"), bg="#202020",
                                       fg="#666666")
        self.lbl_roi_status.pack(pady=5)

        self.btn_scroll_container = tk.Frame(self.frame_side, bg="#202020")
        self.btn_scroll_container.pack(side="top", fill="both", expand=True)
        self.btn_canvas = tk.Canvas(self.btn_scroll_container, bg="#202020", highlightthickness=0)
        self.btn_scrollbar = tk.Scrollbar(self.btn_scroll_container, orient="vertical", command=self.btn_canvas.yview)
        self.btn_scrollable_frame = tk.Frame(self.btn_canvas, bg="#202020")
        self.btn_scrollable_frame.bind("<Configure>",
                                       lambda e: self.btn_canvas.configure(scrollregion=self.btn_canvas.bbox("all")))
        self.btn_canvas.create_window((0, 0), window=self.btn_scrollable_frame, anchor="nw", width=SIDEBAR_WIDTH - 20)
        self.btn_canvas.configure(yscrollcommand=self.btn_scrollbar.set)
        self.btn_canvas.pack(side="left", fill="both", expand=True)
        self.btn_scrollbar.pack(side="right", fill="y")

        self.frame_ctrl = tk.Frame(self.btn_scrollable_frame, bg="#202020")
        self.frame_ctrl.pack(fill="x", padx=20, pady=10)

        btn_style = {"font": ("Arial", 11, "bold"), "height": 1, "fg": "white", "relief": "raised"}
        cp = 4

        tk.Button(self.frame_ctrl, text="🚨 EMERGENCY STOP", command=self.emergency_stop_func, bg="#FF0000",
                  **btn_style).pack(fill="x", pady=cp)
        tk.Button(self.frame_ctrl, text="Clear Map+WP", command=self.clear_map, bg="#444", **btn_style).pack(fill="x",
                                                                                                             pady=cp)
        self.btn_start = tk.Button(self.frame_ctrl, text="START", command=self.toggle_robot_func, bg="#00AA00",
                                   **btn_style)
        self.btn_start.pack(fill="x", pady=cp)
        tk.Button(self.frame_ctrl, text="Clear Log", command=self.clear_logs, bg="#444", **btn_style).pack(fill="x",
                                                                                                           pady=cp)
        tk.Button(self.frame_ctrl, text="전면 주차", command=lambda: self.trigger_parking_func("FRONT"), bg="#FF8C00",
                  **btn_style).pack(fill="x", pady=cp)
        tk.Button(self.frame_ctrl, text="후면 주차", command=lambda: self.trigger_parking_func("REAR"), bg="#8B4513",
                  **btn_style).pack(fill="x", pady=cp)

    def setup_log_view(self):
        self.frame_log_left = tk.Frame(self.frame_bottom, bg="#222")
        self.frame_log_left.pack(side="left", fill="both", expand=True)

        scrollbar = tk.Scrollbar(self.frame_log_left)
        scrollbar.pack(side="right", fill="y")
        self.log_tree = ttk.Treeview(self.frame_log_left, columns=("1", "2", "3"), show="headings",
                                     yscrollcommand=scrollbar.set)
        self.log_tree.heading("1", text="Time")
        self.log_tree.column("1", width=120, anchor="center")
        self.log_tree.heading("2", text="Action")
        self.log_tree.column("2", width=150, anchor="center")
        self.log_tree.heading("3", text="Message")
        self.log_tree.column("3", width=700, anchor="w")
        self.log_tree.pack(fill="both", expand=True, padx=2, pady=2)
        scrollbar.config(command=self.log_tree.yview)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", background="#222", foreground="white", fieldbackground="#222", rowheight=25,
                        font=("Arial", 10))
        style.map("Treeview", background=[('selected', '#444')])

    def refresh_logs(self):
        logs = self.db.fetch_logs(limit=50)
        if not logs:
            return
        newest = logs[-1][0]
        if self.last_log_id == -1 or newest > self.last_log_id or len(self.log_tree.get_children()) == 0:
            for item in self.log_tree.get_children():
                self.log_tree.delete(item)
            for log in logs:
                self.log_tree.insert("", "end", values=(log[1], log[2], log[3]))
            self.log_tree.yview_moveto(1.0)
            self.last_log_id = newest

    def clear_map(self):
        with self.state.lock:
            self.state.path, self.state.waypoints, self.state.wp_index = [], [], 0

    def clear_logs(self):
        for item in self.log_tree.get_children():
            self.log_tree.delete(item)

    def on_map_click(self, event):
        s, ox, oy = self.get_dynamic_params()
        cx, cy = self.px_to_cm(event.x, event.y, s, ox, oy)

        with self.state.lock:
            if len(self.state.waypoints) >= 5:
                return
            self.state.waypoints.append((cx, cy))
            wp_num = len(self.state.waypoints)

        self.db.insert_log(cx, cy, 0.0, "WP_ADD", "USER", f"WP #{wp_num} Added")

    def on_map_drag(self, event):
        s, ox, oy = self.get_dynamic_params()
        cx, cy = self.px_to_cm(event.x, event.y, s, ox, oy)

        with self.state.lock:
            if len(self.state.waypoints) >= 5:
                return

            if not self.state.waypoints:
                self.state.waypoints.append((cx, cy))
            else:
                lx, ly = self.state.waypoints[-1]
                dist = math.hypot(cx - lx, cy - ly)
                if dist >= DRAW_SAMPLING_DIST:
                    self.state.waypoints.append((cx, cy))

    def update_gui(self):
        if not self.state.running:
            return

        self.refresh_logs()
        self.canvas.delete("all")
        self.radar_canvas.delete("all")

        s, ox, oy = self.get_dynamic_params()

        with self.state.lock:
            curr_w, curr_h = self.state.view_w, self.state.view_h
            cx, cy, ca = self.state.gui_x, self.state.gui_y, self.state.a
            sx, sy, sa = self.state.start_x, self.state.start_y, self.state.start_a
            path, wps, wpi = list(self.state.path), list(self.state.waypoints), self.state.wp_index
            lidar = list(self.state.lidar)
            status, is_moving = self.state.mission_status, self.state.is_moving

        # grid
        if self.canvas.winfo_width() > 100:
            st = 50
            sx_grid = (int((self.view_center_x - curr_w) / st) - 1) * st
            ex_grid = (int((self.view_center_x + curr_w) / st) + 1) * st
            sy_grid = (int((self.view_center_y - curr_h) / st) - 1) * st
            ey_grid = (int((self.view_center_y + curr_h) / st) + 1) * st
            for x in range(sx_grid, ex_grid + 1, st):
                px, _ = self.cm_to_px(x, 0, s, ox, oy)
                self.canvas.create_line(px, 0, px, self.canvas.winfo_height(), fill="#F0F0F0")
            for y in range(sy_grid, ey_grid + 1, st):
                _, py = self.cm_to_px(0, y, s, ox, oy)
                self.canvas.create_line(0, py, self.canvas.winfo_width(), py, fill="#F0F0F0")

            # info
            self.canvas.create_rectangle(20, 20, 360, 115, fill="#E8F5E9", outline="#2E7D32")
            f = ("Consolas", 10, "bold")
            self.canvas.create_text(35, 45, text=f"START  | X:{sx:7.2f}, Y:{sy:7.2f}, θ:{sa:6.1f}°", anchor="w",
                                    font=f, fill="#1B5E20")
            self.canvas.create_text(35, 70, text=f"CURR   | X:{cx:7.2f}, Y:{cy:7.2f}, θ:{ca:6.1f}°", anchor="w",
                                    font=f, fill="#1B5E20")

            # path
            if len(path) > 1:
                pts = [c for xy in path for c in self.cm_to_px(*xy, s, ox, oy)]
                self.canvas.create_line(pts, fill="blue", width=2)

            # wp
            for i, (wx, wy) in enumerate(wps):
                px, py = self.cm_to_px(wx, wy, s, ox, oy)
                col = "orange" if i == wpi else "gray"
                self.canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill=col, outline="black")
                self.canvas.create_text(px, py - 15, text=f"WP{i + 1}", fill="#555", font=("Arial", 8, "bold"))

            # robot
            rx, ry = self.cm_to_px(cx, cy, s, ox, oy)
            rad_robot = math.radians(ca)
            p1 = (rx + 15 * math.cos(rad_robot), ry - 15 * math.sin(rad_robot))
            p2 = (rx + 10 * math.cos(rad_robot + 2.6), ry - 10 * math.sin(rad_robot + 2.6))
            p3 = (rx + 10 * math.cos(rad_robot - 2.6), ry - 10 * math.sin(rad_robot - 2.6))
            self.canvas.create_polygon(p1, p2, p3, fill="red", outline="black")

        # radar
        rc = 130
        for r in range(40, 121, 40):
            self.radar_canvas.create_oval(rc - r, rc - r, rc + r, rc + r, outline="#004400", dash=(2, 2))
        for i, d in enumerate(lidar):
            if 0 < d < 300:
                ra_lid = math.radians(i - 90)
                lx, ly = rc + (d * 0.4) * math.cos(ra_lid), rc + (d * 0.4) * math.sin(ra_lid)
                self.radar_canvas.create_rectangle(lx, ly, lx + 1, ly + 1, fill="#00FF00", outline="")

        norm_deg = (ca + 360) % 360
        self.lbl_curr.config(text=f"X: {cx:6.1f} Y: {cy:6.1f}\nA: {math.radians(norm_deg):.2f} rad")
        self.lbl_status.config(text=f"STATUS: {status}")
        self.btn_start.config(text="STOP" if is_moving else "START", bg="#CC0000" if is_moving else "#00AA00")

        self.root.after(50, self.update_gui)
