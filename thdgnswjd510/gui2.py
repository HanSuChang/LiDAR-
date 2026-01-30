import tkinter as tk
from tkinter import ttk
import threading
import time
import math
import datetime
import os
import heapq  # A* 알고리즘용

# ==========================================
# [라이브러리 임포트]
# ==========================================
try:
    import requests
except ImportError:
    print("⚠️ requests 라이브러리가 없습니다. (pip install requests)")
    requests = None

try:
    from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, Float, desc
    from sqlalchemy.orm import sessionmaker, declarative_base
    from sqlalchemy.sql import func

    ORM_AVAILABLE = True
except ImportError:
    ORM_AVAILABLE = False
    print("⚠️ SQLAlchemy not installed. (pip install sqlalchemy)")

# ==========================================
# [설정 및 상수]
# ==========================================
DB_FILE_NAME = "robot_db.db"
DATABASE_URL = f"sqlite:///{DB_FILE_NAME}"

ROBOT_IP = "192.168.0.244"
ROBOT_PORT = "5000"
CONTROL_URL = f"http://{ROBOT_IP}:{ROBOT_PORT}/control"

# 알고리즘 상수
CMD_TICK = 0.05
FWD_SPEED = 0.14
CRAWL_SPEED = 0.08
TARGET_DIST = 35.0
FRONT_LIMIT = 40.0
TURN_90_TIME = 1.3
MIN_CORNERS_TO_FINISH = 4
FINISH_RADIUS = 60.0
PRECISE_TARGET = 3.0

MAX_WAYPOINTS = 5
WP_ARRIVE_DIST = 15.0  # A* 경로 포인트 도달 판정 거리
WP_TURN_THRESH = 0.25
WP_ROT_SPEED = 0.45
WP_FWD_SPEED = 0.10

LEFT_90_CMD = (0.00, 0.8)
RIGHT_90_CMD = (0.00, -0.8)
STOP_CMD = (0.00, 0.0)

# 맵 설정
MAP_WIDTH_CM = 200
MAP_HEIGHT_CM = 100

# 초기 창 크기
INIT_WIN_W = 1300
INIT_WIN_H = 850
SIDEBAR_WIDTH = 320
LOG_HEIGHT = 250

PADDING_LEFT = 60
PADDING_RIGHT = 30
PADDING_TOP = 30
PADDING_BOTTOM = 50

GUI_START_X = 50.0
GUI_START_Y = 50.0
GUI_START_A = 0.0  # 초기 각도 (Degree)


# ==========================================
# [A* 알고리즘 클래스]
# ==========================================
class AStarPlanner:
    def __init__(self, ox, oy, resolution, robot_radius):
        self.resolution = resolution
        self.rr = robot_radius
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.x_width, self.y_width = 0, 0
        self.obstacle_map = None
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        pq = []
        heapq.heappush(pq, (0, self.calc_grid_index(start_node)))

        while True:
            if not open_set:
                return [], []

            cost, c_id = heapq.heappop(pq)
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                if n_id in closed_set: continue
                if not self.verify_node(node): continue

                if n_id not in open_set:
                    open_set[n_id] = node
                    priority = node.cost + self.calc_heuristic(node, goal_node)
                    heapq.heappush(pq, (priority, n_id))
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
                        priority = node.cost + self.calc_heuristic(node, goal_node)
                        heapq.heappush(pq, (priority, n_id))

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    def calc_heuristic(self, n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_pos):
        return index * self.resolution + min_pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)
        if px < self.min_x or py < self.min_y or \
                px >= self.max_x or py >= self.max_y:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]


# ==========================================
# [ORM 모델 정의]
# ==========================================
Base = declarative_base() if ORM_AVAILABLE else object


class RobotLog(Base):
    __tablename__ = 'robot_log'
    id = Column(Integer, primary_key=True, autoincrement=True)
    log_time = Column(DateTime)
    rel_x = Column(Float, default=0.0)
    rel_y = Column(Float, default=0.0)
    rel_angle = Column(Float, default=0.0)
    side_dist = Column(Float, default=0.0)
    action = Column(String(50), default="")
    log_type = Column(String(20))
    message = Column(Text)

    def __repr__(self):
        return f"<Log {self.id}: {self.message}>"


# ==========================================
# [데이터베이스 매니저]
# ==========================================
class DatabaseManager:
    def __init__(self):
        self.engine = None
        self.Session = None
        self.mock_mode = False
        self.mock_logs = []
        self.mock_id_counter = 0
        self.connect()

    def connect(self):
        if not ORM_AVAILABLE:
            self.mock_mode = True
            return
        try:
            self.engine = create_engine(DATABASE_URL, connect_args={'check_same_thread': False})
            Base.metadata.create_all(self.engine)
            self.Session = sessionmaker(bind=self.engine)
            print(f"✅ SQLite Connected: {os.path.abspath(DB_FILE_NAME)}")
        except Exception as e:
            print(f"⚠️ DB Error: {e}")
            self.mock_mode = True

    def insert_log(self, rel_x, rel_y, rel_angle, action, side_dist, log_type, message=""):
        current_time = datetime.datetime.now()
        if self.mock_mode:
            self.mock_id_counter += 1
            now_str = current_time.strftime("%H:%M:%S")
            self.mock_logs.append((self.mock_id_counter, now_str, action, message))
            return

        session = self.Session()
        try:
            new_log = RobotLog(
                rel_x=float(rel_x), rel_y=float(rel_y), rel_angle=float(rel_angle),
                action=str(action), side_dist=float(side_dist),
                log_type=str(log_type), message=str(message),
                log_time=current_time
            )
            session.add(new_log)
            session.commit()
        except:
            pass
        finally:
            session.close()

    def fetch_logs(self, limit=50):
        if self.mock_mode:
            return sorted(self.mock_logs, key=lambda x: x[0])[-limit:]
        session = self.Session()
        try:
            logs = session.query(RobotLog).order_by(desc(RobotLog.id)).limit(limit).all()
            result = []
            for log in reversed(logs):
                time_str = log.log_time.strftime("%H:%M:%S")
                display_msg = f"{log.message} (Act:{log.action}, Pos:{log.rel_x:.1f},{log.rel_y:.1f})"
                result.append((log.id, time_str, log.log_type, display_msg))
            return result
        except:
            return []
        finally:
            session.close()


db = DatabaseManager()


# ==========================================
# [공유 상태 관리]
# ==========================================
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()

        self.map_offset_x = GUI_START_X
        self.map_offset_y = GUI_START_Y
        self.map_offset_a = GUI_START_A

        self.gui_x = GUI_START_X
        self.gui_y = GUI_START_Y
        self.a = 0.0
        self.lidar = []
        self.lidar_points = []
        self.path = []
        self.waypoints = []
        self.wp_index = 0
        self.wp_enabled = True
        self.is_moving = False
        self.running = True
        self.logic_reset_needed = False
        self.pose_reset_needed = False

        self.obstacles_x = []
        self.obstacles_y = []
        self.planned_path_x = []
        self.planned_path_y = []
        self.return_mode = False


state = SharedState()


# ==========================================
# [로봇 로직 스레드]
# ==========================================
def normalize_angle(angle):
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle


def send_control(lin, ang):
    if requests is None: return

    def _req():
        try:
            requests.get(CONTROL_URL, params={'lin': lin, 'ang': ang}, timeout=0.2)
        except:
            pass

    threading.Thread(target=_req, daemon=True).start()


def get_robot_data():
    if requests is None: return None
    try:
        resp = requests.get(CONTROL_URL, timeout=0.2)
        if resp.status_code == 200: return resp.json()
    except:
        return None


def robot_logic_thread():
    print("🚀 Robot Logic Thread Started (A* Integrated)")
    db.insert_log(0, 0, 0, "INIT", 0, "SYSTEM", "Logic Thread Started")

    ox, oy = [], []
    for i in range(0, MAP_WIDTH_CM, 5): ox.append(i); oy.append(0.0)
    for i in range(0, MAP_WIDTH_CM, 5): ox.append(i); oy.append(MAP_HEIGHT_CM)
    for i in range(0, MAP_HEIGHT_CM, 5): ox.append(0.0); oy.append(i)
    for i in range(0, MAP_HEIGHT_CM, 5): ox.append(MAP_WIDTH_CM); oy.append(i)

    with state.lock:
        state.obstacles_x = ox
        state.obstacles_y = oy

    a_star = AStarPlanner(ox, oy, resolution=10.0, robot_radius=15.0)

    start_pos = None
    last_log_time = time.time()

    current_path_x = []
    current_path_y = []
    path_target_index = 0
    is_planning_done = False

    while state.running:
        if state.pose_reset_needed:
            start_pos = None
            is_planning_done = False
            state.pose_reset_needed = False
            print("📍 Pose Reset Triggered")

        if not state.is_moving:
            if state.logic_reset_needed:
                start_pos = None;
                is_planning_done = False
                last_log_time = time.time()
                with state.lock:
                    state.wp_index = 0
                    state.planned_path_x = []
                    state.planned_path_y = []
                    state.return_mode = False
                state.logic_reset_needed = False
                print("🔄 Algorithm Reset")
            time.sleep(0.1)
            continue

        data = get_robot_data()
        if not data:
            time.sleep(0.5)
            continue

        pos = data.get("p", {})
        curr_raw_x, curr_raw_y, curr_raw_a = pos.get("x", 0.0), pos.get("y", 0.0), pos.get("a", 0.0)
        raw_s = data.get("s", [])
        if len(raw_s) != 360: raw_s = [0.0] * 360

        if start_pos is None:
            start_pos = {'x': curr_raw_x, 'y': curr_raw_y, 'a': curr_raw_a}
            db.insert_log(0, 0, 0, "START_POS", 0, "CHECKPOINT", "Start/Reset Set")
            continue

        rel_x_cm = (curr_raw_x - start_pos['x']) * 100.0
        rel_y_cm = (curr_raw_y - start_pos['y']) * 100.0
        rel_a = normalize_angle(curr_raw_a - start_pos['a'])

        curr_map_x = state.map_offset_x + rel_x_cm
        curr_map_y = state.map_offset_y + rel_y_cm

        lidar_global_points = []
        for i, dist in enumerate(raw_s):
            d = float(dist)
            if 10.0 < d < 300.0:
                angle_rad = rel_a + math.radians(i - 90)
                obs_rel_x = (d * 100.0) * math.cos(angle_rad)
                obs_rel_y = (d * 100.0) * math.sin(angle_rad)
                obs_x = curr_map_x + obs_rel_x
                obs_y = curr_map_y + obs_rel_y
                lidar_global_points.append((obs_x, obs_y))

        with state.lock:
            state.gui_x = max(0.0, min(curr_map_x, MAP_WIDTH_CM))
            state.gui_y = max(0.0, min(curr_map_y, MAP_HEIGHT_CM))
            state.a = math.degrees(curr_raw_a)
            state.lidar = [float(v) for v in raw_s]
            state.lidar_points = lidar_global_points
            state.path.append((state.gui_x, state.gui_y))
            if len(state.path) > 3000: state.path.pop(0)

        with state.lock:
            wp_list = list(state.waypoints)
            wp_i = state.wp_index
            return_mode = state.return_mode

        if (len(wp_list) > 0 and wp_i < len(wp_list)) or return_mode:
            if not is_planning_done:
                if return_mode:
                    target_wp = (state.map_offset_x, state.map_offset_y)
                    db.insert_log(rel_x_cm, rel_y_cm, rel_a, "RETURN", 0, "ASTAR", "Returning to Start")
                else:
                    target_wp = wp_list[wp_i]
                    db.insert_log(rel_x_cm, rel_y_cm, rel_a, "PLANNING", 0, "ASTAR", f"Planning to WP{wp_i + 1}")

                rx, ry = a_star.planning(curr_map_x, curr_map_y, target_wp[0], target_wp[1])

                if rx and ry:
                    current_path_x = rx[::-1]
                    current_path_y = ry[::-1]
                    path_target_index = 0
                    is_planning_done = True
                    with state.lock:
                        state.planned_path_x = current_path_x
                        state.planned_path_y = current_path_y
                    print("Path Found!")
                else:
                    print("Path Not Found!")
                    send_control(*STOP_CMD)
                    state.is_moving = False
                    continue

            if is_planning_done and path_target_index < len(current_path_x):
                tx = current_path_x[path_target_index]
                ty = current_path_y[path_target_index]
                dx = tx - curr_map_x
                dy = ty - curr_map_y
                dist_to_pt = math.sqrt(dx * dx + dy * dy)

                if dist_to_pt < WP_ARRIVE_DIST:
                    path_target_index += 1
                    if path_target_index >= len(current_path_x):
                        is_planning_done = False
                        if return_mode:
                            print("Returned to Start!")
                            send_control(*STOP_CMD)
                            state.is_moving = False
                            db.insert_log(rel_x_cm, rel_y_cm, rel_a, "FINISHED", 0, "FINISH", "Mission Complete")
                        else:
                            print(f"Reached WP{wp_i + 1}")
                            db.insert_log(rel_x_cm, rel_y_cm, rel_a, "WP_ARRIVE", 0, "CHECKPOINT",
                                          f"Arrived WP{wp_i + 1}")
                            with state.lock:
                                state.wp_index += 1
                                if state.wp_index >= len(state.waypoints):
                                    state.return_mode = True
                        continue

                target_angle = math.atan2(dy, dx)
                angle_diff = normalize_angle(target_angle - rel_a)

                if abs(angle_diff) > WP_TURN_THRESH:
                    rot_cmd = (0.0, WP_ROT_SPEED) if angle_diff > 0 else (0.0, -WP_ROT_SPEED)
                    send_control(*rot_cmd)
                else:
                    send_control(WP_FWD_SPEED, 0.0)

                time.sleep(CMD_TICK)
                continue

        if not wp_list and not return_mode:
            send_control(*STOP_CMD)
            time.sleep(0.5)


# ==========================================
# [GUI 클래스]
# ==========================================
class RobotMapGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Monitor System (Final Fixed)")
        self.root.geometry(f"{INIT_WIN_W}x{INIT_WIN_H}")
        self.root.resizable(True, True)

        self.last_log_id = -1
        self.scale = 1.0
        self.origin_px_x = PADDING_LEFT
        self.origin_px_y = 0
        self.setting_pose_mode = False

        self._build_ui()
        self.update_gui()

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=0)

        # Map Canvas
        self.canvas = tk.Canvas(self.root, bg="white")
        self.canvas.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        self.canvas.bind("<Button-1>", self.on_map_click)
        self.canvas.bind("<Configure>", self.on_resize)

        # Sidebar
        self.sidebar = tk.Frame(self.root, width=SIDEBAR_WIDTH, bg="#202020")
        self.sidebar.grid(row=0, column=1, rowspan=2, sticky="ns")
        self.sidebar.pack_propagate(False)

        # Log Viewer
        self.frame_log = tk.Frame(self.root, bg="#222", height=LOG_HEIGHT)
        self.frame_log.grid(row=1, column=0, sticky="ew")
        self.frame_log.pack_propagate(False)

        # --- Sidebar Widgets ---
        self.sidebar_top = tk.Frame(self.sidebar, bg="#202020")
        self.sidebar_top.pack(side="top", fill="x")

        tk.Label(self.sidebar_top, text="STATUS MONITOR", font=("Impact", 18), bg="#202020", fg="white").pack(
            pady=(20, 10))
        self.radar_canvas = tk.Canvas(self.sidebar_top, bg="black", width=260, height=260, highlightthickness=2,
                                      highlightbackground="#444")
        self.radar_canvas.pack(pady=10)

        # [수정] A값 추가
        self.lbl_start = tk.Label(self.sidebar_top, text=f"START: ({GUI_START_X},{GUI_START_Y},{GUI_START_A})",
                                  font=("Consolas", 12), bg="#202020", fg="#AAAAAA")
        self.lbl_start.pack(pady=5, fill="x", padx=10)
        self.lbl_curr = tk.Label(self.sidebar_top, text="Waiting...", font=("Consolas", 12, "bold"), bg="#202020",
                                 fg="#00FF00")
        self.lbl_curr.pack(pady=5, fill="x", padx=10)

        self.sidebar_mid = tk.Frame(self.sidebar, bg="#202020")
        self.sidebar_mid.pack(side="top", fill="both", expand=True)

        tk.Label(self.sidebar_mid, text="[ WAY POINTS ]", font=("Arial", 10, "bold"), bg="#202020", fg="orange",
                 anchor="center").pack(pady=(15, 5), fill="x", padx=20)
        self.lbl_wps = tk.Label(self.sidebar_mid, text="- None -", font=("Consolas", 10), bg="#202020", fg="white",
                                justify="center", anchor="n")
        self.lbl_wps.pack(pady=5, fill="both", expand=True, padx=20)

        self.sidebar_bottom = tk.Frame(self.sidebar, bg="#202020")
        self.sidebar_bottom.pack(side="bottom", fill="x", pady=20)

        btn_frame = tk.Frame(self.sidebar_bottom, bg="#202020")
        btn_frame.pack(side="bottom", pady=10, fill="x")

        sub_btn_frame = tk.Frame(btn_frame, bg="#202020")
        sub_btn_frame.pack(side="top", fill="x", padx=20, pady=2)

        self.btn_clear_log = tk.Button(sub_btn_frame, text="Clear Log", command=self.clear_logs, bg="#444", fg="white")
        self.btn_clear_log.pack(side="left", fill="x", expand=True, padx=2)

        self.btn_clear_map = tk.Button(sub_btn_frame, text="Clear Map", command=self.clear_map, bg="#444", fg="white")
        self.btn_clear_map.pack(side="right", fill="x", expand=True, padx=2)

        self.btn_start = tk.Button(btn_frame, text="START", command=self.toggle_robot, bg="#00AA00", fg="white",
                                   font=("Arial", 12, "bold"), height=2)
        self.btn_start.pack(side="top", fill="x", padx=20, pady=5)

        self.btn_set_pos = tk.Button(btn_frame, text="Set Pos", command=self.toggle_set_pose_mode, bg="#444", fg="cyan")
        self.btn_set_pos.pack(side="top", fill="x", padx=20, pady=2)

        self.init_log_viewer()

    def init_log_viewer(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", background="#222222", foreground="white", fieldbackground="#222222", rowheight=25,
                        font=("Arial", 10))
        style.configure("Treeview.Heading", background="#444444", foreground="white", font=("Arial", 10, "bold"))
        style.map("Treeview", background=[('selected', '#005500')])

        scrollbar = tk.Scrollbar(self.frame_log)
        scrollbar.pack(side="right", fill="y")

        cols = ("time", "action", "msg")
        self.log_tree = ttk.Treeview(self.frame_log, columns=cols, show="headings", yscrollcommand=scrollbar.set)
        self.log_tree.heading("time", text="Time");
        self.log_tree.column("time", width=120, anchor="center")
        self.log_tree.heading("action", text="Action");
        self.log_tree.column("action", width=150, anchor="center")
        self.log_tree.heading("msg", text="Message / Coordinates");
        self.log_tree.column("msg", width=900, anchor="w")

        self.log_tree.pack(fill="both", expand=True)
        scrollbar.config(command=self.log_tree.yview)

    def toggle_set_pose_mode(self):
        self.setting_pose_mode = not self.setting_pose_mode
        if self.setting_pose_mode:
            self.btn_set_pos.config(bg="cyan", fg="black", text="Click Map")
            db.insert_log(0, 0, 0, "MODE", 0, "GUI", "Select Robot Position on Map")
        else:
            self.btn_set_pos.config(bg="#444", fg="cyan", text="Set Pos")

    def on_map_click(self, event):
        cx, cy = self.px_to_cm(event.x, event.y)
        if not (0 <= cx <= MAP_WIDTH_CM and 0 <= cy <= MAP_HEIGHT_CM):
            return

        with state.lock:
            if self.setting_pose_mode:
                state.map_offset_x = cx
                state.map_offset_y = cy
                state.pose_reset_needed = True
                # [수정] A값 포함하여 갱신 (0.0으로 고정)
                self.lbl_start.config(text=f"START: ({cx:.1f}, {cy:.1f}, 0.0)")
                db.insert_log(cx, cy, 0, "SET_POSE", 0, "INPUT", f"Robot Pos Updated to ({cx:.1f}, {cy:.1f})")
                self.toggle_set_pose_mode()
            else:
                if len(state.waypoints) < MAX_WAYPOINTS:
                    state.waypoints.append((cx, cy))
                    db.insert_log(cx, cy, 0, "WP_ADD", 0, "INPUT", f"WP added")

    def on_resize(self, event):
        w = event.width - (PADDING_LEFT + PADDING_RIGHT)
        h = event.height - (PADDING_TOP + PADDING_BOTTOM)
        if w > 0 and h > 0:
            self.scale = min(w / MAP_WIDTH_CM, h / MAP_HEIGHT_CM)
            self.origin_px_x = PADDING_LEFT
            self.origin_px_y = PADDING_TOP + (MAP_HEIGHT_CM * self.scale)
            self.update_gui_instant()

    def cm_to_px(self, cm_x, cm_y):
        px = self.origin_px_x + (cm_x * self.scale)
        py = self.origin_px_y - (cm_y * self.scale)
        return px, py

    def px_to_cm(self, px, py):
        cm_x = (px - self.origin_px_x) / self.scale
        cm_y = (self.origin_px_y - py) / self.scale
        return cm_x, cm_y

    def refresh_logs(self):
        logs = db.fetch_logs(limit=50)
        if not logs: return
        newest_id = logs[-1][0]
        if newest_id != self.last_log_id:
            for item in self.log_tree.get_children(): self.log_tree.delete(item)
            for log in logs: self.log_tree.insert("", "end", values=(log[1], log[2], log[3]))
            with state.lock:
                moving = state.is_moving
            if moving or (newest_id > self.last_log_id): self.log_tree.yview_moveto(1.0)
            self.last_log_id = newest_id

    def toggle_robot(self):
        with state.lock:
            if state.is_moving:
                state.is_moving = False
                send_control(*STOP_CMD)
                self.btn_start.config(text="START", bg="#00AA00")
                db.insert_log(0, 0, 0, "STOP", 0, "SYSTEM", "User Stopped")
            else:
                state.logic_reset_needed = True
                state.is_moving = True
                self.btn_start.config(text="STOP", bg="#CC0000")
                db.insert_log(0, 0, 0, "START", 0, "SYSTEM", "User Started")

    def clear_map(self):
        with state.lock: state.path = []; state.waypoints = []; state.wp_index = 0; state.planned_path_x = []; state.planned_path_y = []
        db.insert_log(0, 0, 0, "CLEAR", 0, "GUI", "Map Cleared")

    def clear_logs(self):
        for item in self.log_tree.get_children(): self.log_tree.delete(item)

    def draw_grid(self):
        self.canvas.delete("grid")
        px_start, _ = self.cm_to_px(0, 0)
        px_end, _ = self.cm_to_px(MAP_WIDTH_CM, 0)

        for y in range(0, MAP_HEIGHT_CM + 1, 50):
            _, py = self.cm_to_px(0, y)
            self.canvas.create_line(px_start, py, px_end, py, fill="#DDDDDD", width=1, tag="grid")
            self.canvas.create_text(px_start - 25, py, text=str(y), fill="black", tag="grid")

        _, py_start = self.cm_to_px(0, 0)
        _, py_end = self.cm_to_px(0, MAP_HEIGHT_CM)
        for x in range(0, MAP_WIDTH_CM + 1, 50):
            px, _ = self.cm_to_px(x, 0)
            self.canvas.create_line(px, py_start, px, py_end, fill="#DDDDDD", width=1, tag="grid")
            self.canvas.create_text(px, py_start + 15, text=str(x), fill="black", tag="grid")

        ox, oy = self.cm_to_px(0, 0)
        trx, try_ = self.cm_to_px(MAP_WIDTH_CM, MAP_HEIGHT_CM)
        self.canvas.create_rectangle(ox, oy, trx, try_, outline="black", width=2, tag="grid")

        with state.lock:
            for ox, oy in zip(state.obstacles_x, state.obstacles_y):
                px, py = self.cm_to_px(ox, oy)
                self.canvas.create_rectangle(px - 1, py - 1, px + 1, py + 1, fill="black", tag="grid")

            for lx, ly in state.lidar_points:
                px, py = self.cm_to_px(lx, ly)
                self.canvas.create_rectangle(px - 1, py - 1, px + 1, py + 1, fill="orange", outline="", tag="dynamic")

        with state.lock:
            sx, sy = self.cm_to_px(state.map_offset_x, state.map_offset_y)

        self.canvas.create_rectangle(sx - 6, sy - 6, sx + 6, sy + 6, fill="#00FF00", outline="black", width=2,
                                     tag="grid")
        self.canvas.create_text(sx, sy - 15, text="START", fill="#008800", font=("Arial", 9, "bold"), tag="grid")

    def update_gui_instant(self):
        self.canvas.delete("dynamic")
        self.draw_grid()

        with state.lock:
            cx, cy, ca = state.gui_x, state.gui_y, state.a
            path = list(state.path)
            wps = list(state.waypoints)
            wp_i = state.wp_index
            plan_x = list(state.planned_path_x)
            plan_y = list(state.planned_path_y)

        # Draw A* Path
        if len(plan_x) > 1:
            plan_coords = []
            for px, py in zip(plan_x, plan_y):
                plan_coords.extend(self.cm_to_px(px, py))
            self.canvas.create_line(plan_coords, fill="green", width=2, dash=(4, 2), tag="dynamic")

        if len(path) > 1:
            coords = [c for xy in path for c in self.cm_to_px(*xy)]
            self.canvas.create_line(coords, fill="blue", width=2, tag="dynamic")

        for idx, (wx, wy) in enumerate(wps):
            px, py = self.cm_to_px(wx, wy)
            color = "orange" if idx == wp_i else "gray"
            self.canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill=color, tag="dynamic")
            self.canvas.create_text(px, py - 12, text=f"WP{idx + 1}", fill=color, font=("Arial", 9, "bold"),
                                    tag="dynamic")

        rx, ry = self.cm_to_px(cx, cy)
        rad = math.radians(ca)
        p1 = (rx + 12 * math.cos(rad), ry - 12 * math.sin(rad))
        p2 = (rx + 8 * math.cos(rad + 2.5), ry - 8 * math.sin(rad + 2.5))
        p3 = (rx + 8 * math.cos(rad - 2.5), ry - 8 * math.sin(rad - 2.5))
        self.canvas.create_polygon(p1, p2, p3, fill="red", outline="black", tag="dynamic")

    def update_gui(self):
        if not state.running: return
        self.update_gui_instant()

        with state.lock:
            cx, cy, ca = state.gui_x, state.gui_y, state.a
            lidar = list(state.lidar)
            is_moving = state.is_moving
            wps = list(state.waypoints)
            wp_i = state.wp_index
            ret_mode = state.return_mode

        if is_moving and self.btn_start.cget('text') == "START":
            self.btn_start.config(text="STOP", bg="#CC0000")
        elif not is_moving and self.btn_start.cget('text') == "STOP":
            self.btn_start.config(text="START", bg="#00AA00")

        deg_a = (ca + 360) % 360
        self.lbl_curr.config(text=f"X : {cx:6.2f} cm\nY : {cy:6.2f} cm\nA : {deg_a:6.2f} deg")

        if not wps and not ret_mode:
            self.lbl_wps.config(text="- None -")
        else:
            wp_txt = ""
            for i, (wx, wy) in enumerate(wps):
                mark = ""
                if i < wp_i:
                    mark = " [Done]"
                elif i == wp_i and is_moving and not ret_mode:
                    mark = " <<<"
                wp_txt += f"WP{i + 1}: ({wx:5.1f}, {wy:5.1f}){mark}\n"

            if ret_mode:
                wp_txt += "\n[ Returning... ]"
            self.lbl_wps.config(text=wp_txt.strip())

        self.radar_canvas.delete("all")
        rcx, rcy = 130, 130
        scale = 0.4
        for r in range(50, 301, 50):
            rpx = r * scale
            self.radar_canvas.create_oval(rcx - rpx, rcy - rpx, rcx + rpx, rcy + rpx, outline="#004400", dash=(2, 4))
            self.radar_canvas.create_text(rcx, rcy - rpx, text=str(r), fill="#006600", font=("Arial", 7), anchor="s")
        for ang in range(0, 360, 45):
            rad_ang = math.radians(ang - 90)
            ex, ey = rcx + 120 * math.cos(rad_ang), rcy + 120 * math.sin(rad_ang)
            self.radar_canvas.create_line(rcx, rcy, ex, ey, fill="#004400")
            self.radar_canvas.create_text(ex, ey, text=f"{ang}°", fill="#008800", font=("Arial", 8))

        head_rad = math.radians(ca - 90)
        hx = rcx + 100 * math.cos(head_rad)
        hy = rcy + 100 * math.sin(head_rad)
        self.radar_canvas.create_line(rcx, rcy, hx, hy, fill="red", width=2, arrow=tk.LAST)
        self.radar_canvas.create_oval(rcx - 3, rcy - 3, rcx + 3, rcy + 3, fill="red")

        for i, dist in enumerate(lidar):
            if dist <= 0 or dist > 300: continue
            rad_ang = math.radians(i - 90)
            lx = rcx + (dist * scale) * math.cos(rad_ang)
            ly = rcy + (dist * scale) * math.sin(rad_ang)
            self.radar_canvas.create_rectangle(lx, ly, lx + 2, ly + 2, fill="#00FF00", outline="")

        self.refresh_logs()
        self.root.after(50, self.update_gui)


if __name__ == "__main__":
    t = threading.Thread(target=robot_logic_thread, daemon=True)
    t.start()

    root = tk.Tk()
    app = RobotMapGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        state.running = False
        send_control(*STOP_CMD)