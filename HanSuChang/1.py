# 파일명: robot_logic.py
import threading
import time
import math
import datetime
import requests

# ==========================================
# [1. 라이브러리 및 설정] (로직 파트)
# ==========================================
try:
    import pymysql
    from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, Float, desc
    from sqlalchemy.orm import sessionmaker, declarative_base

    ORM_AVAILABLE = True
except ImportError:
    ORM_AVAILABLE = False
    Base = object
    Column = Integer = String = Text = DateTime = Float = desc = lambda *args, **kwargs: None
    create_engine = sessionmaker = declarative_base = lambda *args, **kwargs: None

# MySQL 접속 설정
DB_USER, DB_PASS, DB_HOST, DB_PORT, DB_NAME = "root", "0000", "localhost", "3306", "robot_db"
DATABASE_URL = f"mysql+pymysql://{DB_USER}:{DB_PASS}@{DB_HOST}:{DB_PORT}/{DB_NAME}"
ROBOT_IP, ROBOT_PORT = "192.168.0.243", "5151"
CONTROL_URL = f"http://{ROBOT_IP}:{ROBOT_PORT}/control"

# ==========================================
# [주행/제어 상수]
# ==========================================
CMD_TICK = 0.05

FWD_SPEED_MAX, ROT_SPEED_MAX, ROT_GAIN = 0.25, 0.80, 1.0
STOP_CMD = (0.0, 0.0)

MAX_WAYPOINTS = 10
WP_ARRIVE_DIST, WP_TURN_THRESH = 15.0, 0.4

# --- Human-Like Parking Constants (개선된 주차용)
PARKING_APP_SPEED = 0.22          # 주차 준비 위치로 이동할 때 속도
PARKING_REV_FAST = -0.15          # 후진 진입 속도(멀 때)
PARKING_REV_SLOW = -0.05          # 후진 마무리 속도(가까울 때)
SMOOTH_GAIN = 1.2                 # 부드러운 회전 게인
SAFE_ROT_MAX = 0.6                # 안전 회전 최대값
AVOID_DIST = 20.0                 # 주차 중 장애물 감지 거리

# --- [NEW] Rear Parking Controller Gains / Thresholds
PARK_GOAL_DIST_CM = 1.5           # 최종 위치 오차(cm)
PARK_GOAL_ANG_DEG = 4.0           # 최종 각도 오차(deg)
PARK_GOAL_HOLD_SEC = 0.50         # 목표 조건 유지 시간

PARK_ALIGN_ANG_DEG = 10.0         # 정렬 단계 진입 각도 기준(deg)
PARK_ALIGN_DIST_CM = 20.0         # 정렬 단계 진입 거리 기준(cm)
PARK_ALIGN_HOLD_SEC = 0.25

PARK_REV_PROFILE_D = 80.0         # 속도 프로파일 거리 스케일(cm)
PARK_ROT_SLOW_RATIO = 0.55        # 회전이 클수록 속도 감속 비율

# heading + cross-track 제어
PARK_K_HEADING = 1.6              # 각도 오차 게인
PARK_K_CTE = 0.020                # 크로스트랙(옆) 오차 게인 (cm 기반)
PARK_ROT_CLAMP = 0.60             # 후진 중 회전 클램프

# 후진 안전(라이다 후방 섹터)
REAR_STOP_DIST = 18.0             # 뒤쪽 긴급 정지 거리(cm)
REAR_SLOW_DIST = 30.0             # 뒤쪽 감속 거리(cm)

# --- 벽타기 기본 속도
TARGET_DIST = 45.0
FWD_SPEED, CRAWL_SPEED = 0.15, 0.10

# (사용 안 하더라도 기존 상수 유지)
FRONT_LIMIT_ENTER, FRONT_LIMIT_EXIT = 55.0, 70.0
FRONT_AVOID_ENTER, FRONT_AVOID_EXIT = 45.0, 70.0
TURN_90 = math.pi / 2
TURN_W_CORNER, TURN_W_AVOID, TURN_ALIGN_TOL = 0.45, 0.45, 0.06

AVOID_STRAIGHT_V = 0.07

DEADBAND_CM, ROT_CLAMP, WALL_KP, WALL_KD = 4.0, 0.50, 0.010, 0.030
EMA_ALPHA = 0.30

MIN_CORNERS_TO_FINISH, FINISH_RADIUS = 4, 60.0
MIN_MISSION_TIME = 20.0

# 정면/측면 구간 필터 값
DIST_RED = 55.0

# ==========================================
# [PATCH] 벽 너무 붙는 문제 해결용 파라미터 (WP 없을 때 벽타기)
# ==========================================
DIST_GREEN_DETECT = 120.0
DESIRED_WALL_DIST = 65.0
WALL_ERR_DEADBAND = 3.0

WALL_FOLLOW_KP = 0.010
WALL_FOLLOW_ROT_MAX = 0.35

WALL_TOO_CLOSE = 40.0
WALL_PUSH_AWAY_W = 0.18

# ==========================================
# [ADD] Waypoint 주행 중 장애물 회피 + 벽타기 우회
# ==========================================
MIN_VALID_LIDAR = 15.0  # 라이다 노이즈/본체 간섭 컷

WP_AVOID_ENTER = 50.0  # 전방 이 거리 밑이면 회피 시작
WP_AVOID_EXIT = 90.0  # 길 열림 판정
WP_AVOID_DANGER = 25.0  # 매우 위험(전진 금지)

WP_AVOID_COOLDOWN_SEC = 0.7
WP_AVOID_MAX_TIME_SEC = 8.0

WP_AVOID_TURN_W = 0.60
WP_AVOID_ARC_V = 0.10
WP_AVOID_ALIGN_TOL = 0.12

WP_CLEAR_HOLD_SEC = 0.25

# 벽타기 우회 느낌(로청 스타일)
WP_BYPASS_MIN_SEC = 0.6
WP_BYPASS_EXIT_ERR = 0.35
WP_BYPASS_TIMEOUT_SEC = 6.5
WP_BYPASS_FRONT_HARD = 30.0
WP_BYPASS_FWD_MAX = 0.22

# ==========================================
# [미션 종료 조건]
# ==========================================
FINISH_ZONE_RADIUS = 50.0

# GUI/맵 설정(상수)
MAP_WIDTH_CM = 400
MAP_HEIGHT_CM = 200
SIDEBAR_WIDTH = 300
LOG_HEIGHT = 250

# 시작 위치
GUI_START_X = 50.00
GUI_START_Y = 50.00

# ==========================================
# [2. 데이터베이스 관리자]
# ==========================================
Base = declarative_base() if ORM_AVAILABLE else object


class RobotLog(Base):
    __tablename__ = 'robot_log'
    if ORM_AVAILABLE:
        id = Column(Integer, primary_key=True, autoincrement=True)
        log_time = Column(DateTime)
        rel_x = Column(Float)
        rel_y = Column(Float)
        rel_angle = Column(Float)
        action = Column(String(50))
        log_type = Column(String(20))
        message = Column(Text)


class DatabaseManager:
    def __init__(self):
        self.engine = None
        self.Session = None
        self.mock_mode = not ORM_AVAILABLE
        if ORM_AVAILABLE:
            try:
                self.engine = create_engine(DATABASE_URL, pool_recycle=3600)
                Base.metadata.create_all(self.engine)
                self.Session = sessionmaker(bind=self.engine)
                self.mock_mode = False
            except:
                self.mock_mode = True

    def insert_log(self, rel_x, rel_y, rel_angle, action, log_type, message=""):
        if self.mock_mode:
            return
        session = self.Session()
        try:
            new_log = RobotLog(
                rel_x=rel_x, rel_y=rel_y, rel_angle=rel_angle,
                action=action, log_type=str(log_type), message=message,
                log_time=datetime.datetime.now()
            )
            session.add(new_log)
            session.commit()
        except:
            pass
        finally:
            session.close()

    def fetch_logs(self, limit=50):
        if self.mock_mode:
            return []
        session = self.Session()
        try:
            logs = session.query(RobotLog).order_by(desc(RobotLog.id)).limit(limit).all()
            return [(l.id, l.log_time.strftime("%H:%M:%S"), l.action, l.message) for l in reversed(logs)]
        finally:
            session.close()


# ==========================================
# [3. 공유 상태 객체]
# ==========================================
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.start_x, self.start_y, self.start_a = GUI_START_X, GUI_START_Y, 0.0
        self.gui_x, self.gui_y, self.a = GUI_START_X, GUI_START_Y, 0.0

        self.lidar, self.path, self.waypoints, self.wp_index = [], [], [], 0
        self.is_moving, self.running, self.logic_reset_needed = False, True, False

        self.mission_status, self.is_returning, self.parking_mode = "IDLE", False, "FRONT"

        self.view_w = 400.0
        self.view_h = 200.0
        self.roi_points = []

        # 히스테리시스 카운트(정면 빨간구간)
        self.obstacle_count = 0
        self.clear_count = 0

        # Parking micro-correction state
        self.park_phase = 0
        self.park_in_count = 0
        self.park_settle_until = 0.0

        # [NEW] 주차 조건 유지용 타이머
        self.park_goal_since = None
        self.park_align_since = None

        # WP 회피 상태
        self.wp_avoid_active = False
        self.wp_avoid_phase = 0
        self.wp_avoid_target_heading = None
        self.wp_avoid_start_ts = 0.0
        self.wp_avoid_last_end_ts = 0.0
        self.wp_avoid_clear_since = None

        # WP 회피 중 벽타기 우회 상태
        self.wp_bypass_side = None  # "left" or "right"
        self.wp_bypass_start_ts = 0.0


# ==========================================
# [4. 유틸리티]
# ==========================================
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def rotate_point(x, y, theta):
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return x * cos_t - y * sin_t, x * sin_t + y * cos_t


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def robust_right_distance_cm(samples):
    if not samples:
        return None
    s = sorted(samples)
    idx = int(0.2 * (len(s) - 1))
    return s[max(0, idx)]


# --- Waypoint 회피용 섹터 유틸
def sector_min(raw_s, start_idx, end_idx):
    m = 9999.0
    for i in range(start_idx, end_idx + 1):
        d = float(raw_s[i % 360])
        if d < MIN_VALID_LIDAR:
            continue
        if 0 < d < m:
            m = d
    return m


def get_sectors(raw_s):
    front = min(sector_min(raw_s, 330, 359), sector_min(raw_s, 0, 30))
    right, left = sector_min(raw_s, 240, 300), sector_min(raw_s, 60, 120)
    right_front, left_front = sector_min(raw_s, 300, 330), sector_min(raw_s, 30, 60)
    return front, left, right, left_front, right_front


# [NEW] 후진 안전 섹터
def get_rear_sectors(raw_s):
    rear = sector_min(raw_s, 150, 210)
    rear_left = sector_min(raw_s, 210, 240)
    rear_right = sector_min(raw_s, 120, 150)
    return rear, rear_left, rear_right


def wp_choose_turn_dir(front, left, right, left_front, right_front):
    score_left = 1.4 * left_front + 0.8 * left + 0.3 * front
    score_right = 1.4 * right_front + 0.8 * right + 0.3 * front
    return +1.0 if score_left >= score_right else -1.0  # +1: left turn, -1: right turn


def wp_clear_ok(front, left_front, right_front):
    return (front > WP_AVOID_EXIT) and (left_front > WP_AVOID_EXIT * 0.8) and (right_front > WP_AVOID_EXIT * 0.8)


def wp_bypass_wallfollow_step(follow_side, front, left, right, left_front, right_front):
    side_d = right if follow_side == "right" else left

    # 전방이 너무 위험하면: 전진 금지 + 벽 반대쪽으로 강회전
    if front < WP_BYPASS_FRONT_HARD:
        fwd = 0.0
        rot = +0.65 if follow_side == "right" else -0.65
        return fwd, rot

    err = side_d - TARGET_DIST
    if abs(err) < DEADBAND_CM:
        err = 0.0

    sign = -1.0 if follow_side == "right" else +1.0
    rot = clamp(sign * (WALL_KP * err), -0.45, 0.45)

    fwd = 0.10 + 0.0012 * clamp(front - 30.0, 0.0, 150.0)
    fwd = clamp(fwd, 0.08, WP_BYPASS_FWD_MAX)

    fwd *= (1.0 - 0.35 * min(1.0, abs(rot) / 0.45))
    fwd = max(0.08, fwd)

    return fwd, rot


# [NEW] 후진 주차 제어(포즈 기반: heading + cross-track + 속도 프로파일 + 히스테리시스)
def rear_parking_controller(state, raw_s, rel_a, park_x, park_y, park_a, now_ts):
    """
    park_x, park_y: GUI 좌표계 목표 위치(cm)
    park_a: 목표 각도(rad) (GUI 좌표계에서의 절대/상대 방향)
    rel_a: 현재 로봇 각도(rad)
    """
    # 후방 안전 체크
    rear, rear_left, rear_right = get_rear_sectors(raw_s)

    # 목표 포즈 기준 프레임(park frame)에서 로봇 위치 오차 계산
    # park frame: x=전방(park_a), y=좌측
    dx = state.gui_x - park_x
    dy = state.gui_y - park_y
    x_pf, y_pf = rotate_point(dx, dy, -park_a)  # 로봇 위치를 park frame으로

    dist = math.hypot(dx, dy)

    # 각도 오차(최종은 park_a와 정렬)
    heading_err = normalize_angle(park_a - rel_a)

    # 크로스트랙: y_pf (좌우 치우침)
    cte = y_pf

    # 목표 조건(거리+각도) 유지 체크
    goal_ok = (dist <= PARK_GOAL_DIST_CM) and (abs(math.degrees(heading_err)) <= PARK_GOAL_ANG_DEG)

    # Align 조건(좀 멀어도 자세가 맞는지)
    align_ok = (dist <= PARK_ALIGN_DIST_CM) and (abs(math.degrees(heading_err)) <= PARK_ALIGN_ANG_DEG)

    # Settle(정착) 타이밍
    if state.park_settle_until and now_ts < state.park_settle_until:
        return 0.0, 0.0, "PARK: SETTLE"

    # 후방이 너무 가까우면 즉시 정지
    if rear < REAR_STOP_DIST:
        state.park_settle_until = now_ts + 0.25
        return 0.0, 0.0, f"PARK: REAR STOP (rear={rear:.1f})"

    # 후방이 가까우면 속도 제한
    rear_slow_factor = 1.0
    if rear < REAR_SLOW_DIST:
        # rear가 18~30 사이면 0~1로 스케일
        rear_slow_factor = clamp((rear - REAR_STOP_DIST) / max(1e-6, (REAR_SLOW_DIST - REAR_STOP_DIST)), 0.0, 1.0)

    # Phase 기반 (0: pull-up / 1: reverse entry / 2: align & center / 3: settle & stop)
    phase = state.park_phase
    fwd_cmd, rot_cmd = 0.0, 0.0
    mode = ""

    # ---- Phase 0: Pull-up (상대적으로 자유, 전진)
    if phase == 0:
        mode = "PARK: PULL-UP"
        # setup은 park 포즈 기준으로 약간 앞/옆으로(사람 주차처럼)
        setup_x = park_x + 50.0
        setup_y = park_y + 30.0

        ddx, ddy = setup_x - state.gui_x, setup_y - state.gui_y
        d_setup = math.hypot(ddx, ddy)
        t_angle = math.atan2(ddy, ddx)
        a_diff = normalize_angle(t_angle - rel_a)

        # 전방 안전(간단히 front 섹터로 체크)
        front, left, right, left_front, right_front = get_sectors(raw_s)
        if front < AVOID_DIST:
            return 0.0, 0.0, "PARK: PULL-UP BLOCKED"

        if d_setup < 8.0:
            state.park_phase = 1
            state.park_settle_until = now_ts + 0.40
            state.park_align_since = None
            state.park_goal_since = None
            return 0.0, 0.0, "PARK: PULL-UP DONE -> REVERSE"

        rot_cmd = clamp(a_diff * 1.2, -0.6, 0.6)
        fwd_cmd = PARKING_APP_SPEED if abs(a_diff) < 0.7 else 0.0
        return fwd_cmd, rot_cmd, mode

    # ---- Phase 1: Reverse Entry (큰 곡률 허용, 비교적 빠름)
    if phase == 1:
        mode = "PARK: REVERSE ENTRY"

        # Align 조건 유지 체크(히스테리시스)
        if align_ok:
            if state.park_align_since is None:
                state.park_align_since = now_ts
            elif (now_ts - state.park_align_since) >= PARK_ALIGN_HOLD_SEC:
                state.park_phase = 2
                state.park_settle_until = now_ts + 0.20
                state.park_goal_since = None
                return 0.0, 0.0, "PARK: ENTRY -> ALIGN"
        else:
            state.park_align_since = None

        # 후진 조향: 뒤를 바라보는 기준으로 heading + cte 결합
        back_facing = normalize_angle(rel_a + math.pi)
        # 목표점(park 포즈 위치)으로 향하는 벡터 각도(글로벌)
        target_vec = math.atan2(park_y - state.gui_y, park_x - state.gui_x)
        steering_err = normalize_angle(target_vec - back_facing)

        # 보조로 cte를 섞어 흔들림을 줄임(phase1은 약하게)
        rot_cmd = (1.4 * steering_err) + (PARK_K_CTE * 0.5 * cte)
        rot_cmd = clamp(rot_cmd, -PARK_ROT_CLAMP, PARK_ROT_CLAMP)

        # 속도 프로파일(연속 감속) + 회전 연동 감속
        ratio = clamp(dist / PARK_REV_PROFILE_D, 0.0, 1.0)
        v_mag = abs(PARKING_REV_SLOW) + (abs(PARKING_REV_FAST) - abs(PARKING_REV_SLOW)) * ratio
        v_mag *= (1.0 - PARK_ROT_SLOW_RATIO * min(1.0, abs(rot_cmd) / PARK_ROT_CLAMP))
        v = -v_mag

        v *= rear_slow_factor
        return v, rot_cmd, mode

    # ---- Phase 2: Align & Center (느리게, 자세/중심 맞추기)
    if phase == 2:
        mode = "PARK: ALIGN & CENTER"

        # 목표 조건 유지 체크
        if goal_ok:
            if state.park_goal_since is None:
                state.park_goal_since = now_ts
            elif (now_ts - state.park_goal_since) >= PARK_GOAL_HOLD_SEC:
                state.park_phase = 3
                state.park_settle_until = now_ts + 0.30
                return 0.0, 0.0, "PARK: ALIGN DONE -> SETTLE"
        else:
            state.park_goal_since = None

        # heading + cte 제어(phase2는 본격적으로)
        rot_cmd = (PARK_K_HEADING * heading_err) + (PARK_K_CTE * cte)
        rot_cmd = clamp(rot_cmd, -0.45, 0.45)

        # 아주 가까우면 더 느리게
        ratio = clamp(dist / 40.0, 0.0, 1.0)
        v_mag = abs(PARKING_REV_SLOW) + (0.10 - abs(PARKING_REV_SLOW)) * ratio  # 0.10 정도까지
        v_mag *= (1.0 - 0.65 * min(1.0, abs(rot_cmd) / 0.45))
        v = -v_mag

        v *= rear_slow_factor
        return v, rot_cmd, mode

    # ---- Phase 3: Settle & Stop (완전 정지)
    if phase == 3:
        # 최종적으로도 goal_ok가 유지되는지 체크(혹시 밀림/드리프트 방지)
        if not goal_ok:
            # 다시 미세정렬로 복귀
            state.park_phase = 2
            state.park_settle_until = now_ts + 0.20
            state.park_goal_since = None
            return 0.0, 0.0, "PARK: SETTLE FAIL -> ALIGN"

        return 0.0, 0.0, "PARK: PARKED"

    # fallback
    state.park_phase = 1
    return 0.0, 0.0, "PARK: RESET"


# ==========================================
# [5. 주행 스레드 루프]
# ==========================================
_cmd_lock = threading.Lock()
_desired_lin, _desired_ang = 0.0, 0.0


def set_control(lin, ang):
    global _desired_lin, _desired_ang
    with _cmd_lock:
        _desired_lin, _desired_ang = float(lin), float(ang)


def control_sender_thread(state):
    while state.running:
        with _cmd_lock:
            lin, ang = _desired_lin, _desired_ang
        try:
            requests.get(CONTROL_URL, params={'lin': lin, 'ang': ang}, timeout=0.2)
        except:
            pass
        time.sleep(CMD_TICK)


def robot_logic_thread(state, db):
    start_pos = None
    mission_start_ts = None
    set_control(*STOP_CMD)

    while state.running:
        # 정지/리셋
        if not state.is_moving:
            mission_start_ts = None
            if state.logic_reset_needed:
                start_pos = None
                with state.lock:
                    state.wp_index, state.is_returning, state.mission_status = 0, False, "READY"
                    state.start_x, state.start_y, state.start_a = GUI_START_X, GUI_START_Y, 0.0
                    state.obstacle_count = 0
                    state.clear_count = 0

                    state.park_phase = 0
                    state.park_in_count = 0
                    state.park_settle_until = 0.0
                    state.park_goal_since = None
                    state.park_align_since = None

                    state.wp_avoid_active = False
                    state.wp_avoid_phase = 0
                    state.wp_avoid_target_heading = None
                    state.wp_avoid_start_ts = 0.0
                    state.wp_avoid_last_end_ts = 0.0
                    state.wp_avoid_clear_since = None
                    state.wp_bypass_side = None
                    state.wp_bypass_start_ts = 0.0

                state.logic_reset_needed = False

            set_control(*STOP_CMD)
            time.sleep(0.1)
            continue

        # 로봇 상태 읽기
        try:
            resp = requests.get(CONTROL_URL, timeout=0.2)
            data = resp.json() if resp.status_code == 200 else None
        except:
            data = None

        if not data:
            time.sleep(0.2)
            continue

        raw_x = data.get("p", {}).get("x", 0.0)
        raw_y = data.get("p", {}).get("y", 0.0)
        raw_a = data.get("p", {}).get("a", 0.0)
        raw_s = [float(v) for v in data.get("s", [0.0] * 360)]

        if start_pos is None:
            start_pos = {'x': raw_x, 'y': raw_y, 'a': raw_a}
            db.insert_log(0, 0, 0, "INIT_HEADING", "SYSTEM", "Angle Reset to 0")
            continue

        # 상대좌표 계산
        dx_global, dy_global = (raw_x - start_pos['x']) * 100.0, (raw_y - start_pos['y']) * 100.0
        rel_x_cm, rel_y_cm = rotate_point(dx_global, dy_global, -start_pos['a'])
        rel_a = normalize_angle(raw_a - start_pos['a'])

        # 미션 시작 시간
        if state.is_moving and mission_start_ts is None:
            mission_start_ts = time.time()

        # ==========================
        # 라이다 포인트 분류
        # ==========================
        red_zone_pts = []
        right_green_pts = []
        state_roi_points = []

        for i, dist in enumerate(raw_s):
            if not (15.0 < dist < 120.0):
                continue

            angle = i if i < 180 else i - 360
            rad_scan = math.radians(angle)

            if -55 < angle < 55 and dist < 80.0:
                rx = dist * math.cos(rad_scan)
                ry = dist * math.sin(rad_scan)
                state_roi_points.append((rx, ry))

            if -15 < angle < 15 and dist < DIST_RED:
                red_zone_pts.append(dist)
            elif -55 < angle < -15 and dist < DIST_GREEN_DETECT:
                right_green_pts.append(dist)

        # ==========================
        # 공유 상태 업데이트
        # ==========================
        with state.lock:
            state.gui_x, state.gui_y, state.a = GUI_START_X + rel_x_cm, GUI_START_Y + rel_y_cm, math.degrees(rel_a)
            state.lidar = raw_s
            state.roi_points = state_roi_points
            state.path.append((state.gui_x, state.gui_y))

            wp_list = list(state.waypoints)
            wp_i = state.wp_index
            is_returning = state.is_returning
            pmode = state.parking_mode
            park_phase = state.park_phase

            if len(red_zone_pts) > 0:
                state.obstacle_count += 1
                state.clear_count = 0
            else:
                state.clear_count += 1
                state.obstacle_count = 0

            is_blocked = state.obstacle_count >= 3

            wp_avoid_active = state.wp_avoid_active
            wp_phase = state.wp_avoid_phase
            wp_tgt = state.wp_avoid_target_heading
            wp_start = state.wp_avoid_start_ts
            wp_last_end = state.wp_avoid_last_end_ts
            wp_clear_since = state.wp_avoid_clear_since
            bypass_side = state.wp_bypass_side
            bypass_start = state.wp_bypass_start_ts

        # ==========================
        # WP 완료 확인
        # ==========================
        if (len(wp_list) > 0) and (wp_i >= len(wp_list)) and (not is_returning):
            db.insert_log(rel_x_cm, rel_y_cm, rel_a, "WP_DONE", "SYSTEM",
                          f"All waypoints reached (wp_i={wp_i}/{len(wp_list)}) -> RETURN HOME")
            with state.lock:
                state.is_returning = True
                state.park_phase = 0
                state.park_in_count = 0
                state.park_settle_until = 0.0
                state.park_goal_since = None
                state.park_align_since = None
                state.mission_status = "WP_DONE -> RETURN"
            time.sleep(0.05)
            continue

        # ==========================
        # 주행 결정
        # ==========================
        now_ts = time.time()
        fwd_cmd, rot_cmd, current_mode = 0.0, 0.0, ""

        # 1. WAYPOINT 주행 + WP 회피
        if wp_i < len(wp_list):
            front, left, right, left_front, right_front = get_sectors(raw_s)

            cooldown_ok = (now_ts - wp_last_end) > WP_AVOID_COOLDOWN_SEC
            if (not wp_avoid_active) and cooldown_ok and (front < WP_AVOID_ENTER):
                turn_dir = wp_choose_turn_dir(front, left, right, left_front, right_front)
                follow_side = "right" if turn_dir > 0 else "left"
                tgt = normalize_angle(rel_a + turn_dir * math.radians(60))

                with state.lock:
                    state.wp_avoid_active = True
                    state.wp_avoid_phase = 1
                    state.wp_avoid_target_heading = tgt
                    state.wp_avoid_start_ts = now_ts
                    state.wp_avoid_clear_since = None
                    state.wp_bypass_side = follow_side
                    state.wp_bypass_start_ts = now_ts

                db.insert_log(rel_x_cm, rel_y_cm, rel_a, "WP_AVOID_IN", "WP",
                              f"front={front:.1f}, turn={'L' if turn_dir > 0 else 'R'}, bypass_side={follow_side}")

                wp_avoid_active, wp_phase, wp_tgt, wp_start = True, 1, tgt, now_ts
                bypass_side, bypass_start = follow_side, now_ts
                wp_clear_since = None

            if wp_avoid_active:
                current_mode = f"WP_AVOID(P{wp_phase}) front={front:.1f} side={bypass_side}"

                if (now_ts - wp_start) > WP_AVOID_MAX_TIME_SEC:
                    with state.lock:
                        state.wp_avoid_active = False
                        state.wp_avoid_phase = 0
                        state.wp_avoid_target_heading = None
                        state.wp_avoid_last_end_ts = now_ts
                        state.wp_avoid_clear_since = None
                        state.wp_bypass_side = None
                        state.wp_bypass_start_ts = 0.0
                    db.insert_log(rel_x_cm, rel_y_cm, rel_a, "WP_AVOID_TIMEOUT", "WP", "exit avoid by timeout")
                    fwd_cmd, rot_cmd = 0.0, 0.0

                else:
                    if wp_phase == 1:
                        heading_err = normalize_angle(wp_tgt - rel_a)
                        rot_cmd = clamp(heading_err * 1.2, -WP_AVOID_TURN_W, WP_AVOID_TURN_W)
                        fwd_cmd = 0.0 if front < WP_AVOID_DANGER else WP_AVOID_ARC_V
                        if abs(heading_err) < WP_AVOID_ALIGN_TOL:
                            with state.lock:
                                state.wp_avoid_phase = 2
                                state.wp_avoid_clear_since = None
                            wp_phase = 2

                    elif wp_phase == 2:
                        fwd_cmd, rot_cmd = wp_bypass_wallfollow_step(
                            bypass_side, front, left, right, left_front, right_front
                        )
                        tx, ty = wp_list[wp_i]
                        dxw, dyw = tx - state.gui_x, ty - state.gui_y
                        wp_ang = math.atan2(dyw, dxw)
                        wp_err = normalize_angle(wp_ang - rel_a)

                        can_try_exit = (now_ts - bypass_start) >= WP_BYPASS_MIN_SEC
                        if can_try_exit and wp_clear_ok(front, left_front, right_front) and abs(wp_err) < WP_BYPASS_EXIT_ERR:
                            with state.lock:
                                if state.wp_avoid_clear_since is None:
                                    state.wp_avoid_clear_since = now_ts
                                    wp_clear_since = now_ts
                            if wp_clear_since is not None and (now_ts - wp_clear_since) >= WP_CLEAR_HOLD_SEC:
                                with state.lock:
                                    state.wp_avoid_phase = 3
                                    state.wp_avoid_clear_since = None
                                wp_phase = 3
                        else:
                            with state.lock:
                                state.wp_avoid_clear_since = None
                            wp_clear_since = None

                        if (now_ts - bypass_start) > WP_BYPASS_TIMEOUT_SEC:
                            with state.lock:
                                state.wp_avoid_phase = 3
                                state.wp_avoid_clear_since = None
                            wp_phase = 3

                    elif wp_phase == 3:
                        tx, ty = wp_list[wp_i]
                        dxw, dyw = tx - state.gui_x, ty - state.gui_y
                        wp_ang = math.atan2(dyw, dxw)
                        wp_err = normalize_angle(wp_ang - rel_a)
                        rot_cmd = clamp(wp_err * 1.2, -0.6, 0.6)
                        fwd_cmd = 0.06 if abs(wp_err) < 0.9 else 0.0
                        if abs(wp_err) < 0.25:
                            with state.lock:
                                state.wp_avoid_phase = 4
                            wp_phase = 4

                    else:
                        with state.lock:
                            state.wp_avoid_active = False
                            state.wp_avoid_phase = 0
                            state.wp_avoid_target_heading = None
                            state.wp_avoid_last_end_ts = now_ts
                            state.wp_avoid_clear_since = None
                            state.wp_bypass_side = None
                            state.wp_bypass_start_ts = 0.0
                        db.insert_log(rel_x_cm, rel_y_cm, rel_a, "WP_AVOID_OUT", "WP", "resume waypoint")
                        fwd_cmd, rot_cmd = 0.0, 0.0

                with state.lock:
                    state.mission_status = current_mode

                set_control(fwd_cmd, rot_cmd)

                if state.is_moving and (int(time.time() * 20) % 10 == 0):
                    db.insert_log(rel_x_cm, rel_y_cm, rel_a, "MOVING", "TRAJ",
                                  f"V:{fwd_cmd:.2f}, W:{rot_cmd:.2f}, Mode:{current_mode}")

                time.sleep(CMD_TICK)
                continue

            tx, ty = wp_list[wp_i]
            current_mode = f"GOTO WP#{wp_i + 1}"

            dxw, dyw = tx - state.gui_x, ty - state.gui_y
            dist_wp = math.hypot(dxw, dyw)
            target_angle = math.atan2(dyw, dxw)
            angle_diff = normalize_angle(target_angle - rel_a)

            if dist_wp <= WP_ARRIVE_DIST:
                db.insert_log(rel_x_cm, rel_y_cm, rel_a, "WP_ARRIVE", "CHECKPOINT", f"Reached WP #{wp_i + 1}")
                with state.lock:
                    state.wp_index += 1
                set_control(*STOP_CMD)
                time.sleep(0.3)
                continue

            rot_cmd = clamp(angle_diff * ROT_GAIN, -ROT_SPEED_MAX, ROT_SPEED_MAX)
            fwd_cmd = FWD_SPEED_MAX * max(0.0, 1.0 - (abs(angle_diff) / 1.2))
            if dist_wp < 20.0:
                fwd_cmd *= 0.6
            if abs(angle_diff) > WP_TURN_THRESH:
                fwd_cmd = 0.0

        # 2. 벽타기 (WP 없음)
        elif len(wp_list) == 0 and not is_returning:
            dist_to_start = math.hypot(state.gui_x - GUI_START_X, state.gui_y - GUI_START_Y)
            elapsed_time = (time.time() - mission_start_ts) if mission_start_ts else 0.0

            if elapsed_time > MIN_MISSION_TIME and dist_to_start < FINISH_RADIUS:
                db.insert_log(rel_x_cm, rel_y_cm, rel_a, "FINISH_APPROACH", "SYSTEM",
                              f"Elapsed {elapsed_time:.1f}s. Near Start Pos. Parking...")
                with state.lock:
                    state.is_returning = True
                    state.park_phase = 0
                    state.park_in_count = 0
                    state.park_settle_until = 0.0
                    state.park_goal_since = None
                    state.park_align_since = None
                continue

            if is_blocked:
                fwd_cmd = AVOID_STRAIGHT_V
                rot_cmd = 0.50
                current_mode = "AVOID_OBSTACLE (LEFT)"

            elif len(right_green_pts) > 0:
                right_d = robust_right_distance_cm(right_green_pts)
                if right_d is None:
                    fwd_cmd = CRAWL_SPEED
                    rot_cmd = -0.25
                    current_mode = "WALL_FOLLOWING (NO DATA -> FIND)"
                else:
                    err = right_d - DESIRED_WALL_DIST
                    if abs(err) < WALL_ERR_DEADBAND:
                        err = 0.0

                    rot_cmd = clamp(-WALL_FOLLOW_KP * err, -WALL_FOLLOW_ROT_MAX, WALL_FOLLOW_ROT_MAX)

                    if right_d < WALL_TOO_CLOSE:
                        rot_cmd = clamp(rot_cmd + WALL_PUSH_AWAY_W, -ROT_SPEED_MAX, ROT_SPEED_MAX)

                    rot_ratio = min(1.0, abs(rot_cmd) / max(1e-6, WALL_FOLLOW_ROT_MAX))
                    fwd_cmd = FWD_SPEED * (1.0 - 0.45 * rot_ratio)
                    fwd_cmd = max(CRAWL_SPEED, min(FWD_SPEED, fwd_cmd))

                    current_mode = f"WALL_FOLLOWING d={right_d:.1f} err={err:.1f}"

            else:
                fwd_cmd = CRAWL_SPEED
                rot_cmd = -0.25
                current_mode = "FINDING_WALL (RIGHT)"

        # 3. 복귀 및 주차 (개선된 Human-Like Rear Parking)
        elif is_returning:
            PARK_X, PARK_Y = GUI_START_X, GUI_START_Y

            # 목표 각도(포즈) 추가: 기본은 시작 각도(=0)로 정렬
            # 필요하면 여기서 환경/벽 추정으로 PARK_A를 바꿔도 됨
            PARK_A = 0.0  # rad

            if pmode == "FRONT":
                current_mode = "FRONT PARKING"
                dx, dy = PARK_X - state.gui_x, PARK_Y - state.gui_y
                dist = math.hypot(dx, dy)
                target_angle = math.atan2(dy, dx)
                angle_diff = normalize_angle(target_angle - rel_a)

                if dist <= 1.5:
                    db.insert_log(rel_x_cm, rel_y_cm, rel_a, "PARKED", "SYSTEM", "Front Parked")
                    with state.lock:
                        state.is_moving, state.is_returning = False, False
                    set_control(*STOP_CMD)
                    continue

                rot_cmd = clamp(angle_diff * SMOOTH_GAIN, -SAFE_ROT_MAX, SAFE_ROT_MAX)
                fwd_cmd = PARKING_APP_SPEED * 0.5 if dist < 10.0 else PARKING_APP_SPEED
                if abs(angle_diff) > (0.8 if dist < 10.0 else 0.4):
                    fwd_cmd = 0.0

            elif pmode == "REAR":
                fwd_cmd, rot_cmd, current_mode = rear_parking_controller(
                    state=state,
                    raw_s=raw_s,
                    rel_a=rel_a,
                    park_x=PARK_X,
                    park_y=PARK_Y,
                    park_a=PARK_A,
                    now_ts=now_ts
                )

                # 최종 종료 처리
                if current_mode == "PARK: PARKED":
                    db.insert_log(rel_x_cm, rel_y_cm, rel_a, "PARKED", "SYSTEM", "Rear Parked (Pose+Hold)")
                    with state.lock:
                        state.is_moving, state.is_returning = False, False
                    set_control(*STOP_CMD)
                    continue

        with state.lock:
            state.mission_status = current_mode

        set_control(fwd_cmd, rot_cmd)

        if state.is_moving and (int(time.time() * 20) % 10 == 0):
            db.insert_log(rel_x_cm, rel_y_cm, rel_a, "MOVING", "TRAJ",
                          f"V:{fwd_cmd:.2f}, W:{rot_cmd:.2f}, Mode:{current_mode}")

        time.sleep(CMD_TICK)
