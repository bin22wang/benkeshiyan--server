import csv
import json
import math
import os
import threading
import time
from datetime import datetime
from flask import Blueprint, request, jsonify, send_file

formation_bp = Blueprint('formation', __name__)

formation_enabled = False
formation_leader = None
formation_type = "line"  # line, Diamond, square, custom
formation_params = {}
cars_dict = {} 
udp_server = None 

RECT_POINTS = [
    {"x": 0.75, "y": 0.75, "yaw": 0.0},
    {"x": 2.25, "y": 0.75, "yaw": 90.0},
    {"x": 2.25, "y": 2.25, "yaw": 180.0},
    {"x": 0.75, "y": 2.25, "yaw": -90.0}
]
RECT_DISTANCE_THRESHOLD = 0.15
RECT_YAW_THRESHOLD_DEG = 5.0
RECT_COMMAND_HZ = 10.0
RECT_DATA_HZ = 10.0
RECT_LOG_DIRNAME = "logs"

rect_lock = threading.Lock()
rect_state = {
    "active": False,
    "stop_requested": False,
    "phase": "idle",
    "leader_id": None,
    "start_time": None,
    "idx": None,
    "segment_idx": 0,
    "speed": 0.2,
    "last_cmd": {},
    "records": {},
    "thread": None,
    "data_thread": None,
    "finalized": False,
    "last_files": [],
    "last_error": None
}

FXT_ALPHA_P = 0.5
FXT_BETA_P = 1.5
FXT_ALPHA_V = 2 * FXT_ALPHA_P / (1 + FXT_ALPHA_P)
FXT_BETA_V = 2 * FXT_BETA_P / (1 + FXT_BETA_P)
FXT_K_P1 = 0.5
FXT_K_P2 = 0.3
FXT_K_V1 = 0.4
FXT_K_V2 = 0.2
FXT_COMMAND_HZ = 20.0
FXT_DATA_HZ = 10.0
FXT_LOG_DIRNAME = "logs"
FXT_MAX_SPEED = 0.3

FXT_FORMATION_SLOTS = {
    "diamond": [[0.4, 0.0], [0.4, 0.4], [0.0, 0.4]],
    "triangle": [[0.5, 0.0], [-0.25, 0.433], [-0.25, -0.433]],
    "line": [[0.3, 0.0], [-0.3, 0.0], [0.6, 0.0]]
}

fxt_params = {
    "alpha_p": 0.5,
    "beta_p": 1.5,
    "k_p1": 0.5,
    "k_p2": 0.3,
    "k_v1": 0.4,
    "k_v2": 0.2,
    "formation_type": "diamond"
}

fxt_lock = threading.Lock()
fxt_state = {
    "active": False,
    "stop_requested": False,
    "phase": "idle",
    "leader_id": None,
    "start_time": None,
    "idx": None,
    "segment_idx": 0,
    "speed": 0.2,
    "ref_pos": [0.0, 0.0],
    "ref_vel": [0.0, 0.0],
    "cmd_vel": [0.0, 0.0],
    "prev_heading": None,
    "records": {},
    "thread": None,
    "data_thread": None,
    "finalized": False,
    "last_files": [],
    "last_error": None,
    "assigned_slots": {},
    "follower_cmd_vel": {},
    "follower_prev_heading": {},
    "formation_type": "diamond"
}

FORMATION_CONFIGS = {
    "line": {
        "CAR1": {"x": 0, "y": 0, "yaw": 0},
        "CAR2": {"x": -0.7, "y": 0, "yaw": 0},
        "CAR3": {"x": -1.4, "y": 0, "yaw": 0},
        "CAR4": {"x": -2.1, "y": 0, "yaw": 0}
    },
    "Diamond": {
        "CAR1": {"x": 0, "y": 0, "yaw": 0},
        "CAR2": {"x": -0.7, "y": -0.7, "yaw": 0},
        "CAR3": {"x": -0.7, "y": 0.7, "yaw": 0},
        "CAR4": {"x": -1.4, "y": 0, "yaw": 0}
    },
    "square": {
        "CAR1": {"x": 0, "y": 0, "yaw": 0},
        "CAR2": {"x": 0, "y": -0.7, "yaw": 0},
        "CAR3": {"x": -0.7, "y": -0.7, "yaw": 0},
        "CAR4": {"x": -0.7, "y": 0, "yaw": 0}
    }
}

def _build_offsets_for_leader(formation_type_name, leader_id):
    base_offsets = FORMATION_CONFIGS.get(formation_type_name, FORMATION_CONFIGS["line"])
    leader_base = base_offsets.get(leader_id)
    if not leader_base:
        return base_offsets
    

    adjusted = {}
    for car_id, offset in base_offsets.items():
        adjusted[car_id] = {
            "x": offset["x"] - leader_base["x"],
            "y": offset["y"] - leader_base["y"],
            "yaw": offset.get("yaw", 0) - leader_base.get("yaw", 0)
        }
    return adjusted

def init_formation_controller(cars, server):
    global cars_dict, udp_server
    cars_dict = cars
    udp_server = server
    print("🔧 编队控制器初始化完成")

def _ensure_rect_log_dir():
    log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), RECT_LOG_DIRNAME))
    os.makedirs(log_dir, exist_ok=True)
    return log_dir

def _wrap_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle

def _get_leader_pose(leader_id):
    if not leader_id or leader_id not in cars_dict:
        return None
    car = cars_dict.get(leader_id)
    if not car or not car.connected:
        return None
    return car.position.get("x", 0.0), car.position.get("y", 0.0), car.heading

def _send_rectangle_command(leader_id, vx, vy, target_yaw):
    if not udp_server:
        return False
    cmd = f"[T,{leader_id},{vx:.3f},{vy:.3f},{target_yaw:.1f}]"
    return udp_server.send_to_car(leader_id, cmd)

def _update_rect_last_cmd(car_id, vx, vy):
    with rect_lock:
        rect_state["last_cmd"][car_id] = {"vx": float(vx), "vy": float(vy)}

def _select_nearest_point(x, y):
    best_idx = 0
    best_dist = None
    for idx, point in enumerate(RECT_POINTS):
        dx = point["x"] - x
        dy = point["y"] - y
        dist = (dx * dx + dy * dy) ** 0.5
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx

def _finalize_rect_experiment(reason=None):
    with rect_lock:
        if rect_state["finalized"]:
            return list(rect_state.get("last_files", []))
        rect_state["finalized"] = True
        rect_state["active"] = False
        rect_state["stop_requested"] = False
        rect_state["phase"] = "idle"
        if reason in ("completed", "manual_stop", "stopped"):
            rect_state["last_error"] = None
        else:
            rect_state["last_error"] = reason
        records_by_car = dict(rect_state["records"])

    log_dir = _ensure_rect_log_dir()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    saved_paths = []

    for car_id, rows in records_by_car.items():
        if not rows:
            continue
        filename = f"rect_traj_{car_id}_{timestamp}.csv"
        save_path = os.path.join(log_dir, filename)
        try:
            with open(save_path, "w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(["timestamp", "car_id", "x", "y", "yaw", "set_vx", "set_vy"])
                for row in rows:
                    writer.writerow(row)
            saved_paths.append(save_path)
        except Exception as exc:
            with rect_lock:
                rect_state["last_error"] = str(exc)
            return None

    with rect_lock:
        rect_state["last_files"] = list(saved_paths)
        rect_state["records"] = {}
        rect_state["last_cmd"] = {}

    return list(saved_paths)

def _rect_data_loop():
    interval = 1.0 / RECT_DATA_HZ
    while True:
        with rect_lock:
            if not rect_state["active"]:
                break
            last_cmd_snapshot = dict(rect_state["last_cmd"])
        current_ts = time.time()
        ts_str = datetime.fromtimestamp(current_ts).isoformat(timespec="milliseconds")
        rows_by_car = {}
        for car_id, car in list(cars_dict.items()):
            if not car.connected:
                continue
            cmd = last_cmd_snapshot.get(car_id, {"vx": 0.0, "vy": 0.0})
            rows_by_car.setdefault(car_id, []).append([
                ts_str,
                car_id,
                round(car.position.get("x", 0.0), 4),
                round(car.position.get("y", 0.0), 4),
                round(car.heading, 2),
                round(cmd["vx"], 4),
                round(cmd["vy"], 4)
            ])
        if rows_by_car:
            with rect_lock:
                for car_id, rows in rows_by_car.items():
                    rect_state["records"].setdefault(car_id, []).extend(rows)
        time.sleep(interval)

def _rect_state_loop():
    interval = 1.0 / RECT_COMMAND_HZ
    while True:
        with rect_lock:
            if not rect_state["active"]:
                break
            if rect_state["stop_requested"]:
                break
            leader_id = rect_state["leader_id"]
            phase = rect_state["phase"]
            idx = rect_state["idx"]
            segment_idx = rect_state["segment_idx"]
            speed = rect_state["speed"]

        pose = _get_leader_pose(leader_id)
        if not pose:
            _finalize_rect_experiment("leader_disconnected")
            break
        x, y, yaw = pose

        yaw_rad = math.radians(yaw)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        vx, vy = 0.0, 0.0
        target_yaw = yaw

        abs_speed = abs(speed)

        if phase == "homing":
            target = RECT_POINTS[idx]
            dx = target["x"] - x
            dy = target["y"] - y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < RECT_DISTANCE_THRESHOLD:
                with rect_lock:
                    rect_state["phase"] = "rotate"
                    rect_state["segment_idx"] = 0
            else:
                if dist > 0:
                    forward = cos_yaw * dx + sin_yaw * dy
                    left = -sin_yaw * dx + cos_yaw * dy
                    vx = abs_speed * forward / dist
                    vy = abs_speed * left / dist
        elif phase == "rotate":
            current = RECT_POINTS[(idx + segment_idx) % len(RECT_POINTS)]
            target_yaw = current["yaw"]
            yaw_error = _wrap_angle_deg(target_yaw - yaw)
            if abs(yaw_error) <= RECT_YAW_THRESHOLD_DEG:
                with rect_lock:
                    rect_state["phase"] = "move"
            else:
                vx, vy = 0.0, 0.0
        elif phase == "move":
            current = RECT_POINTS[(idx + segment_idx) % len(RECT_POINTS)]
            target_yaw = current["yaw"]
            next_point = RECT_POINTS[(idx + segment_idx + 1) % len(RECT_POINTS)]
            dx = next_point["x"] - x
            dy = next_point["y"] - y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < RECT_DISTANCE_THRESHOLD:
                next_segment = segment_idx + 1
                if next_segment >= len(RECT_POINTS):
                    _finalize_rect_experiment("completed")
                    break
                with rect_lock:
                    rect_state["segment_idx"] = next_segment
                    rect_state["phase"] = "rotate"
            else:
                if dist > 0:
                    forward = cos_yaw * dx + sin_yaw * dy
                    left = -sin_yaw * dx + cos_yaw * dy
                    vx = abs_speed * forward / dist
                    vy = abs_speed * left / dist

        _update_rect_last_cmd(leader_id, vx, vy)
        _send_rectangle_command(leader_id, vx, vy, target_yaw)
        time.sleep(interval)

    _finalize_rect_experiment("stopped")

def _start_rectangle_experiment(speed):
    leader_id = formation_leader
    pose = _get_leader_pose(leader_id)
    if not pose:
        return False, "leader_unavailable"

    x, y, _ = pose
    idx = _select_nearest_point(x, y)

    with rect_lock:
        if rect_state["active"]:
            return False, "already_running"
        rect_state["active"] = True
        rect_state["stop_requested"] = False
        rect_state["phase"] = "homing"
        rect_state["leader_id"] = leader_id
        rect_state["start_time"] = time.time()
        rect_state["idx"] = idx
        rect_state["segment_idx"] = 0
        rect_state["speed"] = speed
        rect_state["records"] = {}
        rect_state["last_cmd"] = {}
        rect_state["finalized"] = False
        rect_state["last_files"] = []
        rect_state["last_error"] = None

    rect_thread = threading.Thread(target=_rect_state_loop, daemon=True)
    data_thread = threading.Thread(target=_rect_data_loop, daemon=True)
    with rect_lock:
        rect_state["thread"] = rect_thread
        rect_state["data_thread"] = data_thread
    rect_thread.start()
    data_thread.start()
    return True, {"idx": idx, "leader_id": leader_id}

def _stop_rectangle_experiment():
    with rect_lock:
        if not rect_state["active"]:
            return False, "not_running"
        rect_state["stop_requested"] = True
        leader_id = rect_state["leader_id"]

    pose = _get_leader_pose(leader_id)
    if pose:
        _, _, yaw = pose
        _send_rectangle_command(leader_id, 0.0, 0.0, yaw)
        _update_rect_last_cmd(leader_id, 0.0, 0.0)

    deadline = time.time() + 2.5
    while time.time() < deadline:
        with rect_lock:
            if not rect_state["active"]:
                break
        time.sleep(0.05)

    save_paths = _finalize_rect_experiment("manual_stop")
    return True, {"save_paths": save_paths or []}

def get_rectangle_experiment_info():
    with rect_lock:
        return {
            "active": rect_state["active"],
            "leader_id": rect_state["leader_id"],
            "phase": rect_state["phase"],
            "last_files": list(rect_state["last_files"]),
            "last_error": rect_state["last_error"]
        }

def _sig_pow(e, gamma):
    if e == 0.0:
        return 0.0
    return math.copysign(abs(e) ** gamma, e)

def _get_leader_full_state(leader_id, prev_heading, dt):
    if not leader_id or leader_id not in cars_dict:
        return None
    car = cars_dict.get(leader_id)
    if not car or not car.connected:
        return None
    x = car.position.get("x", 0.0)
    y = car.position.get("y", 0.0)
    theta = car.heading
    vx = car.velocity.get("vx", 0.0)
    vy = car.velocity.get("vy", 0.0)
    vz = car.velocity.get("vz", 0.0)
    omega = 0.0
    if abs(vz) > 1e-6:
        omega = vz * 180.0 / math.pi
    elif prev_heading is not None and dt > 0:
        omega = _wrap_angle_deg(theta - prev_heading) / dt
    return x, y, vx, vy, theta, omega

def _fxt_compute_accel(e_pos, e_vel, ref_acc, params=None):
    if params is None:
        params = fxt_params
    alpha_p = params["alpha_p"]
    beta_p = params["beta_p"]
    alpha_v = 2 * alpha_p / (1 + alpha_p)
    beta_v = 2 * beta_p / (1 + beta_p)
    k_p1 = params["k_p1"]
    k_p2 = params["k_p2"]
    k_v1 = params["k_v1"]
    k_v2 = params["k_v2"]
    ux = ref_acc[0] \
         - k_p1 * _sig_pow(e_pos[0], alpha_p) - k_p2 * _sig_pow(e_pos[0], beta_p) \
         - k_v1 * _sig_pow(e_vel[0], alpha_v) - k_v2 * _sig_pow(e_vel[0], beta_v)
    uy = ref_acc[1] \
         - k_p1 * _sig_pow(e_pos[1], alpha_p) - k_p2 * _sig_pow(e_pos[1], beta_p) \
         - k_v1 * _sig_pow(e_vel[1], alpha_v) - k_v2 * _sig_pow(e_vel[1], beta_v)
    return ux, uy

def _fxt_local_assignment(dist_mat):
    num_robot = len(dist_mat)
    num_slot = len(dist_mat[0]) if num_robot > 0 else 0
    if num_robot == 0 or num_slot == 0 or num_robot > num_slot:
        return []
    from itertools import permutations
    best_cost = float('inf')
    best_slots = list(range(num_robot))
    for perm in permutations(range(num_slot), num_robot):
        cost = sum(dist_mat[i][perm[i]] for i in range(num_robot))
        if cost < best_cost:
            best_cost = cost
            best_slots = list(perm)
    return [(i, best_slots[i]) for i in range(num_robot)]

def _finalize_fxt_experiment(reason=None):
    with fxt_lock:
        if fxt_state["finalized"]:
            return list(fxt_state.get("last_files", []))
        fxt_state["finalized"] = True
        fxt_state["active"] = False
        fxt_state["stop_requested"] = False
        fxt_state["phase"] = "idle"
        if reason in ("completed", "manual_stop", "stopped"):
            fxt_state["last_error"] = None
        else:
            fxt_state["last_error"] = reason
        records_by_car = dict(fxt_state["records"])

    log_dir = _ensure_rect_log_dir()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    saved_paths = []

    for car_id, rows in records_by_car.items():
        if not rows:
            continue
        filename = f"fxt_traj_{car_id}_{timestamp}.csv"
        save_path = os.path.join(log_dir, filename)
        try:
            with open(save_path, "w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(["timestamp", "car_id", "x", "y", "yaw", "vx", "vy", "set_vx", "set_vy"])
                for row in rows:
                    writer.writerow(row)
            saved_paths.append(save_path)
        except Exception as exc:
            with fxt_lock:
                fxt_state["last_error"] = str(exc)
            return None

    with fxt_lock:
        fxt_state["last_files"] = list(saved_paths)
        fxt_state["records"] = {}

    return list(saved_paths)

def _fxt_data_loop():
    interval = 1.0 / FXT_DATA_HZ
    while True:
        with fxt_lock:
            if not fxt_state["active"]:
                break
            cmd_vel = list(fxt_state["cmd_vel"])
        current_ts = time.time()
        ts_str = datetime.fromtimestamp(current_ts).isoformat(timespec="milliseconds")
        rows_by_car = {}
        for car_id, car in list(cars_dict.items()):
            if not car.connected:
                continue
            rows_by_car.setdefault(car_id, []).append([
                ts_str,
                car_id,
                round(car.position.get("x", 0.0), 4),
                round(car.position.get("y", 0.0), 4),
                round(car.heading, 2),
                round(car.velocity.get("vx", 0.0), 4),
                round(car.velocity.get("vy", 0.0), 4),
                round(cmd_vel[0], 4),
                round(cmd_vel[1], 4)
            ])
        if rows_by_car:
            with fxt_lock:
                for car_id, rows in rows_by_car.items():
                    fxt_state["records"].setdefault(car_id, []).extend(rows)
        time.sleep(interval)

def _fxt_state_loop():
    dt = 1.0 / FXT_COMMAND_HZ
    interval = dt

    while True:
        with fxt_lock:
            if not fxt_state["active"]:
                break
            if fxt_state["stop_requested"]:
                break
            leader_id = fxt_state["leader_id"]
            phase = fxt_state["phase"]
            idx = fxt_state["idx"]
            segment_idx = fxt_state["segment_idx"]
            speed = fxt_state["speed"]
            ref_pos = list(fxt_state["ref_pos"])
            ref_vel = list(fxt_state["ref_vel"])
            cmd_vel = list(fxt_state["cmd_vel"])
            prev_heading = fxt_state["prev_heading"]

        state = _get_leader_full_state(leader_id, prev_heading, dt)
        if not state:
            _finalize_fxt_experiment("leader_disconnected")
            break
        x, y, vx, vy, yaw, omega = state

        abs_speed = abs(speed)
        target_yaw = yaw
        ref_acc = [0.0, 0.0]

        if phase == "homing":
            target = RECT_POINTS[idx]
            ref_pos = [target["x"], target["y"]]
            ref_vel = [0.0, 0.0]
            target_yaw = target["yaw"]
            dx = target["x"] - x
            dy = target["y"] - y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < RECT_DISTANCE_THRESHOLD:
                with fxt_lock:
                    fxt_state["phase"] = "rotate"
                    fxt_state["segment_idx"] = 0
        elif phase == "rotate":
            current = RECT_POINTS[(idx + segment_idx) % len(RECT_POINTS)]
            target_yaw = current["yaw"]
            ref_pos = [x, y]
            ref_vel = [0.0, 0.0]
            yaw_error = _wrap_angle_deg(target_yaw - yaw)
            if abs(yaw_error) <= RECT_YAW_THRESHOLD_DEG:
                with fxt_lock:
                    fxt_state["phase"] = "move"
                    cur_pt = RECT_POINTS[(idx + segment_idx) % len(RECT_POINTS)]
                    fxt_state["ref_pos"] = [cur_pt["x"], cur_pt["y"]]
                    fxt_state["ref_vel"] = [0.0, 0.0]
        elif phase == "move":
            current = RECT_POINTS[(idx + segment_idx) % len(RECT_POINTS)]
            next_point = RECT_POINTS[(idx + segment_idx + 1) % len(RECT_POINTS)]
            target_yaw = current["yaw"]
            seg_vec_x = next_point["x"] - current["x"]
            seg_vec_y = next_point["y"] - current["y"]
            seg_len = (seg_vec_x * seg_vec_x + seg_vec_y * seg_vec_y) ** 0.5
            if seg_len > 1e-6:
                dir_x = seg_vec_x / seg_len
                dir_y = seg_vec_y / seg_len
            else:
                dir_x, dir_y = 0.0, 0.0
            ref_vel = [abs_speed * dir_x, abs_speed * dir_y]
            ref_pos[0] += ref_vel[0] * dt
            ref_pos[1] += ref_vel[1] * dt

            proj = (ref_pos[0] - current["x"]) * seg_vec_x + (ref_pos[1] - current["y"]) * seg_vec_y
            if proj >= seg_len * seg_len:
                next_segment = segment_idx + 1
                if next_segment >= len(RECT_POINTS):
                    _finalize_fxt_experiment("completed")
                    break
                with fxt_lock:
                    fxt_state["segment_idx"] = next_segment
                    fxt_state["phase"] = "rotate"
                    fxt_state["ref_pos"] = [next_point["x"], next_point["y"]]
                    fxt_state["ref_vel"] = [0.0, 0.0]
                ref_pos = [next_point["x"], next_point["y"]]
                ref_vel = [0.0, 0.0]

        e_pos = [x - ref_pos[0], y - ref_pos[1]]
        e_vel = [vx - ref_vel[0], vy - ref_vel[1]]
        ux, uy = _fxt_compute_accel(e_pos, e_vel, ref_acc)

        cmd_vel[0] += ux * dt
        cmd_vel[1] += uy * dt

        cmd_speed = (cmd_vel[0] ** 2 + cmd_vel[1] ** 2) ** 0.5
        if cmd_speed > FXT_MAX_SPEED:
            cmd_vel[0] *= FXT_MAX_SPEED / cmd_speed
            cmd_vel[1] *= FXT_MAX_SPEED / cmd_speed

        if phase == "rotate":
            cmd_vel[0] = 0.0
            cmd_vel[1] = 0.0

        _send_rectangle_command(leader_id, cmd_vel[0], cmd_vel[1], target_yaw)

        leader_accel = [ux, uy]

        with fxt_lock:
            assigned_slots = dict(fxt_state["assigned_slots"])
            follower_cmd_vel = {k: list(v) for k, v in fxt_state["follower_cmd_vel"].items()}
            formation_type = fxt_state.get("formation_type", "diamond")

        slots = FXT_FORMATION_SLOTS.get(formation_type, FXT_FORMATION_SLOTS["diamond"])

        for car_id, slot_idx in assigned_slots.items():
            if car_id not in cars_dict or not cars_dict[car_id].connected:
                continue
            car = cars_dict[car_id]
            fx = car.position.get("x", 0.0)
            fy = car.position.get("y", 0.0)
            fvx = car.velocity.get("vx", 0.0)
            fvy = car.velocity.get("vy", 0.0)

            slot = slots[slot_idx % len(slots)]
            des_pos = [x + slot[0], y + slot[1]]
            des_vel = [vx, vy]
            des_acc = leader_accel

            e_pos_f = [fx - des_pos[0], fy - des_pos[1]]
            e_vel_f = [fvx - des_vel[0], fvy - des_vel[1]]
            ufx, ufy = _fxt_compute_accel(e_pos_f, e_vel_f, des_acc)

            fcv = follower_cmd_vel.get(car_id, [0.0, 0.0])
            fcv[0] += ufx * dt
            fcv[1] += ufy * dt

            fcv_speed = (fcv[0] ** 2 + fcv[1] ** 2) ** 0.5
            if fcv_speed > FXT_MAX_SPEED:
                fcv[0] *= FXT_MAX_SPEED / fcv_speed
                fcv[1] *= FXT_MAX_SPEED / fcv_speed

            if phase == "rotate":
                fcv[0] = 0.0
                fcv[1] = 0.0

            follower_cmd_vel[car_id] = fcv
            _send_rectangle_command(car_id, fcv[0], fcv[1], target_yaw)

        with fxt_lock:
            fxt_state["ref_pos"] = list(ref_pos)
            fxt_state["ref_vel"] = list(ref_vel)
            fxt_state["cmd_vel"] = list(cmd_vel)
            fxt_state["prev_heading"] = yaw
            fxt_state["follower_cmd_vel"] = {k: list(v) for k, v in follower_cmd_vel.items()}

        time.sleep(interval)

    _finalize_fxt_experiment("stopped")

def _start_fxt_experiment(speed):
    leader_id = formation_leader
    pose = _get_leader_pose(leader_id)
    if not pose:
        return False, "leader_unavailable"

    x, y, _ = pose
    idx = _select_nearest_point(x, y)

    formation_type = fxt_params.get("formation_type", "diamond")
    slots = FXT_FORMATION_SLOTS.get(formation_type, FXT_FORMATION_SLOTS["diamond"])

    followers = []
    for car_id, car in cars_dict.items():
        if car_id == leader_id or not car.connected:
            continue
        followers.append(car_id)

    assigned_slots = {}
    if followers and slots:
        dist_mat = []
        for f_id in followers:
            f_car = cars_dict[f_id]
            row = []
            for slot in slots:
                slot_pos = [x + slot[0], y + slot[1]]
                dx = f_car.position.get("x", 0.0) - slot_pos[0]
                dy = f_car.position.get("y", 0.0) - slot_pos[1]
                row.append((dx * dx + dy * dy) ** 0.5)
            dist_mat.append(row)
        assignment = _fxt_local_assignment(dist_mat)
        for follower_idx, slot_idx in assignment:
            assigned_slots[followers[follower_idx]] = slot_idx

    with fxt_lock:
        if fxt_state["active"]:
            return False, "already_running"
        fxt_state["active"] = True
        fxt_state["stop_requested"] = False
        fxt_state["phase"] = "homing"
        fxt_state["leader_id"] = leader_id
        fxt_state["start_time"] = time.time()
        fxt_state["idx"] = idx
        fxt_state["segment_idx"] = 0
        fxt_state["speed"] = speed
        fxt_state["ref_pos"] = [x, y]
        fxt_state["ref_vel"] = [0.0, 0.0]
        fxt_state["cmd_vel"] = [0.0, 0.0]
        fxt_state["prev_heading"] = None
        fxt_state["records"] = {}
        fxt_state["finalized"] = False
        fxt_state["last_files"] = []
        fxt_state["last_error"] = None
        fxt_state["assigned_slots"] = assigned_slots
        fxt_state["follower_cmd_vel"] = {f_id: [0.0, 0.0] for f_id in assigned_slots}
        fxt_state["follower_prev_heading"] = {}
        fxt_state["formation_type"] = formation_type

    ctrl_thread = threading.Thread(target=_fxt_state_loop, daemon=True)
    data_thread = threading.Thread(target=_fxt_data_loop, daemon=True)
    with fxt_lock:
        fxt_state["thread"] = ctrl_thread
        fxt_state["data_thread"] = data_thread
    ctrl_thread.start()
    data_thread.start()
    return True, {"idx": idx, "leader_id": leader_id, "assigned_slots": assigned_slots}

def _stop_fxt_experiment():
    with fxt_lock:
        if not fxt_state["active"]:
            return False, "not_running"
        fxt_state["stop_requested"] = True
        leader_id = fxt_state["leader_id"]
        assigned_slots = dict(fxt_state["assigned_slots"])

    pose = _get_leader_pose(leader_id)
    if pose:
        _, _, yaw = pose
        _send_rectangle_command(leader_id, 0.0, 0.0, yaw)

    for car_id in assigned_slots:
        if car_id in cars_dict and cars_dict[car_id].connected:
            f_pose = _get_leader_pose(car_id)
            f_yaw = f_pose[2] if f_pose else 0.0
            _send_rectangle_command(car_id, 0.0, 0.0, f_yaw)

    deadline = time.time() + 2.5
    while time.time() < deadline:
        with fxt_lock:
            if not fxt_state["active"]:
                break
        time.sleep(0.05)

    save_paths = _finalize_fxt_experiment("manual_stop")
    return True, {"save_paths": save_paths or []}

def get_fxt_experiment_info():
    with fxt_lock:
        return {
            "active": fxt_state["active"],
            "leader_id": fxt_state["leader_id"],
            "phase": fxt_state["phase"],
            "last_files": list(fxt_state["last_files"]),
            "last_error": fxt_state["last_error"]
        }

def send_formation_command(car_id, command):
    if udp_server:
        return udp_server.send_to_car_reliable(car_id, command, max_retries=4)
    else:
        print(f"⚠️ UDP服务器未初始化，无法发送指令给 {car_id}")
        return False

@formation_bp.route('/api/formation/start', methods=['POST'])
def start_formation():
    global formation_enabled, formation_leader, formation_type
    data = request.json
    leader_id = data.get('leader_id')
    formation_type = data.get('formation_type', 'line')
    if not leader_id:
        return jsonify({'success': False, 'error': '需要指定领航者'})
    if leader_id not in cars_dict or not cars_dict[leader_id].connected:
        return jsonify({'success': False, 'error': f'领航者 {leader_id} 未连接'})
    print(f"🚀 启动编队控制 - 领航者: {leader_id}, 队形: {formation_type}")
    
    formation_offsets = _build_offsets_for_leader(formation_type, leader_id)
        
    old_leader = formation_leader
    formation_leader = leader_id
    formation_enabled = True
    print("🎯 直接启动编队，不发送停止指令")
    
    success_count = 0
    total_cars = 0
    for car_id in cars_dict:
        if not cars_dict[car_id].connected:
            continue
        total_cars += 1
        if car_id == leader_id:
            start_cmd = f"[F,S,{leader_id},{formation_type}]"
            leader_role_cmd = f"[F,L,{car_id}]"
            if send_formation_command(car_id, start_cmd):
                print(f"🎯 向领航者 {car_id} 发送开始指令: {start_cmd}")
                if send_formation_command(car_id, leader_role_cmd):
                    print(f"🎯 向领航者 {car_id} 发送角色指令: {leader_role_cmd}")
                    success_count += 1
        else:
            start_cmd = f"[F,S,{leader_id},{formation_type}]"
            offset = formation_offsets.get(car_id, {"x": 0, "y": 0, "yaw": 0})
            follower_cmd = f"[F,F,{leader_id},{offset['x']},{offset['y']},{offset['yaw']}]"
            if send_formation_command(car_id, start_cmd):
                print(f"🎯 向跟随者 {car_id} 发送开始指令: {start_cmd}")
                if send_formation_command(car_id, follower_cmd):
                    print(f"🎯 向跟随者 {car_id} 发送偏移指令: {follower_cmd}")
                    success_count += 1
                    
    if old_leader and old_leader != leader_id and old_leader in cars_dict:
        if cars_dict[old_leader].connected:
            start_cmd = f"[F,S,{leader_id},{formation_type}]"
            offset = formation_offsets.get(old_leader, {"x": 0, "y": 0, "yaw": 0})
            follower_cmd = f"[F,F,{leader_id},{offset['x']},{offset['y']},{offset['yaw']}]"
            if send_formation_command(old_leader, start_cmd) and send_formation_command(old_leader, follower_cmd):
                print(f"🔄 原领航者 {old_leader} 转换为跟随者")
                
    unicast_success_rate = (success_count / total_cars * 100) if total_cars > 0 else 0
    return jsonify({
        'success': True,
        'message': f'编队控制已启动 - 领航者: {leader_id}, 队形: {formation_type}',
        'formation_leader': formation_leader,
        'formation_type': formation_type,
        'formation_offsets': formation_offsets,
        'unicast_success_count': success_count,
        'total_cars': total_cars,
        'success_rate': f'{unicast_success_rate:.1f}%'
    })

@formation_bp.route('/api/formation/stop', methods=['POST'])
def stop_formation():
    global formation_enabled
    stop_cmd = "[F,T]"
    success_count = 0
    total_cars = 0
    for car_id in cars_dict:
        if cars_dict[car_id].connected:
            total_cars += 1
            if send_formation_command(car_id, stop_cmd):
                success_count += 1
    formation_enabled = False
    unicast_success_rate = (success_count / total_cars * 100) if total_cars > 0 else 0
    print(f"🛑 编队控制已停止，单播发送停止指令: {success_count}/{total_cars} 成功")
    return jsonify({
        'success': True,
        'message': '编队控制已停止',
        'unicast_success_count': success_count,
        'total_cars': total_cars,
        'success_rate': f'{unicast_success_rate:.1f}%'
    })

@formation_bp.route('/api/formation/status')
def get_formation_status():
    return jsonify({
        'formation_enabled': formation_enabled,
        'formation_leader': formation_leader,
        'formation_type': formation_type
    })

@formation_bp.route('/api/formation/rectangle/start', methods=['POST'])
def start_rectangle_experiment():
    data = request.json or {}
    speed = float(data.get('speed', 0.2))

    if not formation_leader:
        return jsonify({'success': False, 'error': '未设置领航者'}), 400

    with rect_lock:
        if rect_state["active"]:
            return jsonify({'success': False, 'error': '矩形实验进行中'}), 400

    speed = abs(speed)
    speed = max(0.05, min(speed, 0.7))
    success, result = _start_rectangle_experiment(speed)
    if not success:
        return jsonify({'success': False, 'error': result}), 400

    return jsonify({
        'success': True,
        'message': '矩形轨迹实验已启动',
        'leader_id': result["leader_id"],
        'start_idx': result["idx"],
        'speed': speed
    })

@formation_bp.route('/api/formation/rectangle/stop', methods=['POST'])
def stop_rectangle_experiment():
    success, result = _stop_rectangle_experiment()
    if not success:
        return jsonify({'success': False, 'error': result}), 400

    save_paths = result.get("save_paths", [])
    files = [os.path.basename(path) for path in save_paths if path]
    download_urls = [f"/api/formation/rectangle/download/{name}" for name in files]

    return jsonify({
        'success': True,
        'message': '矩形轨迹实验已停止并保存',
        'files': files,
        'download_urls': download_urls
    })

@formation_bp.route('/api/formation/rectangle/status')
def get_rectangle_experiment_status():
    info = get_rectangle_experiment_info()
    files = [os.path.basename(path) for path in info.get("last_files", []) if path]
    download_urls = [f"/api/formation/rectangle/download/{name}" for name in files]
    info.update({'download_urls': download_urls, 'files': files})
    return jsonify(info)

@formation_bp.route('/api/formation/rectangle/download/<filename>')
def download_rectangle_file(filename):
    safe_name = os.path.basename(filename)
    if not safe_name:
        return jsonify({'success': False, 'error': '无效文件名'}), 400
    log_dir = _ensure_rect_log_dir()
    file_path = os.path.join(log_dir, safe_name)
    if not os.path.exists(file_path):
        return jsonify({'success': False, 'error': '文件不存在'}), 404
    return send_file(file_path, as_attachment=True, download_name=safe_name)

@formation_bp.route('/api/formation/fxt/start', methods=['POST'])
def start_fxt_experiment():
    data = request.json or {}
    speed = float(data.get('speed', 0.2))

    if not formation_leader:
        return jsonify({'success': False, 'error': '未设置领航者'}), 400

    with fxt_lock:
        if fxt_state["active"]:
            return jsonify({'success': False, 'error': 'FXT实验进行中'}), 400

    speed = abs(speed)
    speed = max(0.05, min(speed, 0.7))
    success, result = _start_fxt_experiment(speed)
    if not success:
        return jsonify({'success': False, 'error': result}), 400

    return jsonify({
        'success': True,
        'message': 'FXT矩形轨迹实验已启动',
        'leader_id': result["leader_id"],
        'start_idx': result["idx"],
        'speed': speed
    })

@formation_bp.route('/api/formation/fxt/stop', methods=['POST'])
def stop_fxt_experiment():
    success, result = _stop_fxt_experiment()
    if not success:
        return jsonify({'success': False, 'error': result}), 400

    save_paths = result.get("save_paths", [])
    files = [os.path.basename(path) for path in save_paths if path]
    download_urls = [f"/api/formation/fxt/download/{name}" for name in files]

    return jsonify({
        'success': True,
        'message': 'FXT矩形轨迹实验已停止并保存',
        'files': files,
        'download_urls': download_urls
    })

@formation_bp.route('/api/formation/fxt/status')
def get_fxt_experiment_status():
    info = get_fxt_experiment_info()
    files = [os.path.basename(path) for path in info.get("last_files", []) if path]
    download_urls = [f"/api/formation/fxt/download/{name}" for name in files]
    info.update({'download_urls': download_urls, 'files': files})
    return jsonify(info)

@formation_bp.route('/api/formation/fxt/download/<filename>')
def download_fxt_file(filename):
    safe_name = os.path.basename(filename)
    if not safe_name:
        return jsonify({'success': False, 'error': '无效文件名'}), 400
    log_dir = _ensure_rect_log_dir()
    file_path = os.path.join(log_dir, safe_name)
    if not os.path.exists(file_path):
        return jsonify({'success': False, 'error': '文件不存在'}), 404
    return send_file(file_path, as_attachment=True, download_name=safe_name)

@formation_bp.route('/api/formation/fxt/params', methods=['GET', 'POST'])
def fxt_params_handler():
    if request.method == 'GET':
        with fxt_lock:
            return jsonify({"success": True, "params": dict(fxt_params)})
    data = request.json or {}
    with fxt_lock:
        for key in ["alpha_p", "beta_p", "k_p1", "k_p2", "k_v1", "k_v2", "formation_type"]:
            if key in data:
                fxt_params[key] = data[key]
    return jsonify({"success": True, "params": dict(fxt_params)})

@formation_bp.route('/api/formation/custom', methods=['POST'])
def set_custom_formation():
    global formation_enabled, formation_leader
    data = request.json
    custom_offsets = data.get('offsets', {})
    leader_id = data.get('leader_id')
    if not custom_offsets or not leader_id:
        return jsonify({'success': False, 'error': '需要提供领航者ID和编队偏移量'})
    if leader_id not in cars_dict or not cars_dict[leader_id].connected:
        return jsonify({'success': False, 'error': f'领航者 {leader_id} 未连接'})
        
    formation_leader = leader_id
    formation_enabled = True
    print(f"🔧 设置自定义编队 - 领航者: {leader_id}, 偏移量: {custom_offsets}")
    
    success_count = 0
    total_cars = 0
    for car_id in cars_dict:
        if not cars_dict[car_id].connected:
            continue
        total_cars += 1
        if car_id == leader_id:
            start_cmd = f"FORMATION:CUSTOM,{leader_id}"
            leader_cmd = "FORMATION:LEADER,CUSTOM"
            if send_formation_command(car_id, start_cmd) and send_formation_command(car_id, leader_cmd):
                success_count += 1
        else:
            start_cmd = f"FORMATION:CUSTOM,{leader_id}"
            offset = custom_offsets.get(car_id, {"x": 0, "y": 0, "yaw": 0})
            follower_cmd = f"FORMATION:FOLLOWER,{leader_id},{offset['x']},{offset['y']},{offset['yaw']}"
            if send_formation_command(car_id, start_cmd) and send_formation_command(car_id, follower_cmd):
                success_count += 1
                
    unicast_success_rate = (success_count / total_cars * 100) if total_cars > 0 else 0
    return jsonify({
        'success': True,
        'message': '自定义编队已设置',
        'formation_leader': formation_leader,
        'formation_offsets': custom_offsets,
        'unicast_success_count': success_count,
        'total_cars': total_cars,
        'success_rate': f'{unicast_success_rate:.1f}%'
    })

@formation_bp.route('/api/formation/configs')
def get_formation_configs():
    return jsonify({
        'success': True,
        'formation_configs': FORMATION_CONFIGS
    })

@formation_bp.route('/api/formation/update_offsets', methods=['POST'])
def update_formation_offsets():
    global formation_enabled, formation_leader
    if not formation_enabled:
        return jsonify({'success': False, 'error': '编队控制未启动'})
    data = request.json
    new_offsets = data.get('offsets', {})
    if not new_offsets:
        return jsonify({'success': False, 'error': '需要提供新的偏移量'})
        
    print(f"🔄 更新编队偏移量: {new_offsets}")
    success_count = 0
    total_cars = 0
    for car_id, offset in new_offsets.items():
        if car_id in cars_dict and car_id != formation_leader and cars_dict[car_id].connected:
            total_cars += 1
            update_cmd = f"[F,U,{formation_leader},{offset['x']},{offset['y']},{offset['yaw']}]"
            if send_formation_command(car_id, update_cmd):
                print(f"🔄 向小车 {car_id} 发送偏移更新: {update_cmd}")
                success_count += 1
                
    success_rate = (success_count / total_cars * 100) if total_cars > 0 else 0
    return jsonify({
        'success': True,
        'message': f'编队偏移量已更新，通知 {success_count}/{total_cars} 辆小车',
        'updated_cars': success_count,
        'success_rate': f'{success_rate:.1f}%'
    })

def get_formation_info():
    return {
        'enabled': formation_enabled,
        'leader': formation_leader,
        'type': formation_type
    }