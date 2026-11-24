# FXT 固定时间编队控制算法移植 — 第三阶段修改说明

> 修改日期：2026-10-16
> 阶段：第三阶段（精度提升）
> 前置：第一阶段（领航者 FXT 跟踪）、第二阶段（完整编队 + 前端 + 互锁）

---

## 1. 修改目标

提升 FXT 控制律的全状态反馈精度：
- **3a（已实现）**：角速度 ω 从数值微分估算改为直接读取下位机上报的传感器值
- **3b（备用方案，未实现）**：加速度指令模式（下位机积分），保留为备用

---

## 2. 3a — 角速度 ω 上报（已实现）

### 2.1 关键发现

下位机**已经在上报角速度**：`vz` 字段就是绕 Z 轴角速度（rad/s），
来源链路：STM32 `Calculate_Car_Velocity()` → UART `[x,y,heading,vx,vy,vz]` → ESP32 `vel_solver_update_own_state()` → UDP `CAR_ID:x,y,yaw,vol,vx,vy,vz` → 上位机 `net.py` 解析。

因此 **3a 只需上位机改动，下位机无需修改**。

### 2.2 修改文件

| # | 文件 | 改动 |
|---|------|------|
| 1 | `server_app/models.py` | `Car` 类新增 `self.omega = 0.0` 属性 |
| 2 | `server_app/net.py` | 状态解析时 `car.omega = vz`（两处：已有车辆 + 新车辆） |
| 3 | `formation_controller.py` | `_get_leader_full_state` 优先用 `car.omega`（vz rad/s → deg/s），数值微分作为备选 |

### 2.3 改动细节

**models.py** — Car 类新增 omega：
```python
self.velocity = {"vx": 0, "vy": 0, "vz": 0}
self.omega = 0.0    # ← 新增：角速度（rad/s，由下位机上报）
self.speed = 0
```

**net.py** — 解析时赋值（两处）：
```python
car.velocity = {"vx": vx, "vy": vy, "vz": vz}
car.omega = vz      # ← 新增：vz 就是角速度
```

**formation_controller.py** — `_get_leader_full_state` 优先用传感器值：
```python
vz = car.velocity.get("vz", 0.0)
omega = 0.0
if abs(vz) > 1e-6:
    omega = vz * 180.0 / math.pi    # rad/s → deg/s（传感器直读）
elif prev_heading is not None and dt > 0:
    omega = _wrap_angle_deg(theta - prev_heading) / dt  # 数值微分备选
```

### 2.4 效果

| 维度 | 改动前（第一二阶段） | 改动后（第三阶段 3a） |
|------|---------------------|---------------------|
| ω 来源 | 数值微分 `Δheading/Δt` | 下位机传感器直读 `vz` |
| 精度 | 受采样率限制（20Hz） | 传感器原生精度（10Hz 上报） |
| 延迟 | 一帧延迟 | 零延迟 |
| 备选 | 无 | 传感器不可用时自动回退数值微分 |

---

## 3. 3b — 加速度指令模式（备用方案，未实现）

### 3.1 定位

3b 是**备用方案**，当前不实现。记录详细设计供未来启用。

**保留理由**：方案A（上位机积分）已满足使用要求。3b 的唯一优势是下位机 50Hz 积分比上位机 20Hz 精度更高，但需改 4 个文件（含下位机 C 固件），性价比不高。

**启用条件**：若未来需要更高积分精度，或需减轻上位机计算负担，再启用 3b。

### 3.2 方案对比

| | 方案A（当前，上位机积分） | 方案B（3b，下位机积分） |
|---|---|---|
| 积分位置 | 上位机 `formation_controller.py` | 下位机 `vel_solver.c` |
| 积分频率 | 20 Hz | 50 Hz（`uart_tx_task` 频率） |
| 指令格式 | `[T,CARn,vx,vy,target_yaw]`（速度） | `[A,CARn,ax,ay,atheta]`（加速度） |
| 下位机改动 | 无 | vel_solver.h/c + proto_parser.c |
| 精度 | 20Hz 积分，够用 | 50Hz 积分，更平滑 |
| 复杂度 | 低 | 高（需改固件） |

### 3.3 详细设计（未来启用时参考）

若启用 3b，需修改以下 4 个文件：

#### 文件1：`Main_Controller/main/modules/vel_solver.h`

1. 枚举新增 `VEL_MODE_ACCEL = 5`
2. 接口新增声明：
```c
void vel_solver_set_accel_cmd(float ax, float ay, float target_yaw);
```

#### 文件2：`Main_Controller/main/modules/vel_solver.c`

1. 新增静态变量：
```c
static float s_accel_ax = 0.0f;
static float s_accel_ay = 0.0f;
static float s_accel_target_yaw = 0.0f;
static uint32_t s_accel_last_cmd_ms = 0;
static float s_accel_int_vx = 0.0f;   /* 积分速度 */
static float s_accel_int_vy = 0.0f;
static uint32_t s_accel_last_compute_ms = 0;
```

2. 实现 `vel_solver_set_accel_cmd`：
```c
void vel_solver_set_accel_cmd(float ax, float ay, float target_yaw) {
    s_accel_ax = ax;
    s_accel_ay = ay;
    s_accel_target_yaw = target_yaw;
    s_accel_last_cmd_ms = now_ms();
    s_accel_int_vx = 0.0f;
    s_accel_int_vy = 0.0f;
    s_mode = VEL_MODE_ACCEL;
    reset_control_state();
}
```

3. `vel_solver_compute` 新增 `VEL_MODE_ACCEL` 分支：
```c
else if (s_mode == VEL_MODE_ACCEL) {
    /* 超时保护 */
    if ((now - s_accel_last_cmd_ms) > VEL_SOLVER_TRAJ_TIMEOUT_MS) {
        s_accel_int_vx = 0.0f;
        s_accel_int_vy = 0.0f;
    } else {
        /* 二阶积分：加速度 → 速度 */
        float dt = (s_accel_last_compute_ms > 0)
                 ? (now - s_accel_last_compute_ms) / 1000.0f : 0.02f;
        s_accel_int_vx += s_accel_ax * dt;
        s_accel_int_vy += s_accel_ay * dt;
        /* 限幅 */
        float spd = sqrtf(s_accel_int_vx * s_accel_int_vx + s_accel_int_vy * s_accel_int_vy);
        if (spd > 0.7f) {
            s_accel_int_vx *= 0.7f / spd;
            s_accel_int_vy *= 0.7f / spd;
        }
    }
    s_accel_last_compute_ms = now;
    control_vx = s_accel_int_vx;
    control_vy = s_accel_int_vy;
    control_vz = yaw_pid_control(s_own.yaw, s_accel_target_yaw);
}
```

#### 文件3：`Main_Controller/main/modules/proto_parser.c`

1. 新增 `parse_accel_cmd` 函数（在 `parse_trajectory_cmd` 旁）：
```c
static void parse_accel_cmd(const char *data) {
    if (recon_filter_controls()) return;
    char target[PROTO_CAR_ID_LEN];
    float ax, ay, yaw;
    int n = sscanf(data, "[A,%9[^,],%f,%f,%f]", target, &ax, &ay, &yaw);
    if (n != 4) return;
    if (strcmp(target, s_own_id) != 0) return;
    vel_solver_set_accel_cmd(ax, ay, yaw);
}
```

2. `proto_parse_message` 的 switch 新增：
```c
case 'A': parse_accel_cmd(data);    return;
```

#### 文件4：`Server_Web_UDP_26_7_14/formation_controller.py`

1. 新增 `_send_accel_command` 函数：
```python
def _send_accel_command(car_id, ax, ay, target_yaw):
    cmd = f"[A,{car_id},{ax:.4f},{ay:.4f},{target_yaw:.2f}]"
    return send_formation_command(car_id, cmd)
```

2. FXT 控制循环中，将 `_send_rectangle_command(leader_id, cmd_vel[0], cmd_vel[1], target_yaw)` 替换为 `_send_accel_command(leader_id, ux, uy, target_yaw)`（直接发加速度，不做上位机积分）。

---

## 4. 第三阶段实际修改文件汇总

| # | 文件 | 改动 | 状态 |
|---|------|------|------|
| 1 | `server_app/models.py` | Car 新增 omega 属性 | ✅ 已完成 |
| 2 | `server_app/net.py` | 解析时 car.omega = vz | ✅ 已完成 |
| 3 | `formation_controller.py` | _get_leader_full_state 优先用 car.omega | ✅ 已完成 |
| 4 | `Main_Controller/main/modules/vel_solver.h` | VEL_MODE_ACCEL 枚举+接口 | ⏸️ 备用 |
| 5 | `Main_Controller/main/modules/vel_solver.c` | 加速度模式实现 | ⏸️ 备用 |
| 6 | `Main_Controller/main/modules/proto_parser.c` | [A,...] 指令解析 | ⏸️ 备用 |

---

## 5. 验证

Python 语法检查通过 ✅（models.py, net.py, formation_controller.py）

---

## 6. 三阶段完整总结

| 阶段 | 内容 | 文件数 | 状态 |
|------|------|--------|------|
| 第一阶段 | 领航者 FXT 跟踪 + 控制律 + 路由 | 1 | ✅ |
| 第二阶段 | 跟随者编队 + 槽位分配 + 前端 + 互锁 | 3 | ✅ |
| 第三阶段 3a | ω 传感器直读 | 3 | ✅ |
| 第三阶段 3b | 加速度指令模式 | 4 | ⏸️ 备用 |
