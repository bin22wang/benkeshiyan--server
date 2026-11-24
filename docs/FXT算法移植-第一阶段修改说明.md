# FXT 固定时间编队控制算法移植 — 第一阶段修改说明

> 修改日期：2026-10-16
> 阶段：第一阶段（最小可验证，仅改上位机，下位机不动）
> 算法来源：`D:\desktop\课题\实验平台验证\FXT_V3\`（MATLAB 仿真代码）

---

## 1. 修改目标

将 MATLAB 验证的 FXT（Fixed-Time Consensus）固定时间控制律移植到实验平台，
替换矩形轨迹实验中领航者的简单比例速度给定，实现固定时间位置跟踪。

本阶段仅实现**领航者单车跟踪**，跟随者编队控制留待第二阶段。

---

## 2. 修改文件

| 文件 | 改动类型 |
|------|---------|
| `Server_Web_UDP_26_7_14/formation_controller.py` | 新增 FXT 参数、控制律、状态机、路由（不修改原有矩形轨迹代码） |

下位机固件（`Main_Controller/`）**未修改**，前端（`templates/index.html`）**未修改**。

---

## 3. 新增内容详解

### 3.1 FXT 控制参数（硬编码常量）

对应 MATLAB `init_simulation.m:71-92`，位于 `formation_controller.py` 矩形参数之后：

| 参数 | 值 | 说明 | MATLAB 对应 |
|------|----|------|-------------|
| `FXT_ALPHA_P` | 0.5 | 位置误差幂次（0<α<1） | `ctrl.alpha_p` |
| `FXT_BETA_P` | 1.5 | 位置误差幂次（β>1） | `ctrl.beta_p` |
| `FXT_ALPHA_V` | 0.667 | 速度误差幂次（=2α/(1+α)） | `ctrl.alpha_v` |
| `FXT_BETA_V` | 1.2 | 速度误差幂次（=2β/(1+β)） | `ctrl.beta_v` |
| `FXT_K_P1` | 4.5 | 位置非线性项增益1 | `ctrl.k_p1` |
| `FXT_K_P2` | 2.0 | 位置非线性项增益2 | `ctrl.k_p2` |
| `FXT_K_V1` | 3.2 | 速度非线性项增益1 | `ctrl.k_v1` |
| `FXT_K_V2` | 1.4 | 速度非线性项增益2 | `ctrl.k_v2` |
| `FXT_COMMAND_HZ` | 20.0 | 控制频率（Hz） | `1/sys.dt` |
| `FXT_DATA_HZ` | 10.0 | 数据采集频率（Hz） | — |
| `FXT_MAX_SPEED` | 0.7 | 速度限幅（m/s） | — |

> 注：航向控制参数（`k_theta*`, `k_omega*`）本阶段未使用，航向仍由下位机 yaw PID 控制。

### 3.2 FXT 状态字典 `fxt_state`

与 `rect_state` 结构类似，新增字段：
- `ref_pos` / `ref_vel`：参考轨迹位置与速度（移动参考点）
- `cmd_vel`：积分得到的速度指令 `[vx, vy]`
- `prev_heading`：上一帧航向，用于角速度 ω 数值微分

### 3.3 核心函数

| 函数 | 作用 | MATLAB 对应 |
|------|------|-------------|
| `_sig_pow(e, gamma)` | sign(e)·\|e\|^γ，固定时间非线性项 | `formation_controller.m:122-125` |
| `_get_leader_full_state()` | 获取领航者全状态 (x,y,vx,vy,θ,ω)，ω 用数值微分估算 | — |
| `_fxt_compute_accel(e_pos, e_vel, ref_acc)` | FXT 位置控制律，输出加速度 (ux, uy) | `formation_controller.m:39-42` |
| `_fxt_state_loop()` | 20Hz 控制线程，状态机 + FXT 律 + 速度积分 | `main.m:18-50` 主循环 |
| `_fxt_data_loop()` | 10Hz 数据采集线程，记录位姿+速度+指令 | — |
| `_finalize_fxt_experiment()` | 结束实验，保存 CSV（`fxt_traj_{car}_{时间戳}.csv`） | — |
| `_start_fxt_experiment()` / `_stop_fxt_experiment()` | 启停控制 | — |
| `get_fxt_experiment_info()` | 查询状态（供互锁使用） | — |

### 3.4 控制律公式

**FXT 固定时间位置控制律**（对应 MATLAB `formation_controller.m:39-42`）：

```
e_pos = [x - ref_x, y - ref_y]          # 位置误差
e_vel = [vx - ref_vx, vy - ref_vy]      # 速度误差

u_x = ref_ax - k_p1·sign(e_pos_x)^α_p - k_p2·sign(e_pos_x)^β_p
            - k_v1·sign(e_vel_x)^α_v - k_v2·sign(e_vel_x)^β_v
u_y = ref_ay - k_p1·sign(e_pos_y)^α_p - k_p2·sign(e_pos_y)^β_p
            - k_v1·sign(e_vel_y)^α_v - k_v2·sign(e_vel_y)^β_v
```

**二阶→一阶适配（方案A）**：FXT 输出加速度，上位机积分成速度后下发：

```
vx_cmd += u_x · dt
vy_cmd += u_y · dt
下发指令: [T, leader, vx_cmd, vy_cmd, target_yaw]
```

### 3.5 状态机流程

复用矩形轨迹的三阶段状态机，但 move 阶段速度由 FXT 律计算：

```
homing → rotate → (move → rotate) × 4 → completed
```

| 阶段 | 参考位置 | 参考速度 | 航向控制 | 切换条件 |
|------|---------|---------|---------|---------|
| homing | 最近矩形顶点（固定） | 0 | 目标顶点航向 | 距离 < 0.15m |
| rotate | 当前位置（保持） | 0 | 下位机 yaw PID | 航向误差 < 5° |
| move | 沿段方向匀速推进 | speed·方向 | 段航向 | 参考点投影超过段长 |

### 3.6 新增 API 路由

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/formation/fxt/start` | 启动 FXT 实验，参数 `speed`（0.05~0.7 m/s） |
| POST | `/api/formation/fxt/stop` | 停止并保存 CSV |
| GET | `/api/formation/fxt/status` | 查询状态 |
| GET | `/api/formation/fxt/download/<filename>` | 下载轨迹 CSV |

---

## 4. 与原矩形轨迹实验的关键差异

| 维度 | 原矩形轨迹 | FXT 实验（本阶段） |
|------|-----------|-------------------|
| 速度计算 | 简单比例 `speed·方向分量` | FXT 固定时间律 + 加速度积分 |
| 控制频率 | 10 Hz | 20 Hz |
| 状态反馈 | 仅位置 (x, y, heading) | 位置 + 速度 (vx, vy) + ω 数值微分 |
| 理论保证 | 无收敛性保证 | 固定时间收敛，T_max 与初始状态无关 |
| 指令格式 | `[T,leader,vx,vy,yaw]` | 相同（下位机无需改） |
| CSV 字段 | timestamp,car_id,x,y,yaw,set_vx,set_vy | 额外增加 vx,vy（实际速度） |

---

## 5. 已知限制（待后续阶段解决）

1. **仅领航者控制**：跟随者编队控制未实现（第二阶段）
2. **航向非 FXT**：航向仍由下位机 yaw PID 控制，未用 FXT 航向律
3. **ω 数值微分**：角速度通过 `Δheading/Δt` 估算，精度有限（第三阶段改为下位机上报）
4. **参数硬编码**：FXT 参数不可前端调节（第二阶段加 UI）
5. **无互锁**：FXT 实验期间未锁定单车控制/遥控（第二阶段在 `control.py` 加互锁）
6. **增益未整定**：MATLAB 增益在 1000Hz 下整定，平台 20Hz 直接套用可能需调参

---

## 6. 验证方法

1. 设置领航者：通过编队控制界面指定领航者
2. 调用启动接口：
   ```bash
   curl -X POST http://localhost:5000/api/formation/fxt/start \
        -H "Content-Type: application/json" \
        -d '{"speed": 0.2}'
   ```
3. 观察领航者是否沿矩形轨迹运动，收敛行为是否符合固定时间特性
4. 调用停止接口，下载 CSV 分析轨迹：
   ```bash
   curl -X POST http://localhost:5000/api/formation/fxt/stop
   curl http://localhost:5000/api/formation/fxt/status
   ```
5. 对比 `fxt_traj_*.csv` 与 `rect_traj_*.csv`，分析位置/速度误差收敛曲线

---

## 7. 后续阶段预告

| 阶段 | 内容 | 涉及文件 |
|------|------|---------|
| 第二阶段 | 跟随者 FXT 编队 + 槽位分配 + 前端参数面板 + 互锁 | `formation_controller.py`, `index.html`, `control.py` |
| 第三阶段 | 下位机上报 ω + 可选加速度指令模式 | `uart.c`, `app_main.c`, `net.py`, `models.py`, `vel_solver.h/c`, `proto_parser.c` |
