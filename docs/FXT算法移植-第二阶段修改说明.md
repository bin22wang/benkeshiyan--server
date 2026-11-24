# FXT 固定时间编队控制算法移植 — 第二阶段修改说明

> 修改日期：2026-10-16
> 阶段：第二阶段（完整编队 + 前端参数面板 + 互锁）
> 前置：第一阶段已实现领航者单车 FXT 跟踪

---

## 1. 修改目标

在第一阶段基础上扩展为**完整的多车 FXT 编队控制**：
- 跟随者使用 FXT 控制律跟踪领航者+槽位偏移
- 槽位分配算法（局部最优）
- 前端参数可调面板
- FXT 实验期间互锁单车控制

---

## 2. 修改文件

| # | 文件 | 改动类型 |
|---|------|---------|
| 1 | `Server_Web_UDP_26_7_14/formation_controller.py` | 新增槽位分配、可调参数、跟随者控制律、参数路由 |
| 2 | `Server_Web_UDP_26_7_14/templates/index.html` | 新增 FXT 参数面板、JS 函数、状态轮询 |
| 3 | `Server_Web_UDP_26_7_14/server_app/routes/control.py` | 新增 FXT 实验互锁 |

下位机固件**未修改**。

---

## 3. formation_controller.py 改动详解

### 3.1 新增队形槽位定义 `FXT_FORMATION_SLOTS`

对应 MATLAB `init_simulation.m:56-68`：

| 队形 | 槽位偏移 |
|------|---------|
| diamond（菱形） | `[1,0], [1,1], [0,1]` |
| triangle（三角形） | `[2,0], [-1,√3], [-1,-√3]` |
| line（直线） | `[1,0], [-1,0], [2,0]` |

### 3.2 新增可调参数字典 `fxt_params`

替代第一阶段的硬编码常量，支持前端动态设置：

```python
fxt_params = {
    "alpha_p": 0.5, "beta_p": 1.5,
    "k_p1": 4.5, "k_p2": 2.0, "k_v1": 3.2, "k_v2": 1.4,
    "formation_type": "diamond"
}
```

### 3.3 修改 `_fxt_compute_accel` — 使用可调参数

函数签名新增 `params=None` 参数，从 `fxt_params` 字典读取增益和幂次，
`alpha_v` 和 `beta_v` 由 `alpha_p`、`beta_p` 派生计算。

### 3.4 新增槽位分配函数 `_fxt_local_assignment(dist_mat)`

对应 MATLAB `formation_controller.m:132-156` 的 `local_assignment`：
- 输入：距离矩阵 [跟随者数 × 槽位数]
- 算法：全排列遍历找最小总距离（小规模最优分配，避免依赖匈牙利算法工具箱）
- 输出：[(follower_idx, slot_idx), ...]

### 3.5 扩展 `_fxt_state_loop` — 跟随者 FXT 控制律

在领航者控制之后，新增跟随者控制循环（对应 MATLAB `formation_controller.m:84-119`）：

```
对每个跟随者 i:
  slot = FXT_FORMATION_SLOTS[formation_type][assigned_slots[i]]
  des_pos = leader_pos + slot          # 期望位置 = 领航者 + 槽位偏移
  des_vel = leader_vel                 # 期望速度 = 领航者速度
  des_acc = leader_accel               # 期望加速度 = 领航者控制输出

  e_pos = follower_pos - des_pos
  e_vel = follower_vel - des_vel
  u_f = _fxt_compute_accel(e_pos, e_vel, des_acc)  # FXT 律

  follower_cmd_vel[i] += u_f * dt      # 积分成速度
  _send_rectangle_command(i, vx, vy, target_yaw)   # 下发
```

关键设计：跟随者的期望加速度 = 领航者的实际控制输出 `leader_accel`，
这是 FXT 编队控制律的核心（对应 MATLAB `a_p0 = u_k(1:2,1)`）。

### 3.6 修改 `_start_fxt_experiment` — 新增槽位分配

启动时自动完成槽位分配：
1. 收集所有已连接的跟随者
2. 计算距离矩阵 `dist_mat[i][j]` = 跟随者i 到 槽位j 的距离
3. 调用 `_fxt_local_assignment` 得到最优分配
4. 初始化各跟随者的速度积分状态 `follower_cmd_vel`

### 3.7 修改 `_stop_fxt_experiment` — 停止所有车

停止时向领航者和所有跟随者下发零速度指令。

### 3.8 新增参数路由 `/api/formation/fxt/params`

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/formation/fxt/params` | 获取当前 FXT 参数 |
| POST | `/api/formation/fxt/params` | 设置 FXT 参数（alpha_p, beta_p, k_p1, k_p2, k_v1, k_v2, formation_type） |

### 3.9 fxt_state 新增字段

| 字段 | 说明 |
|------|------|
| `assigned_slots` | 槽位分配结果 `{car_id: slot_idx}` |
| `follower_cmd_vel` | 各跟随者速度积分 `{car_id: [vx, vy]}` |
| `follower_prev_heading` | 各跟随者上一帧航向 |
| `formation_type` | 队形类型 |

---

## 4. index.html 改动详解

### 4.1 FXT 参数面板（矩形轨迹面板下方新增）

| 控件 | ID | 说明 |
|------|----|------|
| 速度输入 | `fxtSpeed` | 0.05~0.7 m/s |
| 队形选择 | `fxtFormationType` | 菱形/三角形/直线 |
| α_p 输入 | `fxtAlphaP` | 0.1~0.9 |
| β_p 输入 | `fxtBetaP` | 1.1~3.0 |
| k_p1/k_p2/k_v1/k_v2 | `fxtKP1` 等 | 增益参数 |
| 启动按钮 | — | `startFxtExperiment()` |
| 停止按钮 | — | `stopFxtExperiment()` |
| 状态显示 | `fxtExperimentStatus` | 运行状态 |

### 4.2 新增 JS 函数

| 函数 | 说明 |
|------|------|
| `startFxtExperiment()` | 先 POST 参数，再 POST 启动 |
| `stopFxtExperiment()` | POST 停止，自动下载 CSV |
| `getFxtExperimentStatus()` | 1秒轮询状态，自动下载新文件 |
| `updateFxtExperimentStatus()` | 更新状态显示 |
| `triggerFxtDownload()` | 触发文件下载 |

### 4.3 状态轮询

在页面初始化中新增：
```javascript
getFxtExperimentStatus();
setInterval(getFxtExperimentStatus, 1000);
```

---

## 5. control.py 改动详解

### 5.1 新增 import

```python
from formation_controller import get_formation_info, get_rectangle_experiment_info, get_fxt_experiment_info
```

### 5.2 互锁逻辑

在 `control_car_position` 和 `control_car_velocity` 中新增 FXT 实验检查：

```python
fxt_info = get_fxt_experiment_info()
if fxt_info.get('active'):
    return jsonify({'success': False, 'error': 'FXT编队实验进行中，已锁定单车位置控制！'})
```

效果：FXT 实验期间，单车位置控制和遥控均被拒绝，防止指令冲突。

---

## 6. 完整 API 接口列表

| 方法 | 路径 | 说明 | 阶段 |
|------|------|------|------|
| POST | `/api/formation/fxt/start` | 启动 FXT 编队实验 | 1 |
| POST | `/api/formation/fxt/stop` | 停止并保存 CSV | 1 |
| GET | `/api/formation/fxt/status` | 查询状态 | 1 |
| GET | `/api/formation/fxt/download/<filename>` | 下载 CSV | 1 |
| GET/POST | `/api/formation/fxt/params` | 获取/设置 FXT 参数 | **2** |

---

## 7. 与第一阶段的差异

| 维度 | 第一阶段 | 第二阶段 |
|------|---------|---------|
| 控制对象 | 仅领航者 | 领航者 + 所有跟随者 |
| 跟随者控制 | 无 | FXT 编队律（跟踪领航者+槽位） |
| 槽位分配 | 无 | 启动时局部最优分配 |
| 参数配置 | 硬编码 | 前端可调 + API 设置 |
| 前端 UI | 无 | 参数面板 + 队形选择 + 启停按钮 |
| 互锁 | 无 | FXT 期间锁定单车控制/遥控 |

---

## 8. 已知限制（待第三阶段解决）

1. **ω 数值微分**：角速度仍通过 `Δheading/Δt` 估算，精度有限
2. **航向非 FXT**：航向仍由下位机 yaw PID 控制
3. **增益未整定**：参数可在前端调节，但 20Hz 下的最优增益需实验确定
4. **槽位全局坐标**：槽位偏移在全局坐标系下定义（与 MATLAB 一致），未做车体坐标系转换

---

## 9. 验证方法

1. 打开 Web 界面，在"FXT固定时间编队实验"面板设置参数
2. 先通过编队控制界面指定领航者
3. 点击"启动FXT编队"，观察所有车是否形成指定队形并沿矩形运动
4. 点击"停止并下载数据"，分析各车 CSV 轨迹
5. 验证互锁：FXT 实验期间尝试单车控制应被拒绝

---

## 10. 后续阶段预告

| 阶段 | 内容 | 涉及文件 |
|------|------|---------|
| 第三阶段 | 下位机上报 ω + 可选加速度指令模式 | `uart.c`, `app_main.c`, `net.py`, `models.py`, `vel_solver.h/c`, `proto_parser.c` |
