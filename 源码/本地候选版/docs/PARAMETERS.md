# S 弯参数手册（2026-11-06 23:30，速度治理版）

车：`/root/racecar/tools/s_bend/`；改参数：`./set_param.py <key> <value>`；看当前：`./set_param.py --show`；每次改完要重跑 `./run_dense.sh`。

本版结构与 21:04 的 APPROVED 版不同，改动集中在三处：**安全门不再含速度**、**新增连续速度治理器**、**观测事件**。下面的参数说明以车上的代码为准。

## 一、当前生效参数（Stage 1）

| 参数 | 当前值 | 作用 | 建议范围 |
|---|---:|---|---|
| `target_speed_mps` | 1.0 | 最终目标速度 | 0.8–1.5（上限 2.0）|
| `launch_speed_mps` | 0.6 | 起步目标速度 | 0.4–0.8，必须 ≤ target |
| `speed_ramp_seconds` | 1.0 | 起步到 target 的时间 | 0.8–2.0 |
| `speed_feedforward_pwm` | 1530 | 速度环前馈 | 1520–1560 |
| `speed_kp` | 30 | 速度环 P | 15–40 |
| `speed_ki` | 8 | 速度环 I | 3–12 |
| `launch_pwm` | 1556 | 静止起步 PWM（越过电机死区）| 1552–1560 |
| `minimum_pwm` | 1500 | 行驶时 PWM 下限 | 保持 1500 |
| `maximum_pwm` | 1600 | PWM 上限 | ≤1600 |
| `lookahead_base_m` | 0.7 | 基础前视距离 | 0.55–0.9 |
| `lookahead_speed_seconds` | 0.25 | 每 m/s 增加的前视 | 0.15–0.35 |
| `preview_heading_weight` | 0.35 | 航向预瞄权重 | 0.2–0.5 |
| `angle_kp` | 0.8 | 转向 P | 0.6–1.0 |
| `angle_kd` | 0.26 | 转向 D（抑制画龙）| 0.2–0.4 |
| `angle_derivative_filter_seconds` | 0.2 | D 项滤波 | 0.15–0.3 |
| `steer_mode` | `pursuit` | 转向律：`pursuit`=前瞻点几何牵引（见三点六），`pd`=旧角度 PD | `pursuit` / `pd` |
| `pursuit_gain` | 0.85 | 前瞻点曲率缩放；1.0 = 纯几何追踪，画龙就往下调 | 0.6–1.0 |
| `pp_heading_gain` | 0.5 | 路线切向额外曲率 `sin(ψ)/L` 的权重（补偿"转晚"）| 0–0.7 |
| `aim_ema_alpha` | 0.45 | 瞄准距离/航向的 EMA 系数，1.0 = 关 | 0.3–1.0 |
| `kf_enable` | `true` | 瞄准点横向偏移/航向的卡尔曼滤波 | `true` / `false` |
| `kf_process_noise` | 0.5 | 卡尔曼加速度噪声（越大跟随越快、越抖）| 0.2–3.0 |
| `kf_measure_noise` | 0.02 | 卡尔曼测量方差（越大越不信单帧）| 0.005–0.1 |
| `kf_max_gap_s` | 0.6 | 超过这么久没有规划帧就重新起算（不跨间隙外推）| 0.3–1.0 |
| `max_steer_deg` | 35 | **舵角上限**（相对中位 ±deg），不是单帧变化量 | 20–45 |
| `full_steer_deg` | 65 | 舵角→曲率模型的满舵角 | 45–90 |
| `transition_extension_m` | 0.3 | 路线末端允许外推长度 | 0.2–0.5 |
| `corner_offset_m` | 0.40 | 出弯内贴量（见二）| 0.30–0.55 |
| `follow_offset_m` | 0.35 | **已废弃，不参与控制** | — |
| `distance_mode` | `half_width` | 必须保持这个值 | 不要改 |
| `planner_mode` | `dense` | `dense`=现有版本，`corridor`=两边缘走廊实验版 | `dense` / `corridor` |
| `curv_prior` | `true` | **稀疏边界曲率先验**（见四点二）：2 点行用对面行的曲率，3–5 点行把自身曲率按 `curv_prior_beta` 拉向对面行 | `true` / `false`（关掉做 A/B）|
| `curv_prior_beta` | 0.4 | 3–5 点行借对面曲率的比例；1.0=完全忽略本行 | 0.3–0.5（0.6 实测更差）|
| `row_memory` | `false` | 旧机制：用上一帧中心线补点。**实测退化，默认关**（见四点二）| `true` / `false` |
| `cross_row` | `false` | 旧机制：用对面行平移补点。`curv_prior` 开后基本不再触发，默认关 | `true` / `false` |
| `debug_viz` | `false` | 向 `/s_bend/debug` 发 RViz2 MarkerArray（见三点五）| 只在调试时开 |

### 速度治理器 / 安全门（新增，别和旧参数混用）

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `a_lat_mps2` | 1.5 | 侧向加速度上限，`v_curv=sqrt(a_lat/κ)` |
| `a_accel_mps2` | 0.8 | 速度参考上升斜率 |
| `a_decel_mps2` | 3.0 | 速度参考下降斜率（只限参考值，不靠它刹车）|
| `coast_decel_mps2` | 1.5 | 反解净空用的物理滑行减速度 |
| `reaction_time_s` | 0.20 | 反解净空用的反应时间 |
| `gate_floor_m` | 0.25 | 门的下限：**唯一**的候选通过条件（避障距离，见三）|
| `clearance_probe_m` | 2.4 | 净空探针长度（开阔时返回这个值）|
| `clearance_step_m` | 0.05 | 足迹沿弧线的采样间距 |
| `clearance_min_points` | 2 | 同一足迹矩形内至少几个回波才算挡住 |
| `heading_error_enter_deg` | 28 | 航向误差 latch 进入（转向跟不上）|
| `heading_error_exit_deg` | 18 | latch 退出 |
| `v_lat_cap_mps` | 0.65 | latch 期间的速度上限 |
| `v_min_cruise_mps` | 0.30 | 巡航下限，永不超过 `v_clear` |
| `blocked_stop_s` | 1.0 | 连续阻塞且已停稳 → `stop('blocked_front')` |
| `blocked_timeout_s` | 2.0 | 连续阻塞总时长 → 停车（即使还没停稳）|
| `blocked_speed_mps` | 0.05 | 判定“已停稳”的速度 |
| `curvature_percentile` | 90 | 制动视窗内曲率取 p90 |
| `pwm_slew_up_us` | 6 | PWM 每个控制周期的上升上限 |
| `pwm_slew_down_us` | 12 | PWM 下降上限（比上升快，先收油）|
| `curvature_ff_gain` | 0.0 | 曲率前馈增益（0=关；只在记录证明 PD 滞后时开 0.4–0.7）|

## 二、速度治理器怎么工作

每拍（20 Hz）在 `choose_steering` 之后、任何 `neutral/publish` 之前：

```
v_ramp  = launch + (target-launch) * min(1, t/ramp_s)
v_curv  = sqrt(a_lat / max(kappa_look, 1e-3))
v_clear = v_allow(C)                       # C = 本拍选中舵角的实测净空
v_lat   = v_lat_cap  若航向误差 latch 中，否则 target
v_target = clamp(min(target, v_ramp, v_curv, v_clear, v_lat), 上一拍±a·dt)
v_target = max(v_target, min(v_min_cruise, v_clear))   # 只在门通过时
```

`v_allow` 由“刹车距离 + 余量 ≤ C”反解：

| C (m) | 1.35 | 1.15 | 0.90 | 0.60 | 0.50 | 0.40 | 0.30 | 0.25 |
|---|---|---|---|---|---|---|---|---|
| v_allow，floor 0.30 | 1.50 | 1.32 | 1.07 | 0.69 | 0.53 | 0.32 | 0 | 0 |
| v_allow，floor 0.25（当前）| 1.54 | 1.37 | 1.13 | 0.77 | 0.62 | 0.43 | 0.19 | 0 |

**门只回答“还有多远”，速度由治理器给。** 门不含速度，所以不会在高速时二值化（旧式 `required = max(.28,.12+.25v+.25v²)` 在 1.5 m/s 时要求 1.06 m，比走廊剩下的还宽，必然全拒）。

- `kappa_look`：沿 `route.centers` 累积弧长，取制动视窗 `v_ref*T + v_ref²/(2a_coast)`（`v_ref=max(|v|,target)`）内的 `|route.curvature|`，3 点平滑后取 p90。本赛道 κ≈0.5 1/m → `v_curv≈1.73`，**这条 S 弯不会触发曲率项**；它是紧弯保险。
- 航向误差 latch：`|err|>28°` 进入、`<18°` 退出，期间速度 ≤0.65。`err` 是 PD 的输入角（`atan2(target)+preview_weight*heading`），不是横向距离——中心线起点在车前 0.3–1.1 m，横向误差量不准。

## 三、安全门（`choose_steering`）

1. 候选通过条件只有一条：选中舵角沿弧线扫过的**足迹净空 C ≥ `gate_floor_m`（当前 0.25 m）**。
2. 净空探针 2.4 m，5 cm 一格采样；同一足迹矩形内 ≥2 个回波才算挡住（单束噪点不算）。
3. 相对候选 `[0,±4,±8,±12,±16,±20]` 全部不过时，扫绝对角（±`max_steer`，步长 6°）取 `argmax C`，**永不返回 None**。
4. 净空路径的足迹用激光系：`front'=front-laser_x=0.17`、`rear'=rear+laser_x=0.39`。`forward_path_points` 已经把点换到激光系，旧代码用车身系常数等于白送 0.23 m 前向余量。
5. 被拒的那一拍**不切中立**：保持 `argmax C` 的舵角，滑行（`coast`），速度参考被压到 0。
6. 连续阻塞：停稳满 `blocked_stop_s` 或总时长满 `blocked_timeout_s` → `stop('blocked_front …')`，发中性并退出落盘。**不倒车**（用户决定：卡住就停，人工处理）。
7. **`gate_floor_m` 就是“避障距离”**：车在扫过的弧线上必须留出这么多空。它同时是 `v_allow` 的零点，所以调小它有两个效果——门更宽容（少拒/少停）+ 贴桶时允许的速度更高。

   调法（一次 0.05 m，`./set_param.py gate_floor_m 0.25`，改完必须重跑 `./run_dense.sh`）：

   | 值 | 效果 | 什么时候用 |
   |---:|---|---|
   | 0.35 | 最保守，离桶远就减速/停 | 新赛道、锥桶位置不确定 |
   | **0.25** | 当前值，比原 0.30 略松 | 默认 |
   | 0.20 | 只在前方真的堵死时才拒 | 频繁 ALIGN_ROUTE、但没撞到东西 |
   | 0.15 | 贴着锥桶走 | **下限**，再小就退化成没有门 |

   - 调小之后必须看 `min C` 和 `C - H(v_target)`：如果 `min C` 长期贴着新 floor、或者 `C-H` 经常为负，说明这是在拿余量换速度，要调回去。
   - **不要**通过调小 `gate_floor_m` 去掩盖别的问题（自回波、idle 抖动）：先把 `C` 的绝对值看清楚，19:38 起步时 `C` 应该是 2.4，不是 0.4。
   - `clearance_probe_m`（2.4）不是避障距离，它是“视野有多远”，不要为了“调小避障”去动它。

## 三点五、RViz2 调试可视化（`debug_viz`）

```bash
cd /root/racecar/tools/s_bend
./set_param.py debug_viz true      # 只在调试时开
./run_dense.sh --observe           # 或正常跑；改完必须重跑
```

- 话题 `/s_bend/debug`（`visualization_msgs/MarkerArray`，10 Hz，marker lifetime 0.3 s，节点停了自己消失）。
- RViz2：**Fixed Frame 设成 `/scan` 的 `frame_id`**（本车是 `laser_link`），Add → MarkerArray → `/s_bend/debug`。
  不用配 TF。
- 颜色/编号：

| id | 内容 | 外观 |
|---|---|---|
| 0 | 左边界拟合曲线 | 蓝线 |
| 1 | 右边界拟合曲线 | 橙线 |
| 2 | 实际跟随的中心线 `route.centers` | 绿色粗线 |
| 3 / 4 | 参与拟合的左 / 右边界点（**含 `row_memory` 补出的点**）| 蓝 / 橙点 |
| 5 | 瞄准点 `route.target` | 黄色球 |
| 6 | 当前舵角对应的扫掠弧线（白线，2.4 m）| 白线 |
| 7 | 全部聚类锥桶 | 灰点 |

- 关掉：`./set_param.py debug_viz false`。开着会让 `plan_ms` 略微上升（纯 Python 拼 marker），
  看 `plan_ms` p95 是否还 <25 ms。

## 三点六、前瞻点牵引转向（`steer_mode` / EMA / 卡尔曼）

```bash
./set_param.py steer_mode pursuit     # 默认；pd = 旧的角度误差 PD
./set_param.py kf_enable true
./run_dense.sh                        # 改完必须重跑
```

**几何**：车体系下瞄准点 `(x, y)`，过车原点和该点的圆曲率
`κ = 2y/(x²+y²)`；舵角用与安全门**同一个**线性模型反算：
`δ[deg] = κ · car.radius · full_steer`。所以 `pursuit_gain=1.0` 就是纯几何追踪，
门、治理器都不用改。弯里光靠瞄准点会"转晚了"，再叠加路线切向的曲率
`κ += pp_heading_gain · sin(ψ)/L`。

**两级滤波，各管一件事**：

| 滤波器 | 作用对象 | 为什么 |
|---|---|---|
| EMA（`aim_ema_alpha`）| 瞄准距离 `x`、路线航向 `ψ` | 两者都慢变；单帧 EMA 就能压掉"行点数变化导致瞄准点横摆"的闪烁 |
| 卡尔曼（`kf_*`）| 瞄准点横向偏移 `y`、航向 `ψ` | 状态 `[值, 变化率]` 的常速模型：一帧噪声只被部分采信，成趋势的变化快速跟上；规划帧掉一拍时用预测值顶住，而不是重新起算 |

回到 `route=None` 或直线保持帧时两个滤波器都 `reset()`——否则上一个弯的横向偏移会被
带进下一个弯。日志 `control_decision` 事件新增 `steer / aim_lat / aim_dist / aim_rate`。

**离线对比**（`replay_steer.py`，同一批瞄准点喂给两套律）：

| 记录 | 指标 | PD | 前瞻点（gain .85） |
|---|---|---|---|
| 19:38 | 舵量 mean / p90 / max | 12.0 / 26.8 / 42.0 | 14.3 / 30.3 / 34.5 |
| | 帧间 \|Δ舵\| mean / p90 / max | 4.19 / 12.28 / 23.22 | **3.05 / 7.31 / 22.21** |
| | 贴限幅比例 | 4% | 2% |
| 22:41 | 舵量 mean / p90 | 6.8 / 15.6 | **11.0 / 18.3**（补上转向不足）|
| | 帧间 \|Δ舵\| mean / max | 3.12 / 19.34 | **2.62 / 9.04** |

调参方向：车画龙 → 降 `pursuit_gain`（0.6–0.8）；弯里还是转不过来 → 先加
`pp_heading_gain`（0.5→0.7），再加 `curvature_ff_gain`。`kf_process_noise` 越大跟随越快、
越小越平滑；`kf_measure_noise` 越大越不信单帧测量。

> **顺带修了一个单位 bug**：`curvature_ff_gain` 原来写成
> `math.degrees(κ·radius·full_steer)`，而 `κ·radius·full_steer` 本来就是**度**，
> 等于乘了 57.3——只要 `curvature_ff_gain > 0.02`，舵机就会被直接顶到限幅。
> 以前默认是 0，所以没人发现；现在它才是文档里说的那个增益。

## 四点二、稀疏边界拟合：曲率先验（`curv_prior` / `curv_prior_beta`）

- **问题**：某一边只剩 2–3 个锥桶时，拟合对点的增删极敏感。2 点行只能拟合**直线**——曲率恒为 0，
  而 0 曲率正好出现在最需要转弯的地方；3–5 点行的二次曲率也会被单个锥桶的进出带得乱跳。
  22:41 的记录里 49 帧有 23 帧存在 <3 点的行，中心线在 x=1 m 处的 y 单帧跳变 p90 8.7 cm、max 20.3 cm；
  t=6.24 s 左行掉到 2 点后，规划器退回 `_hold_route` 直线（curv=0，航向冻结在 +3°），车就“转不过来”。
- **做法**（不给拟合增加任何新点，只借一个数）：
  - 对面那一行是**同一条车道从另一侧看**，同一位置曲率相同 → 当作先验。取先验时按本行自己的 x 跨度
    在对面曲线上取平均（不是取全曲线中位数，S 过渡处对面行可能已经在下一个弯里）。
  - 本行 **2 点**：过这两个点、二阶导固定为对面曲率的抛物线（`fit_quadratic_anchor`）。
    两点仍精确通过，形状来自先验。
  - 本行 **3–5 点**：先自己拟合一条二次曲线，把二次项按 `beta` 拉向对面曲率，
    再对剩下的 b、c 做最小二乘（`fit_quadratic_blend`）。`beta=0.4` 表示 60% 听自己的。
  - 对面行自己不足 3 点时没有先验，退回原逻辑。
- **怎么调**：`./set_param.py curv_prior false` 关掉做 A/B；`./set_param.py curv_prior_beta 0.5`
  更信对面。实测 β=0.6/0.8 明显更差（中心线 y 跳变 p90 反而上升），别往上加。
- **旧机制已关**：`row_memory` / `cross_row` 是"用上一帧中心线 / 对面行平移**补点**"的方案。
  在三条记录（19:38 / 21:25 / 22:41）上 A/B：补点版 `route=None` 帧 9→18（19:38）、
  straight-hold 11→12（22:41），比不开还差；而曲率先验版在同一组记录上是 `route=None` 18→4 / 4→0、
  straight-hold 12→7 / 5→3。补点是从"上一帧的路线"反推边界，本身就是一个正反馈环
  （路线→补点→路线），所以默认关闭。日志里 `row_stats[s]` 的 `points` 是实测点数、
  `fit_points` 是参与拟合的点数，两者相等就说明没在补点。

## 四、代码里固定的常数（不通过 json 调）

- 圆弧拟合只对**紧弯**生效：`R ≤ 3.5 m`、转角 `≥ 0.8 rad（46°）`、`rms ≤ 0.18 m`；缓弯仍用二次/样条。
- 出弯内贴条件：`side=+1`（在左行）且 `heading < -0.1 rad`；目标 y 回移 `半宽 - corner_offset_m`。
  - 0.40 → 回移约 0.25 m；0.50 → 回移约 0.15 m。
- 电机死区实测：1548 几乎不动，1552 起跳；`launch_pwm` 必须 ≥1552。
- 编码器有效上限 3.0 m/s（超过判无效）。
- 舵机速率限制 85°/s。
- 前视点下限 0.55 m。
- `ObstacleHold`：单帧被拒=hold；连续 2 帧或 0.9 s=fault。fault 时中立并等路线连续 3 帧干净、且车已停稳才重新驱动；单帧被拒直接按 `a_accel` 加回，不停车。
- 航向跳变保护已删除（会冻住真弯）。

## 五、按症状调

| 症状 | 先调 | 方向 |
|---|---|---|
| 直道太慢 | `target_speed_mps` | +0.1/次；先过验收线再加 |
| 起步太冲 | `launch_speed_mps`↓、`speed_ramp_seconds`↑ | 0.5 / 1.3 |
| 起步不走 | `launch_pwm`↑（1558–1560）| 先确认 `/encoder` 有 50 Hz |
| 速度一卡一卡 | `speed_kp`↓、`speed_ki`↓ | 30→20，8→5；`minimum_pwm` 保持 1500 |
| 进弯太晚/太宽 | `lookahead_base_m`↓ | 0.7→0.6 |
| 切内/擦内侧 | `lookahead_base_m`↑、`preview_heading_weight`↓ | 0.75 / 0.30 |
| 出弯往右前方甩 | `corner_offset_m`↓ | 0.40→0.35 |
| 出弯太贴左、擦红锥 | `corner_offset_m`↑ | 0.40→0.50 |
| 舵机画龙/抖 | `angle_kd`↑、`angle_derivative_filter_seconds`↑、`angle_kp`↓ | 0.32 / 0.25 / 0.7 |
| 转向太肉 | `angle_kp`↑、`preview_heading_weight`↑ | 0.9 / 0.4 |
| 弯里掉速太多 | 先看 `control_decision.reason`；`lateral` 多=latchet 太紧，`clearance` 多=门真的近 | 提 `heading_error_enter_deg`，**别**降 `gate_floor_m` |
| 直道也上不去速度 | 看 `reason`：`curv`→`a_lat` 太高或曲率估高了；`clearance`→视野里有锥桶 | 逐个看 |
| 中途丢线停 | 先看日志：`encoder null`=线松；`target=None`=锥桶丢；`target.x<=0.12`=到末端 | 代码问题不要用参数硬救 |
| 末端提前停 | `transition_extension_m`↑ | 0.3→0.4 |
| 停不下来、贴桶才停 | `gate_floor_m`↑（0.35）、`coast_decel_mps2`↓ | 先查 `C-H(vt)` 是否长期为负 |

## 六、推荐调试顺序（一次只改一项）

1. **先保线路**：`lookahead_base_m`、`corner_offset_m`；
2. **再修平滑**：`angle_kd`、`angle_derivative_filter_seconds`；
3. **最后加速度**：只用 `target_speed_mps`，每次 +0.1–0.2，跑 2–3 次；
4. 觉得某一版好，立刻备份：

```bash
cd /root/racecar/tools
cp -a s_bend "s_bend_GOOD_$(date +%Y%m%d_%H%M%S)"
```

## 七、每次跑完看什么

```bash
cd /root/racecar/tools/s_bend
./analyze_dense_run.py dense_last_run.json --criteria 1     # 1/2/3 = 阶段号
```

新事件：`command` 事件带 `v`/`vt`；`control_decision` 每 0.1 s 一条，字段
`reason`（target/ramp/curv/clearance/lateral/gate）、`clearance`、`floor`、`probe`、
`passing`、`widen`、`v`、`vt`、`v_ramp/v_curv/v_clear/v_lat`、`err_deg`、`latched`、`kappa`、`hold`。

重点：

- `stop`：不得出现 `blocked_front / cone_corridor_lost / sensor_or_driver_unavailable`；
  失败跑会写成 `manual`，所以要同时看 `WAIT_ROUTE/ALIGN_ROUTE` 最长连续时长 >1.0 s 即失败；
- `gate rejections`：移动中 ≤2 次、最长 ≤0.2 s；
- `clearance C`：`min ≥ gate_floor_m`（当前 0.25），允许单帧、不允许连续两帧；
- `C - H(v_target)`：`min ≥ -0.05 m`；
- 驱动窗口内中立占比：Stage1 ≤15%、Stage2 ≤12%、Stage3 ≤10%；
- `|error| deg`：Stage1/2 中位 ≤15°、p90 ≤32°；Stage3 中位 ≤20°、p90 ≤38°；
- 直线段 `command.v` 中位 ≥0.85×target，全程 p10 ≥0.60×target；
- `plan ms`：p95 <25 ms、max <50 ms（19:38 是 med 9.1/max 24）。
- 每次跑都有 `records/logs/run_<stamp>.log`（含文件 sha256），别只依赖 JSON。

## 八、分阶段

| 阶段 | target | lookahead | max_steer | corner | curvature_ff |
|---|---:|---:|---:|---:|---:|
| 1 | 1.0 | 0.70 | 35 | 0.40 | 0.0 |
| 2 | 1.2 | 0.70 | 38 | 0.40 | 0.0（滞后才开 0.4）|
| 3 | 1.5 | 0.75 | 42 | 0.40 | 0.4–0.7 |

切换：

```bash
cd /root/racecar/tools/s_bend
./set_param.py target_speed_mps 1.2
./set_param.py max_steer_deg 38
./run_dense.sh
```

任一阶段 3 跑中 2 跑通过才进下一级；出现停车/撞桶/横向比上一级更差 → 回退上一级，先诊断。

## 九、备份与回退

- 本版上车前的完整备份：`/root/racecar/tools/s_bend_pre_governor_20261106_231225`
- 21:04 的 APPROVED（旧安全门、无治理器）：`/root/racecar/tools/s_bend_APPROVED_20261106_210450`
- 21:49 的 APPROVED2（失败配置：target 1.5 / corner 0.28）：**不要用**，仅作对照
- 回退参数：`dense_settings.json.bak` 是上一次 `set_param.py` 的备份。
- 目录间恢复（保留当前参数时只拷代码）：

```bash
cp /root/racecar/tools/s_bend_pre_governor_20261106_231225/dense_s_bend.py .
cp /root/racecar/tools/s_bend_pre_governor_20261106_231225/core/vehicle_io.py core/
cp /root/racecar/tools/s_bend_pre_governor_20261106_231225/core/geometry.py core/
```

## 六点五、两边缘走廊实验版（corridor）

- `planner_mode=corridor` 时用 `corridor/` 下的 `CorridorPlanner`；
- 现状：模块已接入、observe 能跑；配对成功率不够（经常 `route=none`），**暂不上车实跑**。

## 十、不要碰
- `racecar/` 基础包、`hairpin.py`、`run_hairpin.sh`、`start_sensors.sh`、`stop_car.sh`；
- `distance_mode` 必须保持 `half_width`；
- `max_steer_deg ≤ full_steer_deg ≤ 90`；
- `launch_pwm ≤ maximum_pwm ≤ 1600`；
- `/dev/ttyACM0` 永远不要直接写；诊断文件不要写到车上除 `s_bend/records/` 以外的地方。
