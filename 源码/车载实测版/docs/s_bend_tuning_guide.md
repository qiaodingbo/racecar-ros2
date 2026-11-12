# 小车 S 弯（dense S-bend）参数设置指南

代码位置：车上 `davinci-mini` 的 `/root/racecar/tools/s_bend/`
（`dense_s_bend.py`、`dense_reference.py`、`vehicle_io.py`、`geometry.py`、`dense_settings.json`、`run_dense.sh`）
本文对应 2026-11-05 21:31 那一次 `dense_last_run.json`。

> **2026-11-05 22:11 已应用**：改动前的整个目录备份在车上
> `/root/racecar/tools/s_bend_backup_20261105_221117/`。当前生效值就是第 5 节的 JSON；
> 舵量上限已参数化为 `max_steer_deg`，宽度超时 1.0 s，共享锥桶 0.12 m，宽度跳变限幅。
> **22:37 追加**：第一次实跑在 S 弯拐点停车的根因是"中心线 x 回折"被整段拒绝，已改成裁掉远端回折。
> **22:42 追加**：第二次实跑在同一位置停车的根因是"换基准行时候选行中心线偏 1.1 m，旧代码先切后拒"，
> 已改成换行前先比对、不一致就保持旧行；同时把 `maximum_pwm` 上限从 1558 放到 **1600**，
> `target_speed_mps` 从 0.35 提到 **0.60**。改动前的目录另存为
> `/root/racecar/tools/s_bend_backup_20261105_224655/`。
> **22:52 追加**：第三次实跑在第二弯入口停车，左行 4 点拟合曲率到 ±1.4 1/m，偏移中心线半径小于车的最小半径，
> 曲率 guard 把整条路判死；已在密度投票前先剔除几何上不可行的行、改用另一侧。速度提到 **1.0 m/s**；
> 备份 `/root/racecar/tools/s_bend_backup_20261105_225246/`。

---

## 0. 一句话结论

这套 S 弯不是"固定打角 + 定时"，而是**每帧重新拟合锥桶边界、算中心线、追前视点**的闭环。
所以参数分三层，调参顺序不能反：

1. **执行器层**：舵量上限、中位、PWM–速度关系（不改这层，后面调增益都是白调）；
2. **跟踪层**：前视距离、`angle_kp/kd`、`preview_heading_weight`；
3. **速度层**：`target_speed_mps`、PWM 前馈、PI。

你这次记录的症结在第 1 层：**舵量 4.2 秒顶死在 +16° 上限**，横向偏差一直没收敛。
先改舵量，再谈 kp/kd。

另外，这次最后停车不是撞锥桶，是拐点处**宽度测量连续 5 帧失效、超过 0.55 s 过期**导致 planner 主动丢路线（见第 4、7.2 节）；这个要单独修。

---

## 1. 程序在做什么

```
/scan ──聚类──> 锥桶点
      ──按左右分行──> 从最近锥桶链式连接（最多 8 跳、相邻 < 2×车宽、转角 < 47°）
      ──选"更密"的一行当基准──> 三次样条拟合
      ──基准行向车道内平移 实测宽度/2──> 中心线
      ──中心线上前视 L 米处──> target
      ──PD(目标方位 + 0.5×路径切向)──> 舵机角
      ──安全窗检查/限速/限幅──> /car_cmd_vel
```

速度环：`/encoder` 速度 → PI → PWM。
坐标系约定（很重要，别搞反）：

- `linear.x` = **电机 PWM（us）**，1500 中立；
- `angular.z` = **舵机角（度）**，`servo_center = 84.5`，**大于 84.5 是左转**；
- 激光在车头前 `laser_x = 0.23 m`；
- 车道宽 `width` 是两排锥桶的**实测横向距离**，中心线偏移量 = `width/2`（`distance_mode` 只能设 `half_width`）。

---

## 2. `dense_settings.json` 逐项

| 参数 | 现值 | 作用 | 建议 / 调整方向 |
|---|---:|---|---|
| `target_speed_mps` | 1.20 | 速度环目标（m/s） | 1.0/1.2 能跑但会因安全窗走走停停；现在 1.2，配合放宽后的安全窗 |
| `distance_mode` | `half_width` | 中心线 = 实测宽度的一半 | **不要动**，代码只认这个值 |
| `lookahead_base_m` | 0.65 | 前视距离基数（m） | 0.65–0.80。同样的横向偏差 e，L 越大方位角 `atan(e/L)` 越小、越"懒"；抖就加，切内太多就减 |
| `lookahead_speed_seconds` | 0.8 | 前视随速度增加（秒） | 0.5–0.8。低速 0.3 m/s 时它只贡献 0.24 m |
| `transition_extension_m` | 0.25 | 基准行看不到时，允许沿末端切线外推的最大距离 | 0.25–0.35。出弯/短行时频繁 `cone_corridor_lost` 就加到 0.35 |
| `preview_heading_weight` | 0.5 | 把路径切向角按比例加进航向误差（弯道前馈） | 0.35–0.5。它负责"提前进弯"，太大时直道也会偏 |
| `speed_kp` | 60.0 | 速度 PI 比例（us per m/s） | 40–60；0.1 m/s 误差只给 4–6 us，别指望它干重活 |
| `speed_ki` | 20.0 | 速度 PI 积分 | 8–15。前馈准了就把 ki 降下来，减少"油门拖尾" |
| `speed_feedforward_pwm` | 1535.0 | 目标速度的基础 PWM | 最终按实测改；**这次先别动**（见 Step 4，贸然降 ff 可能起步不动） |
| `launch_pwm` | 1548.0 | 静止起步时的 PWM 上限（破静摩擦） | `ff + 12~25`；车不动就加，起步窜就减 |
| `maximum_pwm` | 1600.0 | PWM 输出上限 | 代码上限现在是 **1600**，设再大启动会直接报配置错误；ESC 本身支持 500–2500 |
| `angle_kp` | 1.2 | 每 1° 航向误差给多少舵机度 | 0.8–0.9（配合把 ±16° 放宽）。1.2 时误差 13.3° 就打满 |
| `angle_kd` | 0.15 | 阻尼，抑制摆头 | 0.20–0.30；进弯甩头就加 |
| `angle_derivative_filter_seconds` | 0.15 | 微分低通时间常数 | 0.15–0.20；激光噪声大就加大 |
| `max_steer_deg` | 35.0 | 舵机最大输出偏移（相对 84.5°） | 30–40；先实测舵量–半径再定 |
| `full_steer_deg` | 65.0 | "满舵"命令，用于安全窗曲率换算 | 按实车满舵填；默认 65 |

单位换算（免得调错）：

- `angle_kp = 1.2` 就是"航向误差 1° → 舵机 1.2°"（`error` 是弧度，`math.degrees()` 又乘回来了）；
- `angle_kd = 0.15` 约等于"误差变化 1 rad/s → 舵机 +8.6°"；
- 速度环 `pwm = ff + kp·Δv + ki·∫Δv`，Δv 单位 m/s。

### 2.1 怎么改这些参数

推荐用同目录的 `set_param.py`（只改一个值、自动备份、带校验）：

```bash
cd /root/racecar/tools/s_bend
./set_param.py --show                  # 看当前全部参数
./set_param.py target_speed_mps 0.30   # 改一个
./set_param.py max_steer_deg 30
```

也可以直接 `vim dense_settings.json`（`i` 插入，`Esc`，`:wq` 保存；`:q!` 放弃），改完用
`/usr/bin/python3 -m json.tool dense_settings.json >/dev/null` 校验。
参数只在 `run_dense.sh` 启动时读一次，**改完必须重启节点**。会被拒绝的配置：
`1500 <= ff <= maximum_pwm <= 1600`、`1500 < launch <= maximum_pwm`、
`0 < target_speed_mps <= 2.0`、`10 <= max_steer_deg <= full_steer_deg <= 90`，
`distance_mode` 只能是 `half_width`。

---

## 3. 只能改代码的"隐藏参数"

这些不在 JSON 里，但**比你 JSON 里的任何一项都关键**：

| 位置 | 值 | 含义 | 现状 / 怎么改 |
|---|---:|---|---|
| `dense_settings.json` `max_steer_deg` | 35.0 | 舵机输出硬限幅（相对 84.5°） | **已参数化**；实测舵量–半径后改这里，10–45 合理 |
| `dense_settings.json` `full_steer_deg` | 65.0 | "满舵"对应的命令，用于曲率换算 | 只在安全窗里用；按实车满舵填，默认 65 |
| `dense_reference.py:232` `AnglePD.update` | `self.max_steer` | 舵机输出限幅 | 已改为跟随 `max_steer_deg` |
| `dense_s_bend.py:115` `choose_steering` | `self.max_steer` | 安全窗候选角的过滤上限 | 已同步 |
| `dense_s_bend.py:101-105` `clearance()` | `(offset/full_steer)/radius` | 舵量→曲率线性模型 | 已替换原来的 `tan(16°)` 模型，避免舵量放大后安全窗变"更紧" |
| `vehicle_io.py:216` | `85.0*min(dt,.06)` | 舵机速率限制 85°/s | 未改；甩头再降到 60–70 |
| `dense_s_bend.py:109` | `0,±4,±8,±12`，>20° 时加 `±16,±20` | 安全窗的修正候选 | 已按 `max_steer` 自动扩展 |
| `dense_s_bend.py:97-101` | `max(.28, .12+.25v+.25v²)` / 视界 0.55 起 | 安全距离与前瞻 | **已放宽**（23:10）：1.2 m/s 时 required≈0.78 m，旧公式是 2.14 m，S 弯里必然走走停停 |
| `dense_reference.py:154-160` | 1.25×、3 帧 | 换边判据 | 未改；拐点频繁跳边再调 |
| `dense_reference.py:165-177` | EMA 0.15、单帧跳变 >0.20 m 拒绝、过期 1.0 s | 宽度滤波与超时 | **已改**（0.55 s 太短是这次停车的直接原因） |
| `dense_reference.py:143` | 共享锥桶阈值 0.12 m | 左右行同一颗锥桶去重 | **已改**，并从两行同时删掉 |
| `dense_reference.py:96-98` | 宽度 x 范围 ±0.40 m | 对侧行参与宽度测量的范围 | **已改**（原来 ±0.15 m 太窄） |
| `dense_reference.py:183` | `denominator <= .15` | 中心线曲率上限（等价最小半径 ≈0.83 m） | **待改**：对曲率先平滑再判，见第 7.4 节 |
| `dense_reference.py:189-191` | 前视 0.45–1.2 m | 前视限幅 | 未改；下限 0.45 偏短，可到 0.55 |

---

## 4. 这次 `dense_last_run.json` 说明了什么

- 结果：`stop = cone_corridor_lost`，50 帧规划、161 条命令，约 5 秒。
- 舵机：**161 条命令里 84 条是 +16.0°（100.5），另有 7 条 +14°**；从 3.30 s 起持续到 7.50 s，**连续 4.2 s 顶死在 +16° 上限**。
- 横向：中心线在车左侧从 0.11–0.13 m 一路涨到 **0.48 m**（`analyze_dense_run.py` 在 x=1.0 m 处采样），满舵 4 秒都没收敛——这不是"增益太大在震荡"，而是**舵量不够/需求被限幅截断**的典型形状。
- PWM：中位 1505，最大 1549；起步冲到 1548 后迅速回落到 1505–1520。说明速度 PI 认为 0.3 m/s 只需要 ~1510，**1535 的前馈偏高**。
- 车道宽：1.378–1.440 m，中位 1.424 → 半宽 0.712 m，测量本身很稳。
- `plan_ms` 6.5–24 ms，Atlas 上完全够用。
- **停车那一下不是曲率 guard，是"宽度超时"**：`plan_width` 从 4.30 s 起就冻结在 1.428 m，4.40–4.80 s 五帧都没再更新，4.90 s 正好过了 **0.60 s > 0.55 s** 的过期阈值，`dense_reference.py:171` 直接 `return None` → `cone_corridor_lost`。也就是说：拐点处 `measure_width()` 连续 5 帧测不出有效宽度（对侧行只剩不到 2 个可用点，或宽度从 1.43 掉到 1.18），不是车真的没路。
- 最后一帧左右两行确实各有一个点落在**同一颗锥桶**上（间距 <3 cm），去重后基准行只剩 3 个点；4 点三次样条的曲率在 -1.40~+1.03 1/m 之间大幅摆动。`denominator` 这次最低 0.27，还没踩到 0.15，但余量已经很小——**这是下一个会踩的雷，不是这次停车的原因**。

> 注：记录里没有存 `speed/yaw`，所以"车到底转了多少"无法从文件直接读。但 4.2 s 满舵 + 横向偏差单调变大，已经把"舵量上限"这个嫌疑坐实到可以动手验证的程度。

### 4.1 两次实跑为什么都在同一个位置停

**第一次（22:34，target 0.25）**

- 13.2 s、102 帧、266 条命令，在 S 弯拐点 `stop = cone_corridor_lost`。
- 12.54 s 基准行从右侧切到左侧（`side -1 → +1`），4 帧后 `plan()` 返回 `None`。
- 回放最后一帧：左行样条算出的中心线在 x≈3.8 m 处**向回折了 0.9 mm**（`diff(centers.x) = -0.0009`），而代码里 `np.any(np.diff(centers[:,0]) <= 0)` 是**全局**判据，于是把整条 4 m 的路线都丢了——但前视点只在 0.68 m 处，根本不受远端影响。
- 已修：在第一个 x 回折处把中心线裁掉（保留近端）；如果回折出现在前 8 个点里，仍然拒绝。

**第二次（22:39，target 0.35）**

- 11.66 s、85 帧，还是 `cone_corridor_lost`，还是在同一个拐点。
- 11.29 s `side=-1`、`target=(0.99,-0.07)`；11.39 s 直接 `None`，没有可见的换边。
- 回放最后一帧：左侧行密度 1.53 > 右侧 0.92，换行逻辑决定切到左行；但左行只有 4 个点、拟合出的中心线和已经走过的路线差 **1.118 m**（允许 0.281 m）。旧代码是"先切过去、再检查"，检查失败就 `return None`。
- 已修：换行前先比对候选行中心线与当前路线，偏差 >0.2×车道宽就**不切**、保持旧行，等新行一致了再切；旧行真的消失（不可用）时才强制接受新行。
- 回放：第二次 85/85 帧、第一次 102/102 帧全部能规划；在当前拐点位置 `--observe` 也稳定输出 `dense_right`。

**第三次（22:49，target 0.6，跑到第二弯入口）**

- 9.95 s、81 帧，`cone_corridor_lost`；这次已经过了第一弯、到了第二弯入口。
- 基准行在第二弯前切到了左行（`side=+1`），不是没切；问题是左行只有 4 个点，样条曲率冲到 **±1.4 1/m**。
- 半宽约 0.76 m 时 `1 + side·d·κ` 最小只有 **0.108 ≤ 0.15**，`plan()` 判定"中心线比车的最小半径还紧"，直接返回 `None`。
- 已修：在按密度选行之前，先对左右两行各算一次偏移中心线的 `denominator`；几何上不可行的行直接剔除，用另一侧。回放三次实跑全部帧都能规划，最后一帧改用右行、`target=(1.24,0.07)`。
- 根本问题仍是 4 点样条的曲率噪声；这次先用"换一侧"兜住，后续再做曲率平滑（7.4）。

**速度**

- 第一次 0.25 m/s、第二次 0.35 m/s；驱动段没有一帧 1500，不是走走停停，纯粹是目标速度低。
- 现在 `target_speed_mps=1.20`、`maximum_pwm=1600`；1.2 m/s 下旧安全窗要求 2.14 m 空路，S 弯里必然走走停停，已放宽（见 4.2）。

### 4.2 1.2 m/s 的两次问题（23:04 / 23:07）

- **停在原地不动（23:04）**：165 帧全部 `target=None`、335 条命令全是 1500/84.5。那个位置两排锥桶在视野里不平行，宽度样本 0.84~2.96 m，`measure_width` 的中位数±20% 把样本全滤掉，`width` 一直没初始化。已加“左右最近锥桶对”作为初始化兜底（算出来 1.42 m）。
- **不丝滑、中间停一次（23:07）**：154 条命令里 100 条是 1500。旧安全窗 `required = .12+.35v+v²/.9` 在 1.2 m/s 时是 **2.14 m**，比 S 弯走廊还长，车一直 `ALIGN_ROUTE`/neutral；最后在左行旁边不走也是同一原因。已放宽为 `required = .12+.25v+.25v²`（1.2 时 **0.78 m**，视界 0.86 m），这是拿安全余量换速度，跑的时候要留神。

---

## 5. 建议的起步配置

JSON 用这份（`dense_settings_recommended.json`）：

```json
{
  "target_speed_mps": 1.2,
  "distance_mode": "half_width",
  "lookahead_base_m": 0.70,
  "lookahead_speed_seconds": 0.25,
  "transition_extension_m": 0.30,
  "preview_heading_weight": 0.35,
  "speed_kp": 50.0,
  "speed_ki": 12.0,
  "speed_feedforward_pwm": 1535.0,
  "launch_pwm": 1548.0,
  "maximum_pwm": 1600.0,
  "angle_kp": 0.8,
  "angle_kd": 0.26,
  "angle_derivative_filter_seconds": 0.20,
  "max_steer_deg": 35.0,
  "full_steer_deg": 65.0
}
```

这份 JSON 就是车上**当前生效**的 `dense_settings.json`。`max_steer_deg=35` 只是起点：先按 Step 2 实测舵量–半径，再决定是 30 还是 40；`full_steer_deg=65` 是"满舵命令"，用于安全窗的曲率换算，按实车满舵填。

各项理由：

- `target 0.25`：先把转向调稳，再加速度；
- **速度前馈先保持 1535/1548**：这次记录的稳态 PWM 只有 1505–1512，说明 ff 偏高约 25 us，但 `publish()` 里 `launch_pwm` 只是**上限**、不是起步助推，贸然把 ff 降到 1515 可能连静摩擦都破不了。等 Step 4 直道标定后再单独改；
- `speed_kp 50 / ki 12`：先降低积分拖尾，前馈准了再降 ki；
- `kp 0.9 / kd 0.22`：配合舵量放到 35，先在"能转"和"不摆头"之间取保守值；如果 `--observe` 里 PD 仍长期贴近新限幅，再把 kp 往 1.1–1.2 加；
- `lookahead 0.70 / 0.6`：比原来略长、速度项略小，弯里更顺；
- `extension 0.30`：出弯短行时少丢走廊。

---

## 6. 调参顺序（一次只改一件事）

### 实车测试的启动顺序

```bash
# 窗口 A（root）：起激光雷达 + IMU + 编码器 + 底盘驱动，保持前台运行
cd /root/racecar/tools
./start_sensors.sh

# 窗口 B（root）：确认三个话题都有数据
source /opt/ros/humble/setup.bash
ros2 topic hz /scan        # ~10 Hz
ros2 topic hz /IMU_data
ros2 topic hz /encoder

# 窗口 B：先只观察（8 秒，不发运动指令）
cd /root/racecar/tools/s_bend
./run_dense.sh --observe

# 窗口 B：实跑（会动车）
./run_dense.sh
```

停车按 **Ctrl-C**，节点退出前会连发 6 帧中立；不要 `kill -9`。兜底才是
`cd /root/racecar/tools && ./stop_car.sh`（它发的是 motor 1500 / servo 90，不是精确中位）。
底盘没有看门狗，进程被硬杀后最后一条指令会保持，所以手要放在电源开关上。

### Step 0：核对常量（不改代码，只确认）

- 直线用 `--fixed-servo 84.5` 或手推，车不跑偏 → `servo_center=84.5` 对；
- 手动左打角，`/car_cmd_vel.angular.z` 变大 → 符号对（>84.5 左）；
- `laser_x=0.23`、`half_width=0.175`、`margin=0.055`、`front=0.40`、`rear=0.16` 用卷尺复核；
- 直道 2 m，对比 `/encoder` 积分和卷尺 → 不对就改 `encoder_scale`（现在 1.009）。

### Step 1：只观察，不动车

```bash
cd /root/racecar/tools/s_bend
./run_dense.sh --observe
```

8 秒内看日志：

- `baseline=dense_left/right` 是否稳定，`width` 是否 1.40±0.05；
- `PD=±16.0deg` 是否长期贴顶 → 贴顶就是舵量不够；
- `preview` 是否在 0.45–1.2 内，`target` 的 x 是否经常 <0.3（前视太短/外推不够）；
- `plan_ms` < 20 ms。

### Step 2：确认舵量上限（现在是 JSON 参数）

先在平地/架子上实测舵量–转弯半径（PWM 1520 慢慢走，分别给 84.5±16、±30、±45，量直径）：

- 如果 16 对应的半径 ≫ 车道中心线半径（1.2–1.5 m），就把 `max_steer_deg` 加到 30–40；
- 取"半径 ≈ 0.8×目标中心线半径"那一档，通常 **30–40**；
- 只改 `dense_settings.json` 的 `max_steer_deg`，`AnglePD`、`choose_steering` 和安全窗都会自动跟随；
- 如果实车满舵不是 ±65，改 `full_steer_deg` 让安全窗的曲率换算对得上；
- 如果 16 已经能给出 0.8 m 半径，就别再放大——那是抓地/速度问题，不是舵量问题。

验收：`--observe` 里 `PD` 不再长期贴顶，峰值在 `max_steer_deg` 的 70% 以内。

### Step 3：转向 PD

顺序：`angle_kp` → `angle_kd` → `preview_heading_weight` → `lookahead`。

- kp 从 0.9 起，每次 ±0.1；看弯中是否"入弯不够"（偏外）或"出弯甩头"（过冲）；
- 甩头先加 kd（0.22→0.30），不要先减 kp；
- 直道也往弯里偏 → 降 `preview_heading_weight`（0.45→0.35）；
- 弯中切内太多 → 加 `lookahead_base_m`；入弯反应慢 → 减 0.05。

验收：全程舵量不长期贴顶，S 弯两个反向点能明显看到舵量过零。

### Step 4：速度

1. 直道分别用 PWM 1505 / 1515 / 1525 / 1535 走 2 m，记录稳态 `/encoder` 速度；
2. 取最接近目标速度的那一档做 `speed_feedforward_pwm`；
3. `launch_pwm = ff + 15` 左右；`maximum_pwm` 保持 ≤1600；
4. 再把 `target_speed_mps` 从 0.3 → 0.45 → 0.60，每次只加 0.15。

注意 `launch_pwm` 在代码里只是**上限**，不会主动把 PWM 顶上去：静止时实际输出是 `ff + kp·error + 积分`，再和 `launch` 取小。改小 ff 后如果车不起步，先把 `launch` 加回 1548、或临时给一脚（起步前 0.3 s 直接发 1548），别用降 ff 的办法硬扛。

验收：`v` 在目标 ±0.03 m/s 内，PWM 不再从 1548 掉到 1505 那种大起大落。

### Step 5：拐点鲁棒性（见第 7 节）

只有在 Step 2–4 都稳了还出现 `cone_corridor_lost` 时，才动这里的代码。

---

## 7. 拐点处要动的代码（7.1/7.2 已改）

### 7.1 左右行共用一颗锥桶（已改）

原阈值只有 0.03 m；拐点处两行在图像里靠得很近，同一颗锥桶会被同时收进左右行，导致基准行被截断、宽度骤降（这次从 1.43 掉到 1.18）。
现在：`shared < 0.12 m`，并且被共享的那颗**从两行都删掉**再各自拟合（`dense_reference.py:143`）。

### 7.2 宽度一旦测不到，1.0 s 后才丢路线（已改）

原逻辑是 `measure_width()` 返回 `None` 时沿用旧宽度，但 `now - width_at > 0.55` 就整体 `return None`（当时的 `dense_reference.py:171`）。这次停车就是 4.30 s 后宽度冻结、4.90 s 正好 0.60 s 超时。
现在：

- 过期时间 0.55 → **1.0 s**；
- `measure_width` 的 x 范围从 ±0.15 m 放宽到 **±0.40 m**；
- 宽度更新改为 `0.15` 的 EMA，且 `|observed-width| > 0.20 m` 时**拒绝本次更新但仍算"新鲜"**（防止 1.43→1.18 跳变把中心线拽偏，又不会因为一段噪声直接超时）；
- 拐点前后 `plan_width` 不再连续 5 帧一动不动。

验收：停车原因里不再出现 `cone_corridor_lost`；如果还出现，再考虑把 1.0 s 放到 1.5 s 或加 `width_stale` 日志。

### 7.3 拐点换行与中心线回折（已修，两次实跑停车的原因）

**7.3.1 中心线 x 回折**

`plan()` 原来用 `np.any(np.diff(centers[:,0]) <= 0)` 做全局单调性判据。S 弯拐点换基准行后，拟合出的中心线远端会轻微回折（第一次 -0.9 mm @ x≈3.8 m），整条路线就被判死；而前视点只用到 0.68 m 处。
现在：在第一个回折处裁掉远端（`centers/tangent/curvature/denominator` 一起裁），前 8 个点内回折才拒绝。回放 102 帧全部通过。

**7.3.2 换行前先比对，别"先切后拒"**

第二次停车时，左侧行密度 1.53 > 右侧 0.92，换行逻辑决定切到左行；但左行只有 4 个点、中心线与已走路線差 1.118 m（允许 0.281 m）。旧代码先 `side=candidate`，再做 residual 检查，检查失败就返回 `None`。
现在：候选行只有和当前路线差 ≤0.2×车道宽时才切换；否则保持旧行并清空 pending，等新行一致再切。旧行真的不可用时才强制接受新行。回放两次实跑全部帧都能规划。

**7.3.3 几何不可行的行先剔除**

第三次实跑时左行只有 4 个点，样条曲率到 ±1.4 1/m，偏移中心线的 `1 + side·d·κ` 最小 0.108（判据 0.15），`plan()` 直接 `None`。
现在：在密度投票之前，对左右两行各用对侧行测一次宽度、算一次 `denominator`；低于 0.15 的行直接剔除，只在剩下可行的行里选密度。最后一帧因此改用右行，`target=(1.24,0.07)`，不再停车。
这只是兜底；根源还是 4 点样条的曲率噪声，见 7.4。

### 7.4 4 点样条的曲率噪声

`fit_curve` 对 3–5 个点做自然三次样条，端点曲率会冲得很高（这次 -1.40~+1.03 1/m，对应半径 0.71~0.97 m，已经贴住车的最小半径 0.83 m）。
`plan()` 的 `denominator <= .15` 本来是对的（等价于"中心线半径不得小于车的最小半径"），但被噪声触发就会停车。建议：

- 对 `curvature` 做 5–7 点滑动平均后再算 `denominator`；
- 或者把判据改成"平滑后仍超限 + 连续 2 帧"才拒绝。

### 7.5 让 `plan()` 说清楚为什么返回 None

现在 `target=None` 有 6 种原因（无行、无宽度、宽度过期、曲率超限、中心线 x 不单调、外推超长），日志里分不清。
在 `plan()` 每个 `return None` 前加一个 `self.reason='...'`，在 `update_target` 里打印，下一次跑就知道停在哪一条。
这次如果能打出 `reason=width_stale`，就不用像现在这样靠回放推断了。

---

## 8. 每次跑完看什么

把 `dense_last_run.json` 拷回电脑，用同目录的 `analyze_dense_run.py`：

```bash
python3 analyze_dense_run.py dense_last_run.json --limit 35
```

它会打印：

- 停车原因、时长、帧数；
- 舵量直方图 + **贴限幅比例**（>20% 就说明限幅/增益有问题）；
- PWM 中位/最大、速度环是否长期贴 1500 或 1600；
- 宽度、前视、`plan_ms`、换边次数；
- 中心线相对车的横向偏移随时间变化；
- 宽度是否在停车前被"冻结"（宽度连续多帧不变 = `measure_width` 失败；现在 1.0 s 后才会丢路线）；
- 最后一帧的左右行共享锥桶与曲率诊断。

验收指标（按顺序满足）：

| 指标 | 目标 |
|---|---|
| 舵量贴限幅比例 | < 15% |
| 中心线横向偏移 | 全程 |e| < 0.20 m，不单调变大 |
| 换边次数 | 一个 S 弯 1 次（多于此是拟合抖） |
| `width` | 1.40 ± 0.05 m，单帧跳变 < 0.08 m |
| `plan_ms` | < 20 ms |
| 停车原因 | 到终点/手动，而不是 `cone_corridor_lost` |

---

## 9. 一句话版执行清单

1. 先跑 `--observe`，看 `PD` 是不是长期贴 `max_steer_deg`；是，就实测量舵量–半径再调 30–40；
2. 当前 JSON 已生效（target 0.25、max_steer 35），低速实跑；
3. 看 `analyze_dense_run.py` 的贴限幅比例和横向偏移，按 Step 3 调 kp/kd/preview/lookahead；
4. 直道标 PWM–速度，改 ff，再把 target 加回 0.30；
5. 还丢走廊，按 7.4（曲率平滑）→ 7.5（拒绝原因日志）继续修；7.1/7.2/7.3 已经改完。

---

## 10. 本次落地的改动（2026-11-05 22:11 / 22:37）

备份：`/root/racecar/tools/s_bend_backup_20261105_221117/`（22:11 之前）、
`/root/racecar/tools/s_bend_backup_20261105_223707/`（22:37 之前）。

`dense_settings.json`：

- `target_speed_mps` 0.30 → 0.25 → 0.35 → 0.60 → **1.00**（实跑后逐级提速）
- `lookahead_base_m` 0.65 → 0.70
- `lookahead_speed_seconds` 0.8 → 0.6
- `transition_extension_m` 0.25 → 0.30
- `preview_heading_weight` 0.5 → 0.45
- `speed_kp` 60 → 50，`speed_ki` 20 → 12
- `angle_kp` 1.2 → 0.9，`angle_kd` 0.15 → 0.22，微分滤波 0.15 → 0.20
- 新增 `max_steer_deg: 35.0`、`full_steer_deg: 65.0`
- `speed_feedforward_pwm` / `launch_pwm` / `maximum_pwm` 保持 1535 / 1548 / **1600**

`dense_s_bend.py`：

- 新增 `max_steer` / `full_steer` 读取与校验（10 ≤ max ≤ full ≤ 90）
- `AnglePD` 传入 `max_steer`
- `choose_steering` 用 `self.max_steer` 替代硬编码 16；`max_steer > 20` 时安全窗候选扩到 ±20
- `clearance()` 的曲率改为 `(offset/full_steer)/car.radius`

`dense_reference.py`：

- `AnglePD` 增加 `max_steer` 参数（默认 16，向后兼容）
- 共享锥桶阈值 0.03 → 0.12 m，且从两行同时删除
- 宽度过期 0.55 → 1.0 s；x 范围 ±0.15 → ±0.40 m
- 宽度 EMA 0.2 → 0.15，单帧变化 >0.20 m 时拒绝更新但仍刷新时间戳
- 中心线 x 回折：全局 `diff(centers.x) <= 0` 改为"在第一个回折处裁掉远端，前 8 点内回折才拒绝"（22:37，第一次实跑停车的原因）
- 换行前比对：候选行中心线与当前路线差 >0.2×宽度时不切换、保持旧行（22:42，第二次实跑停车的原因）
- 几何预筛：密度投票前剔除 `denominator <= 0.15`（中心线比车最小半径还紧）的行，用另一侧（22:52，第三次实跑停车的原因）

新增（都在 `s_bend` 内）：`set_param.py`（改参数 + 备份 + 校验）、`analyze_dense_run.py`、本指南。

未改：`vehicle_io.py`（85°/s 速率限制）、`geometry.py`、`run_dense.sh`。
实车 15 s `--observe` 冒烟测试通过：节点能构造、能收 `/scan`、无异常退出；真实 `dense_last_run.json` 未被覆盖。

注意：`run_dense.sh` 每次启动会删掉同目录的 `s_bend.py`、`corridor.py`、`run.sh`、几个 `*_scan.yaml`、`last_run.json` 等旧文件；别把要留的东西命名成这些。
