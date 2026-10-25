# CONTROL.md — S02 任务控制器说明（v6）

控制器：`control/controller_mission.py`；一键复现：`control/run.py`
机器：`machine.bsg` + `machine.json`（构建历史）；申报见 `declaration.md`。

## 1. 遥测输入（ControllerClient → BAT4，25 Hz，full profile）

| 字段 | 用途 |
|---|---|
| `position`, `velocity`（每块） | 世界状态；根块（Starting Block）= 回收硬件跟踪参考块；Log 块 = 载荷跟踪参考块 |
| `rotation`, `angular_velocity`（每块） | 姿态/角速度；轮转速 =（轮块角速度 − 根块角速度）·轮轴 |
| `fuel`（Fuel Barrel Big） | 推进剂存量（满 100 单位）；质量 = 干重 + 0.035×fuel |
| `machine_integrity`, `alive_block_count` | 完整度 fail-fast；完整度口径见 §6 |

状态重建：
- 星球中心 `C=(0,-600,0)`，`mu=8,456,800`，`R=600`，`g=min(22, mu/r²)`。
- `n=(p−C)/|..|`；`vr=v·n`；`vt=|v−vr·n|`；表高 `h=|p−C|−R`；地面相对高度 `h_rel=h−ground_alt`
  （`ground_alt` = 发射架上根块表高，实测 5.35）。
- 机体系：`M = R_root_live · R_root_authored⁻¹`（BSG 构建坐标 → 世界）。
  **构建坐标系的"上"/推力轴 = 世界 +Y 方向**（BSG 坐标 y=up）；`thrust_dir = M·(0,1,0)`。
  注意：display 的 [East,North,Up] 是 BSG (x,z,y) 的换序，写控制器时以 BSG 坐标为准。
- 飞行平面：`n0`=发射点径向，`d0`=世界 +Z 的水平投影，平面法向 `p=n0×d0`（≈世界 +X）。
  平面内水平方向 `h_hat = p×n`；轨道相位角 `θ = atan2((p−C)·d0, (p−C)·n0)`（在轨道平面内，
  不能用世界 x-y 的 atan2）。
- 轨道根数：二体公式（判据/监视用）；**闸门以 trajectory.csv 实测为准**。

## 2. 控制输出（每轮一次 `send_indexes`，仅变化时发送）

| 执行器 | 通道 | 律 |
|---|---|---|
| 4× Booster | `ThrustKey` + `fthrust=2.0` | **全局 PWM**：需求 duty∈(0,1) 时按「fire_timer/pwm_acc」累积成满推脉冲；duty≥0.985 连续开；duty≤0 关。原因：实测 fthrust 与推力强非线性（半档只有 31% 推力）且半档燃耗效率差 3-4 倍。 |
| 4× Reaction Steering Block | `speed` + `LeftKey/RightKey` + `damper=0` | 目标轮速 = 当前轮速比 − K_w·(α·轴)，K_w=0.16，限幅 ±0.48；α = kp·(d×d_cmd) − kd·ω⊥（PD） |
| 1× Decoupler | `ExplodeKey` | 分离窗口内保持 0.4 s |

`a_max = F_FULL/m`，`F_FULL=2030`（4 引擎满推合力，cal01 实测），`m = 干重 + 0.035×fuel`。

## 3. 阶段转移（状态机）

| 阶段 | 目标/律 | 转移条件 |
|---|---|---|
| `pad` | 记录发射参考系；轮控保持 | t>0.8 s |
| `ascent` | **垂直满推**（推力轴=径向外，气动对称 → 姿态稳定），PWM 满推脉冲 | 预测弹道远地点 `r_apo=1/(1/r−vr²/2μ) ≥ 2100` |
| `coast_up` | 关车，尾部朝上滑行 | `vr ≤ 0.5`（到达远地点） |
| `pitch` | 真空段俯仰程序：期望方向从 n 以 1.2 rad/s 转向水平 `h_hat` | 转到 90° 且 `thrust_dir·h_hat>0.93` |
| `ins_burn` | 水平入轨点火（+3.4° 仰角），PWM 满推 | `vt ≥ 0.99·v_circ`（或 rp≥1320 且 |vr|<20）→ cutoff |
| `coast` | 轮控保持、尾部朝上 | `rp>1180` → 触发分离（Decoupler 通电 0.4 s） |
| `separate` | 跟踪载荷块 | 两体距离超过初始距离+2 或 3 s |
| `orbit` | 轮控保持尾部朝上；每 0.4 s 用弹道预报（含标定阻力）计算「现在反推 Δv=v−15」的落点相位 | `abs(err)<0.04 rad` 或相位误差过零 → 离轨；`abs(err)<0.60` 时提前对准反推方向 |
| `deorbit` | 反推（推力轴对准 −v 且余弦>0.90 才点火） | 速度 ≤ 15 u/s |
| `fall` | 弹道下落，轮控保持尾部朝上（推力轴=径向外） | h ≤ 700 |
| `brake` | 满推反推（对准余弦>0.75） | 速度 ≤ 32 u/s |
| `land` | 高度比例下沉率 `vr_ref=−max(1,min(0.16·h_rel,16))`，垂直推力 = g + 3.5(vr_ref−vr)（vr<−32 硬刹车）；水平 PD（`0.7·herr_signed − 2.5·vh`，倾角限 ±30°×min(1,h/45)） | h_rel<0.7 且 speed<1.2 → 关车 |
| `wait` | 关车；持续累计载荷扫角/最小半径；回收机静止计时 | 载荷扫角≥1080°、载荷全程 r>1100、回收机静止≥10 s |
| `done` | 写 mission_summary 退出 | — |

注（重要，来自 v1m01-v1m10 的失败）：细长机体在大气段被气动风标力矩主导（实测扰动力矩
大于轮控权限），**不能要求机体把推力指向任意方向**；因此上升段采用"垂直满推 → 高空滑行 →
真空俯仰 → 水平点火"的 lofted 剖面（E23），下降段只用"垂直推力 + 小倾角水平修正"。
另有两点工程约束：(a) 引擎只在 throttle>0.5 点火 → 所有推力需求统一经全局 PWM 转成满推脉冲；
(b) 尾部朝上的机体无法把推力指向水平/下方 → 不设独立 approach 阶段。

## 4. 关键阈值（本会话实测/离线标定）

| 量 | 值 | 出处 |
|---|---|---|
| 满推合力 F_FULL | 2030 force units（4 引擎） | cal01 25 Hz 遥测 |
| 满推燃料流量 | 7.69 单位/s | cal01 |
| 阻力模型 | `a_drag = 0.028·e^(−h/90)·v²` | cal01（h 5-180 实测；h>300 外推，末段闭环补偿） |
| PWM min pulse | 0.16 s | 引擎点火上升时间 ~0.1 s |
| 上升目标轨道 | 近地点 ≥1320、远地点 1500-1800 | 多轮实飞 |
| 离轨目标速度 | 25 u/s | offline/descent_sim3.py + 实飞 |
| 制动高度/目标速度 | h=450 / 32 u/s | offline + 实飞 |
| 着陆下沉率 | h_rel<83 时 ≤0.3·h_rel，最低 1 u/s | offline |
| 轮控 PD | kp 0.9-2.2，kd 1.2-1.8，K_wheel=0.16，cap 0.48 | cal01 轮响应 + 实飞 |

## 5. 快速中止条件（fail-fast）

- 完整度 < 0.90（任意时刻）
- 上升段 t>80 s 仍未满足入轨判据
- 燃料 ≤ 1 单位且不在 `wait/done/land` 阶段
- 总时间 > 900 s（仿真秒）
- 中止 = 关引擎（fthrust=0、释放按键）+ 记 `abort_reason` + 写 summary 正常退出。

## 6. 完整度口径

- 申报分组基线：**载荷组** = Log 载荷块 + Decoupler（设计分离，不计损伤）；
  **回收硬件组** = 其余全部发射块（起始块、主油箱、4 引擎、8 着陆腿块、4 尾翼、4 反作用轮）。
- 完整度 = 回收硬件组存活/完好比例。判定数据：轨迹遥测里的每块记录是否存在
  （块被摧毁后不再出现在遥测记录中）+ `machine_integrity` 交叉核对。
  设计分离不计损伤；分离后回收机掉块才计入。
- 判定：着陆静止后连续 10 s 内完整度 ≥0.90。

## 7. 落点测量口径

- 跟踪参考块 = **起始块**（回收硬件申报跟踪块，GUID 见 `declaration.md`）。
- 着陆偏差 `d = R·arccos(clamp(n_final·n_target))`；
  `n_final` = 跟踪参考块**静止最终位置**的径向单位矢（静止判定通过后的最后位置，含滑滚位移），
  `n_target` = 发射场目标方向 = 发射时跟踪参考块初始位置的径向单位矢。
- 不用"离目标最近的幸存块"，不用触地瞬间位置。
- 逐项实测在 `offline/gates.py`，输出 `gates.json`。

## 8. 产物与复现

- `control/run.py`：一键复现（调用官方 `besiege_cli run` → 合并 telemetry → 生成
  `trajectory.csv` → 闸门检查）。
- `final_run/trajectory.csv`：覆盖 launch→Evaluation Endpoint 全程；列 = 时间、参考块位置/半径/
  地面相对高度、载荷位置/半径、载荷累计扫角。所有值均取自遥测实测（无轨道外推）。
- `final_run/gates.json`：G1-G5 逐项实测判定。
