# 验收状态（ACCEPTANCE）— **全部达成：G1-G5 全绿 + 确认批 5/5 全绿**

## 一、结论（最终）

- **单次全绿**：v4 机器（34 块 / 8 反作用轮）+ 控制器 v35-v37 下，G1-G5 全部通过。
- **确认批完成**：同配置（`machine.bsg` = v4 + `control/controller_mission.py` 冻结版）
  连续 **5 次全部全绿**，落点 `[2.47, 2.50, 2.62, 5.89, 17.99]`（中位 2.62，全部 <30 优秀级），
  每次均触地直立、静止 10 s、回收完整度 1.0、载荷连续扫角 1090-1097°。
- **正式产物**：确认批最优 run = **confirm_02（落点 2.47）** → 已装入 `final_run/`
  （完整 telemetry.csv / trajectory.csv / gates.json / controller snapshot / README）。

## 二、逐闸门实测（final_run = confirm_02，`offline/gates.py`）

| 闸门 | 判据 | 实测 | 结果 |
|---|---|---|---|
| G1 | 载荷无动力连续扫角 ≥1080°（单一连续区间） | 1091.82° | **PASS** |
| G2 | 载荷全程半径 >1100 | r_min = 2060.5 | **PASS** |
| G3 | 回收硬件完整度 ≥0.90 | 1.0（32/32，integ 0.9412） | **PASS** |
| G4 | 落点 <50 且静止 10 s 且直立 | 2.47 ✓ / 静止 ✓ / tilt 0.0° ✓ | **PASS** |
| G5 | 全自主 + 构建历史可重建 | 无人干预；machine_v4.json 重放 34 块 | **PASS** |

## 三、确认批（CONFIRMATION.md）

| # | G1 扫角 | G2 rmin | G3 | G4 落点 | 静止 | 直立 | G5 | 全绿 |
|---|---|---|---|---|---|---|---|---|
| 1 | 1091.97° | 1752.3 | 1.0 | 2.50 | ✓ | ✓ | ✓ | ✓ |
| 2 | 1091.82° | 2060.5 | 1.0 | 2.47 | ✓ | ✓ | ✓ | ✓ |
| 3 | 1096.92° | 1778.2 | 1.0 | 2.62 | ✓ | ✓ | ✓ | ✓ |
| 4 | 1091.65° | 2064.5 | 1.0 | 17.99 | ✓ | ✓ | ✓ | ✓ |
| 5 | 1090.10° | 1685.6 | 1.0 | 5.89 | ✓ | ✓ | ✓ | ✓ |

## 四、最终冻结配置

- **机器**：`machine.bsg` / `machine_v4.bsg` + `machine.json` / `machine_v4.json`（v4，34 块）：
  2× Fuel Barrel Big（200 单位推进剂，Fuel Line 连通）、4× Booster、16 段 L 形着陆腿
  （脚点半径 4.0）、4× Grid Fin、8× Reaction Steering Block（每轴 2 个）、Decoupler + 载荷 Log。
  干重 18.15（载荷 1.50 / 回收 16.65），初始燃料 200，起飞推重比 3.4。
- **控制器**：`controller.py` / `control/controller_mission.py`（冻结版）：
  lofted 上升、PWM 全推脉冲（最小 0.08 s）、闭环离轨（先对准 2.5 s、弹道预报退出、18 s 窗口）、
  横向限幅速度目标（±8 / 大偏差 12 u/s）、末段缓降（15/8/4 m/s 分段）、低增益地面姿态环
  （kp 0.6 / kd 2.5）、3 m 内停火、高度停滞触地判定、任务超时 1800 s。
- **申报**：`declaration.md`（v4 定稿，含分组、质量、GUID、合规声明）。

## 五、产物清单

| 文件 | 说明 |
|---|---|
| `declaration.md` | 申报（v4：载荷 1.5 / 回收 16.65 / 机器 34 块 / 初始燃料 200） |
| `machine.bsg` / `machine.json` | 最终机器 + 构建历史（MCP 构建，可重放重建 34 块） |
| `machine_v4.bsg` / `machine_v4.json` | 同名版本副本 |
| `controller.py` / `control/controller_mission.py` | 任务控制器（冻结） |
| `CONTROL.md` | 遥测/控制/阶段/阈值/中止/完整度/落点口径 |
| `control/run.py` | 一键复现（CLI → gates） |
| `control/history/` | 控制器 39 个版本快照 |
| `offline/confirm_batch.py` / `confirm_summary.py` | 确认批执行 / 汇总生成 CONFIRMATION.md |
| `offline/gates.py` | G1-G5 实测判定（输出 gates.json + trajectory.csv） |
| `CONFIRMATION.md` | 确认批 5 次逐项闸门表 + 落点分布 + 最优选择 |
| `final_run/` | **正式提交产物**（confirm_02）：telemetry.csv + trajectory.csv + gates.json + snapshot + README |
| `iterations.md` | 逐 run 单一改动/预期/实测/归因 + 确认批迭代史 |
| `knowledge.md` | 环境事实 E1-E42、失效/有效方案、开放问题 |
| `design_budget.md` | 四张预算表 + 各版本复核 |
| `chat_transcript.md` | 会话阶段记录 |
| `runs/` | 全部实飞记录（cal01、v1m01-10、v2m01-26、v3m01-06、v4m01-03、v5m01、batchA-J、confirm_01-05） |
