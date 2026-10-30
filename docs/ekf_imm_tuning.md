# EKF / IMM 调参手册

**参数在哪里、调大调小会怎样、遇到问题调哪个。**

---

## 0. 参数在哪个文件

| 文件 | 放什么 | 改完要重编译吗 |
|---|---|---|
| **`configs/IMM.yaml`** | 几乎全部可调参数（模型、Q、R、P0、Huber、一致性） | **不用**，直接跑就生效 |
| `tasks/auto_aim/target.cpp` | 状态定义、观测函数 `h`、装甲板匹配逻辑 | 要 |
| `tools/extended_kalman_filter.hpp` | 通用 EKF 实现（一般不动） | 要 |

**绝大多数调参只需要改 `configs/IMM.yaml`。**

程序启动时会自动去**主 config 同目录**找 `IMM.yaml`（例如 `configs/sentry_example.yaml` → `configs/IMM.yaml`），所以部署到 `install/` 后同样成立；读不到时用代码内默认值，不会崩。

---

## 1. 怎么改

1. 编辑 `configs/IMM.yaml`
2. 跑 `ekf_bench` 看分数
3. 满意就直接用，实车程序启动时自动读取

```bash
# 改完直接跑，不用编译
./cmake-build-debug/benchmarks/ekf/ekf_bench run --out logs/ekf_bench/try
./cmake-build-debug/benchmarks/ekf/ekf_bench compare --a cur --b try

# 也可以临时指定另一份配置做 A/B
./cmake-build-debug/benchmarks/ekf/ekf_bench run --imm /tmp/IMM_2.yaml --out logs/ekf_bench/try2
```

**两个注意**：

- 文件不存在或某个键写错 → 用代码里的默认值（和本文件一致），不会崩。
- `transition.diag + 2 × off_diag` 必须等于 1，否则程序会自动按 `diag` 修正并打警告。

---

## 2. 参数分三类

| 类别 | 有哪些 | 能不能每个模型单独设 |
|---|---|---|
| **模型专属** | `models[*].sa2`、`v2`、`ca`、`damping`、`motion.*` | **必须每个模型一套**，这就是 IMM 的“假设” |
| **IMM 层** | `transition.*`、`probability.*` | 三个模型共用一套规则 |
| **全体共享** | `measurement.*`、`initial_covariance`、`huber.*`、`consistency.*` | **三个模型必须完全一样** |

**为什么 `R` 不能按模型调**：三个模型看的是同一批观测，`R` 描述的是“PnP 有多不准”，跟目标怎么运动无关。给某个模型更小的 `R`，它的似然会被人为抬高，什么都不做就能赢，`mu` 就失真了。

---

## 3. 模型专属参数

### `models[*].sa2` —— 平移的“不守规矩程度”

| 模型 | 当前值 |
|---|---|
| `models[0]` 匀速 | 10 |
| `models[1]` 匀加速 | 10 |
| `models[2]` 静止 | 0.1 |

- **调大**：跟踪快，急变向不滞后，但稳态抖。
- **调小**：平滑，但急停急起慢半拍。
- 100 Hz 下，`sa2=10` ≈ 速度每帧抖 0.03 m/s。
- 建议范围 **1 ~ 50**。

### `models[*].v2` —— 角速度的“不守规矩程度”

| 模型 | 当前值 |
|---|---|
| 匀速 | 150 |
| 匀加速 | 400 |
| 静止 | 150 |

- **调大**：`w` 响应快，小陀螺跟得上，但 `w` 抖。
- **调小**：`w` 稳，但起转滞后。
- 建议范围 **100 ~ 600**。超过 600 会明显拖累远距离场景。

### `models[*].ca` / `damping` —— 模型的身份

| 模型 | `ca` | `damping` | 效果 |
|---|---|---|---|
| 匀速 | false | false | 加速度每帧清零 |
| 匀加速 | true | false | 加速度延续 |
| 静止 | false | true | 平移速度衰减 |

**一般不要动**，改了就不是原来的模型了。

### `motion.acc_damping` —— 匀加速模型的加速度衰减

当前 **0.90**。

- **调大**（接近 1）：加速度能无限漂移，机动猛，但远距离会把位置和 `yaw` 带抖。
- **调小**（0.85）：加速度更听话，稳态稳，但急变向慢。
- 建议范围 **0.85 ~ 0.95**。

### `motion.damping_tau` —— 静止模型的时间常数

当前 **0.30 秒**。

- **调大**：静止模型更像匀速模型。
- **调小**：静止判定更硬，但拖累平移跟踪。
- 建议范围 **0.2 ~ 0.5 秒**。

---

## 4. IMM 层参数

### `transition.diag` —— 模型切换速度

当前 **0.90**。

- 平均驻留 = `1 / (1 - 0.90)` = 10 帧 = **0.1 秒**。
- **调大**（0.95）：切换慢，`mu` 平滑，但要 0.2 秒才反应过来。
- **调小**（0.85）：切换快，但 `mu` 一直抖。
- 建议范围 **0.85 ~ 0.95**。

### `transition.off_diag`

当前 **0.05**，必须满足 `off_diag = (1 - diag) / 2`。

### `probability.initial`

当前 `[0.8, 0.15, 0.05]`。

- 只影响开头一两秒，一般不用动。

### `probability.floor`

当前 **0.001**。

- 防止某个模型被压到 0 后再也回不来。
- **调大**：冷门模型复活快，但污染输出。
- **调小**：低于 1e-4 容易永久锁死。

---

## 5. 观测噪声 `R`（最该先调的）

位置：`measurement` 段。

| 键 | 当前值 | 含义 |
|---|---|---|
| `yaw_var` | 0.004 | 方位角方差，σ ≈ 3.6° |
| `pitch_var` | 0.004 | 俯仰角方差，σ ≈ 3.6° |
| `distance_gain` | 1.0 | 距离方差 = `gain × log(|Δangle|+1) + offset` |
| `distance_offset` | 1.0 | 距离方差的最小值，σ ≈ 1 米 |
| `armor_yaw_gain` | 0.005 | 装甲板朝向方差 = `gain × log(distance+1) + offset` |
| `armor_yaw_offset` | 0.09 | 装甲板朝向方差的最小值 |

- **调大**：更信运动模型，平滑但滞后，`nis/dof` 变小。
- **调小**：更信观测，跟得快但抖，`nis/dof` 变大。
- **判断标准**：跑 bench 看 `nis/dof`，目标 **0.8 ~ 1.2**。当前只有 0.03~0.7，说明整体偏大。

**实测参考**（14 维 IMM，6 场景）：

| 配置 | `far_noisy` 的 nis/dof | `turn` pos_rmse | `uniform` pos_rmse |
|---|---|---|---|
| `gain=1.0, offset=1.0`（当前） | 0.71 | 0.134 | 0.171 |
| `gain=0.1, offset=0.05` | **1.39** | **0.090** | **0.116** |

`R` 缩小后平移/变向明显改善，但 `spin` 会退化约 35%（小陀螺的角度噪声被放大了）。所以 `R` 要**逐项调**，不要一刀切乘系数。

---

## 6. 初始协方差 `initial_covariance`

当前：

```
[1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1, 25, 25, 25]
```

顺序：`x vx y vy z vz a w r l h ax ay az`。

- **调大**：开头几帧收敛快，但抖。
- **调小**：开头保守，收敛慢。
- 只影响前几帧，之后会被观测拉平，**不用精调**。
- 唯一值得改的是 `r`（当前 1，即 σ=1 米，但真实半径只有 0.2 米）。

---

## 7. 鲁棒更新 `huber`

| 键 | 当前值 | 含义 |
|---|---|---|
| `enable` | true | 关掉即退化为标准 EKF |
| `k` | -1 | 降权阈值，自动 = `sqrt(χ²₀.₉₅(4)) ≈ 3.08` |
| `block_size` | 4 | 与 4 维量测绑定，**不要改** |

- `k` **调大**：更信观测，离群点影响大。
- `k` **调小**：更保守，正常观测也会被降权。
- 建议范围 **2.5 ~ 3.5**。

---

## 8. 一致性 `consistency`

| 键 | 当前值 | 含义 |
|---|---|---|
| `window_seconds` | 1.0 | NIS 统计窗口（秒） |
| `min_window_samples` | 20 | 判据生效前的最小样本数 |
| `nis_threshold` | -1 | 单帧 NIS 阈值，自动 95% 分位 |
| `tracker_fail_rate` | 0.4 | 窗口内失败率超过此值判 lost |

- `window_seconds` / `tracker_fail_rate` **调大** → 判 lost 更迟钝；**调小** → 更容易丢目标。

---

## 9. 遇到什么调什么

| 现象 | 先查 | 再调 |
|---|---|---|
| 急停急起跟不上 | `pred_pos` 是否明显大于 `pos_rmse` | `models[1].sa2` 调大；`transition.diag` 调小 |
| 匀速时位置/角度抖 | `pos_rmse` | `models[0].sa2` 调小；`transition.diag` 调大 |
| 小陀螺 `w` 追不上 | `w_rmse` | `models[0].v2` 调大；`transition.diag` 调小 |
| 远距离抖 | `far_noisy` 的 `nis/dof` | `measurement.distance_gain` 调小（见第 5 节） |
| `mu` 一直不变 | `nis/dof` 是否远小于 1 | 修 `measurement`，这是根因 |
| 频繁 lost | `nis_fail_rate` | 真失配就修 R/Q；否则调大 `tracker_fail_rate` 或 `window_seconds` |
| 单帧被错误识别带偏 | `huber_weight` 是否接近 1 | `huber.k` 调小 |
| 半径估到离谱值 | `diverged` 日志 | 调小 `initial_covariance` 里 `r` 的值 |

---

## 10. 调参顺序

1. **先修 `R`**（第 5 节）：把 `nis/dof` 拉到 0.8~1.2。这一步不做，后面都是白调。
2. **再调 `Q`**（`sa2` / `v2`）：一次一个模型、一个参数。
3. **最后调 `transition`**：只影响切换速度，不影响单模型精度。
4. 每次只改一个参数，跑全部 6 个场景，看综合分和逐场景变化。

---

## 11. 不要动

- **加速度必须在状态末尾**（11/12/13）。`aimer`、`planner`、`shooter` 都依赖 0~10 的索引。
- **`R` 不能按模型单独设**。
- **`huber.block_size` 必须是 4**。
- **静止模型不能阻尼角速度**，否则小陀螺的 `w_rmse` 会翻几倍。
- **`off_diag` 必须等于 `(1 - diag) / 2`**。

---

## 12. 当前分数

| 版本 | 综合分 |
|---|---|
| 单模型 EKF（基线） | 1.0000 |
| IMM 11 维 + Huber | 0.7410 |
| IMM 14 维（CA）+ Huber | 0.8800 |

14 维在近距离小陀螺更好，平移/变向略差。等 `R` 校准后重测。
