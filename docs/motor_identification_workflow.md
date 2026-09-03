# DAMIAO 手臂馬達：參數補齊、Torque FRF 與 Controller \(\omega\) 辨識流程

更新日期：2026-09-03

## 1. 最終目標

本流程的目標不是重新辨識整顆馬達，也不是用掃頻取代既有韌體參數。

目標是：

1. 讀取 DAMIAO 驅動器已經存在的電氣、馬達與控制參數，視為已知先驗。
2. 用少量針對性的硬體實驗補上「韌體參數表沒有提供、但手臂控制器建模需要」的機械參數。
3. 建立從 **command torque → joint position / velocity** 的實際頻率響應模型。
4. 由模型決定可用的 controller frequency scale \(\omega_{usable}\)，再用

\[
K_p(q)=M(q)\omega^2
\]

\[
K_d(q)=2\zeta\omega M(q)
\]

設計手臂控制器。

> 核心原則：**保留已知參數，只辨識缺失參數。**

---

## 2. 目前已知的 DM-J4310-2EC 參數

目前從驅動器 RID 讀出的參數：

```text
UV_Value: 15
KT_Value: 0
OT_Value: 100
OC_Value: 0.800000011920929
ACC: 2
DEC: -2
MAX_SPD: 600
MST_ID: 0
ESC_ID: 1
TIMEOUT: 0
cmode: 1
Damp: 0.00030128698563203216
Inertia: 1.7256099454243667e-05
hw_ver: 1445998643
sw_ver: 925970485
SN: 1412444213
POLE/NPP: 14
Rs: 0.7106770277023315
Ls: 0.000355415977537632
Flux: 0.004345269873738289
Gr: 10
PMAX: 12.5
VMAX: 30
TMAX: 10
I_BW: 1000
KP_ASR: 0.003719999920576811
KI_ASR: 0.0020000000949949026
KP_APR: 54
KI_APR: 0
OV_Value: 32
GREF: 1
Deta: 4
V_BW: 0
IQ_cl: 0
VL_cl: 0
can_br: 4294967295
sub_ver: 0
```

已知參數中，不應透過 Stage 4 FRF 重複「重新辨識」的主要項目包括：

- \(R_s\)：相電阻
- \(L_s\)：相電感
- \(\lambda\) / Flux：磁鏈
- NPP / pole pairs
- motor-side inertia（韌體辨識值，可作為先驗）
- gear ratio \(G_r\)
- current-loop bandwidth / controller gains
- PMAX / VMAX / TMAX：協議映射範圍

DAMIAO V1.4 協議說明指出，驅動器本身會做馬達參數標定，主要辨識相電阻、相電感、磁鏈等；Damp 只作參考且「未使用到」，Inertia 的單位為 kg·m²。

---

## 2.1 通訊層：以裝置內 PMAX / VMAX / TMAX 為準

`MotorType` 在 MIT CAN 編解碼中的主要作用，是選擇三個映射範圍：

\[
q \leftrightarrow [-P_{MAX}, P_{MAX}],\qquad
\dot q \leftrightarrow [-V_{MAX}, V_{MAX}],\qquad
\tau \leftrightarrow [-T_{MAX}, T_{MAX}]
\]

因此自動 probe 的第一優先不是猜完整實體型號，而是直接讀 RID 21/22/23：

```text
RID 21 = PMAX
RID 22 = VMAX
RID 23 = TMAX
```

只要這三個 register 有效，就直接建立該顆馬達的 protocol limits。內建 `MotorType` table 只保留作為顯示名稱與 register 讀取失敗時的 fallback。
CLI fallback 可接受 `DM4310P`、`DM4340P`、`DM8009P`，並映射到相同的 non-P protocol limit family。

目前目標產品的 protocol family：

```text
DM4310 / DM4310P   -> 同一組 limit family
DM4340 / DM4340P   -> 同一組 limit family
DM6006             -> DM6006 limit family
DM8006             -> DM8006 limit family
DM8009 / DM8009P   -> 同一組 limit family
```

P / non-P 的機械結構、摩擦或個別 Rs/Ls 等實測值可以不同，但如果 PMAX/VMAX/TMAX 相同，CAN MIT frame 的 position / velocity / torque scaling 就相同。後續 Stage 1~4 仍使用每顆馬達自己的 register 與實測資料，不假設 P / non-P 的機械參數完全一致。

韌體 `sw_ver` / `hw_ver` 只作診斷與型號提示，不再作為 protocol scaling 的權威來源，因為不同實體型號可能共享 firmware version family。

---

## 3. 真正需要補上的項目

實際手臂建模較缺：

- static / breakaway friction：\(\tau_s^+,\tau_s^-\)
- Coulomb friction：\(\tau_c^+,\tau_c^-\)
- effective viscous friction：\(B_{eff}\)
- effective load / joint inertia：\(J_{eff}(q)\) 或模型校正量
- gearbox / transmission torsional stiffness：\(K_g\)
- transmission damping：\(B_g\)
- flexible resonance / anti-resonance frequency
- modal damping ratio \(\zeta_r\)
- CAN + drive + computation effective delay
- 實際 torque 可用線性範圍 \(\tau_{max,linear}(\dot q)\)

這些才是 Stage 1~4 需要補的內容。

---

# 4. 四階段辨識流程

## Stage 1 — VEL characterization：動摩擦與 torque baseline

### 目的

量出 motor + gearbox 在不同穩態轉速下，維持運動所需的 torque。

這一階段 **不是找 stall torque，也不是找最大 torque**。

### 控制方式

```text
Control Mode: VEL
Command: multiple positive/negative steady velocities
Response: actual velocity + feedback torque + temperatures
```

例如可採多個正負速度工作點：

```text
-0.75 V_test
-0.50 V_test
-0.25 V_test
+0.25 V_test
+0.50 V_test
+0.75 V_test
```

每個速度點：

1. 等待速度進入穩態。
2. 捨棄 acceleration transient。
3. 記錄一段 steady-state torque。
4. 計算 torque mean / RMS / variance。

### 預期模型

第一版可用：

\[
\tau_f(\dot q)
\approx
\tau_c\,\mathrm{sgn}(\dot q)+B\dot q
\]

若正負方向明顯不對稱：

\[
\tau_f(\dot q)=
\begin{cases}
\tau_c^+ + B^+\dot q,& \dot q>0\\
-\tau_c^- + B^-\dot q,& \dot q<0
\end{cases}
\]

### Stage 1 輸出

```text
coulomb_friction_positive_nm
coulomb_friction_negative_nm
viscous_friction_positive_nms_rad
viscous_friction_negative_nms_rad
torque_noise_rms_nm
steady_torque_vs_velocity.csv
```

---

## Stage 2 — Breakaway test：找到真正的啟動 torque 下界

### 為什麼需要

直接指定例如 `0.1 Nm` 做 torque sweep 可能完全無法跨過：

- static friction
- gearbox friction
- seal / bearing friction
- backlash dead zone
- gravity / load bias

如果 joint 不動，FRF 的 \(Q/\Tau\) 沒有辨識價值。

### 控制方式

使用 MIT torque-only：

```text
kp = 0
kd = 0
q = 0
dq = 0
tau = slowly increasing ramp
```

從靜止開始，以很慢的 torque ramp 增加：

\[
\tau_{cmd}(t)=r t
\]

直到偵測到可靠的運動，例如：

\[
|\dot q| > \dot q_{threshold}
\]

且連續維持 N 個 sample，避免 encoder noise 被誤判成啟動。

正負方向分別量：

\[
\tau_{break,+},\quad \tau_{break,-}
\]

### Stage 2 輸出

```text
breakaway_torque_positive_nm
breakaway_torque_negative_nm
breakaway_position_rad
breakaway_velocity_rad_s
breakaway_test.csv
```

### 對 Stage 4 的用途

Stage 4 excitation 必須至少具有足夠幅值跨過實際 deadband：

\[
A_\tau > \tau_{break}
\]

但不能只因為跨過 breakaway 就一路提高到 TMAX。

---

## Stage 3 — Torque capability envelope：找到掃頻的實際線性上界

### 為什麼 TMAX 不夠

`TMAX` 是 MIT command / feedback mapping 的最大範圍，不代表每一個轉速下都能在線性、可持續地輸出該 torque。

高速時受到：

- back-EMF
- bus voltage
- current limit
- PWM / voltage saturation
- thermal limits

影響，實際可用 torque 會隨速度下降。

因此要辨識：

\[
\tau_{max,linear}(\dot q)
\]

### 概念流程

在幾個代表性的 operating velocity 下：

1. 建立穩定工作點。
2. 逐步增加額外 torque demand / perturbation。
3. 比較 command torque、feedback torque、actual velocity。
4. 找到 response 開始明顯 saturation / nonlinear 的位置。
5. 保留安全 margin 作為 Stage 4 最大 excitation。

### 判斷 nonlinearity 可使用

- command torque 增加，但 feedback torque 不再同比增加
- velocity / acceleration response 不再同比增加
- response distortion 明顯增加
- current / temperature 接近限制
- amplitude-halving / doubling 的 FRF 不再一致

### Stage 3 輸出

```text
torque_linear_limit_vs_velocity.csv
```

可形成：

\[
\tau_{max,linear}=f(\dot q)
\]

這是 Stage 4 的 excitation 上界。

---

## Stage 4 — MIT Torque Frequency Sweep：補齊動態機械參數

### 被辨識 plant

核心 FRF：

\[
P(j\omega)=\frac{Q(j\omega)}{\Tau_{cmd}(j\omega)}
\]

也可同步分析：

\[
\frac{\dot Q(j\omega)}{\Tau_{cmd}(j\omega)}
\]

### 為什麼不是 POS_VEL sweep

目前 controller 的外層 PD 本身就是：

\[
\tau_{PD}=K_p(q_d-q)+K_d(\dot q_d-\dot q)
\]

因此為設計 \(K_p,K_d\) 而辨識的 plant 應該是 torque → joint motion。

POS_VEL 會再經過驅動器內部 position / velocity loops 與 trapezoidal motion profile，會把既有控制器包進被測系統中，不適合用來補純機械 plant 缺項。

### excitation amplitude

Stage 4 不再固定猜 `0.1 Nm`。

幅值由前兩個邊界限制：

\[
\tau_{break} < A_\tau < \tau_{max,linear}
\]

如果需要工作點 bias：

\[
\tau_{cmd}(t)=\tau_0+A_\tau\sin(\omega t)
\]

其中 \(\tau_0\) 用於 gravity / static load holding，\(A_\tau\) 才是 identification excitation。

### 掃頻形式

優先考慮 stepped-sine / sinestream：

```text
frequency f1
  settle cycles
  measure cycles
frequency f2
  settle cycles
  measure cycles
...
```

優點：

- 每個 frequency point 可獨立檢查 SNR
- 可以做 amplitude linearity check
- 可以在 resonance 附近增加頻率解析度
- 比一次快速 chirp 更容易判斷 transient / nonlinear / saturation

### 每筆 raw data 至少紀錄

```text
time_s
frequency_hz
command_torque_nm
feedback_torque_nm
position_rad
velocity_rad_s
tx_timestamp_ns
rx_timestamp_ns
t_mos_c
t_rotor_c
valid
```

### Stage 4 預期補出的參數 / 特徵

- \(J_{effective}\)
- \(K_g\)
- \(B_g\)
- resonance frequency \(\omega_r\)
- anti-resonance frequency
- modal damping ratio
- effective delay
- frequency-dependent gain / phase

對低頻剛性區域，可先檢查：

\[
P(s)\approx\frac{1}{J_{eff}s^2+B_{eff}s}
\]

若有 gearbox / link flexibility，改用 two-inertia model fitting。

---

# 5. 從 FRF 求 controller 的可用 \(\omega\)

最後不是把 torque→position FRF 的某個 `-3 dB` 直接叫做 controller omega。

對 candidate \(\omega_c\)：

\[
K_p(q)=M(q)\omega_c^2
\]

\[
K_d(q)=2\zeta\omega_cM(q)
\]

建立 controller：

\[
C(s)=K_p+K_ds
\]

配合 Stage 4 實測 plant \(P(j\omega)\)，檢查：

- loop gain crossover
- phase margin
- gain margin
- first flexible resonance
- noise amplification
- delay

最後選：

\[
\boxed{\omega_{usable}}
\]

而不是直接等於：

- I_BW
- V_BW
- PMAX / VMAX / TMAX
- POS_VEL 的 -3 dB bandwidth
- torque→position FRF 的任意 -3 dB 點

---

# 6. 模型整合方向

已有 motor electrical model：

\[
L\dot i+Ri+K_e\dot\theta_m=v
\]

mechanical model 可逐步補成：

\[
J_m\ddot\theta_m+B_m\dot\theta_m+\frac{\tau_g}{N}=K_ti
\]

\[
\delta=\frac{\theta_m}{N}-q
\]

\[
\tau_g=K_g\delta+B_g\dot\delta
\]

\[
M(q)\ddot q+C(q,\dot q)\dot q+G(q)+\tau_f(\dot q)=\tau_g
\]

其中 Stage 1~4 只補已知參數表缺失或不可信的機械項。

---

# 7. 硬體安全原則

1. 初次辨識最好讓單軸在安全姿態、有限行程、遠離碰撞區域。
2. Stage 2 torque ramp 必須有 position / velocity / temperature / timeout abort。
3. Stage 3 不應故意以 stall / locked-rotor 方式找極限，優先找線性可用範圍。
4. Stage 4 resonance 附近應允許自動降低 excitation amplitude。
5. TMAX 是 protocol mapping range，不直接視為可持續/線性 torque limit。

---

# 8. DAMIAO 硬體自動辨識：初步結論

DAMIAO RID 表沒有一個公開名稱叫 `MOTOR_TYPE` 的 register；官方 SDK 也要求建立 `Motor` 物件時先傳入 motor type。

但可以做 **best-effort auto detection**：

### 第一層：Firmware signature

讀：

```text
hw_ver RID 13
sw_ver RID 14
SN     RID 15
sub_ver RID 36
```

官方 SDK 將 RID 13~16 當成 `uint32` 回傳。`sw_ver` 實際是 4-byte version code。

目前 DM-J4310 dump：

```text
sw_ver = 925970485 = 0x37313035
```

把 uint32 還原成 little-endian 4 bytes：

```text
35 30 31 37 -> ASCII "5017"
```

DAMIAO 官方 firmware version 說明明確表示：

```text
APP_DM4310(V3)_V5017_04.bin
```

其中 `50` 是「V3 hardware 的 DM4310 motor series code」，`17` 是 firmware version。

因此 `sw_ver = "5017"` 本身就是非常強的 DM4310 識別依據。

### 第二層：protocol limit fingerprint

目前：

```text
PMAX = 12.5
VMAX = 30
TMAX = 10
```

官方 motor-sdk 的 DM4310 limit row 正是：

```text
[12.5, 30, 10]  # DM4310
```

但不能只靠這一層，因為未來型號或使用者修改 mapping parameters 時可能碰撞。

### 第三層：physical parameter fingerprint

目前：

```text
NPP = 14
Gr = 10
Rs ≈ 0.711 ohm
Ls ≈ 355 uH
Flux ≈ 4.345 mWb
```

可和型號 manual / expected calibration range 做交叉檢查。

### 建議 auto-detect policy

```text
1. decode sw_ver / hw_ver
2. match official firmware model-series table
3. cross-check PMAX/VMAX/TMAX
4. cross-check Gr/NPP/electrical params
5. if conflict -> UNKNOWN + require --motor-type override
```

不要在 signature conflict 時自動選型號。

---

# 9. 參考資料

## DAMIAO 官方 / 準官方資料

1. DAMIAO 驅動控制協議 V1.4（官方 GitHub 文件庫）  
   https://github.com/dmBots/damiao-document/blob/master/调试助手使用说明书（达妙驱动控制协议）V1.4.pdf

2. DAMIAO Motor SDK（官方）  
   https://github.com/dmBots/motor-sdk

3. DAMIAO Motor Firmware（官方）  
   https://github.com/dmBots/motor-firmware

4. DAMIAO 固件版本號說明  
   https://github.com/dmBots/motor-firmware/blob/master/版本说明/固件版本说明.md

5. DM-J4310-2EC V1.1 使用說明書  
   已於本專案對話中保存；規格包含 24 V、3 Nm rated、7 Nm peak、10:1 reduction、14 pole pairs、340 uH、650 mΩ。

## Frequency response / controller identification

6. MathWorks — Frequency Response Estimator  
   https://www.mathworks.com/help/slcontrol/ug/frequencyresponseestimator.html

7. MathWorks — Creating Input Signals for Frequency Response Estimation  
   https://www.mathworks.com/help/slcontrol/ug/creating-input-signals-for-estimation.html

8. Rockwell Automation — Motion System Tuning Application Technique  
   https://literature.rockwellautomation.com/idc/groups/literature/documents/at/motion-at005_-en-p.pdf

## Friction / breakaway identification reference

9. Friction / breakaway torque identification example（Wiley / International Journal of Advanced Robotic Systems mirror DOI）  
   https://onlinelibrary.wiley.com/doi/10.1155/2013/946526

---

# 10. 實作時不可忘記的決策

- 主辨識 plant：**MIT torque → joint position / velocity**。
- 不用 POS_VEL 取代 Stage 4。
- 不再假設 `0.1 Nm` 一定能讓軸動。
- Stage 2 先量 breakaway torque，給 excitation 下界。
- Stage 3 再量 torque linear capability，給 excitation 上界。
- Stage 4 的 excitation 必須落在實際可辨識的上下界之間。
- 不重新辨識已由驅動器提供且可信的 Rs/Ls/Flux/NPP/Gr 等項目。
- 最終目的不是得到一個模糊的「截止頻率」，而是得到 **可用 controller \(\omega\)** 與對應 \(K_p,K_d\)。

## `init_motors` AUTO semantics

`DamiaoCAN.init_motors()` keeps CAN IDs first and makes motor/control metadata optional:

```python
# all motors automatic
device.init_motors(send_ids, recv_ids)

# per-motor mix of explicit and automatic limits
device.init_motors(
    send_ids,
    recv_ids,
    [MotorType.DM4310, None, MotorType.DM8009],
)
```

For every `None` entry, initialization reads `PMAX`, `VMAX`, and `TMAX` from the motor and uses
those register values as the protocol scaling limits.  A known built-in family name is attached when
the limit fingerprint is recognizable; otherwise the motor type remains `UNKNOWN` while the
register-defined limits remain fully usable.

If complete positive finite `PMAX/VMAX/TMAX` values cannot be obtained and there is no known
motor-family fallback, initialization raises `MotorLimitResolutionError`.  It must never silently
substitute DM4310 or another arbitrary table entry.
