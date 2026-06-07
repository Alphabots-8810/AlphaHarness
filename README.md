# AlphaHarness 🅰️

**面向 FRC 的只读 NT4 遥测 + step-response 指标层,通过 MCP 暴露给 Claude。**

Alphabots `Alpha*` 系列第三件:**AlphaSim** 离线仿真射击;**AlphaHarness** 在线盯真车、量它的控制环怎么响应——给"会调参的 agent"打地基。

> **范围:** **(c)** 只读 metric 核心 + **(b)** 离线 `.wpilog` 臂 + **(a)** 受人工门控的 `set_gain` 写入。`set_gain` 只有在机器人固件 `tuningMode=true` **且**有人在 Test 模式 enable 时才生效(LLM 不能 enable 机器人;绝不在 FMS 上)。已对 8810 真实机器人代码在 sim 里验证(见 [对真机器人验证](#对真机器人验证))。

---

## 唯一的核心架构思想

**Claude 不从 AdvantageScope 读数据。** AdvantageScope 是个没有 API 的只读 viewer。它和 AlphaHarness 是*兄弟*:两个都连同一个 **NT4 server**(roboRIO)、读同一条流。AlphaHarness 这条只是**双向**的——NT4 是唯一可写的通道,这才让后续的闭环调参成为可能。

```
[robot code]              [NT4 = roboRIO/sim]          [AlphaHarness]        [Claude]
ShooterIO ---AdvantageKit--> /AdvantageKit/... (RO) --sub--> pyntcore client --MCP--> agent
 LoggedTunableNumber  <------ /Tuning/<key>  (RW)  <------------------------   set_gain
```

真正新的那块——也是所有现存 FRC NT/log 工具都缺的——是 **metric 层**:`capture_step_response` 返回约 10 个标量(rise、overshoot %、settle、steady-state error、damping ratio、peak current、saturation),而不是 50 Hz 波形,让 LLM 对着数字推理,而不是淹在样本里。

---

## 快速上手(不需要机器人)

```bash
cd ~/Projects/AlphaHarness
source .venv/bin/activate

# 1) 启动合成机器人 —— 一个发已知二阶响应的真 NT4 server
alphaharness-sim --once --zeta 0.5 --wn 18 --target 60 --warmup 3 --noise 0.3

# 2)(另开一个 shell)把 harness 指过去并测量
python -m tests.e2e_sim        # 打印实测指标 vs 闭式 ground truth
```

`alphaharness-sim` 发的是**和真车一模一样的拓扑**——稀疏的、on-change 的 setpoint 边沿(`/Tuning/SHooterRPS`)加稠密的 ~50 Hz 测量(`/AdvantageKit/RealOutputs/Shooter/MeasuredRPS`)——所以这里能通过的,走的就是生产环境的同一条接入路径。

### 离线臂(scope b —— 不需机器人、不需实时连接)

同一套 metric 层也能读赛后 `.wpilog` 文件(8810 已经在写):

```bash
alphaharness-simlog --zeta 0.45 --wn 20 --target 55 --out /tmp/shot.wpilog   # 合成一个带 ground truth 的 log
python -m pytest tests/test_wpilog.py -q                                      # 验证离线路径
```

然后问 Claude:*"分析 `/path/to/match.wpilog` 里 shooter 的 step。"*(MCP 工具 `list_wpilog_signals` / `analyze_wpilog_step`)。

## 接入 Claude Code

`.mcp.json` 已在本仓库里。在项目目录下,Claude Code 会自动挂上 `alphaharness` MCP server(用 `/mcp` 确认)。然后问 Claude:

> *"连 127.0.0.1,列出 shooter 的信号,抓一次 shooter 的 step response,告诉我 kD 是不是偏低。"*

工具(9 个):`connect` · `status` · `list_signals` · `read_signal` · `capture_step_response` · `set_gain` · `autotune_shooter` · `list_wpilog_signals` · `analyze_wpilog_step`。

## 指向真车 / WPILib sim

| 目标 | 命令 / 调用 |
|---|---|
| 合成机器人 | `connect(server="127.0.0.1")` |
| `./gradlew simulateJava` | `connect(server="127.0.0.1")` |
| 真 roboRIO | `connect(team=8810)` |

对真车:先起 capture,再由人来命令 step(比如在 AdvantageScope Tuning Mode 里设 `/Tuning/SHooterRPS`)。AlphaHarness 会看到稀疏边沿 + 稠密响应。**同一时刻只能有一个写 `/Tuning` 的人**——别让人和 harness 抢同一个 key。

---

## 落地到 8810 的代码(`~/Downloads/8810_work/2026_8810_main`)

**已经有的:** AdvantageKit logging(`LoggedRobot` + `NT4Publisher` + `WPILOGWriter`,所以遥测今天就在 NT 上)、带 `/Tuning` 前缀的 6328 版 `LoggedTunableNumber`、Phoenix6 闭环配置、drive 上的 `SysIdRoutine`。

**这个 v0 已经尊重的几个坑:**
- `ShooterIOSim` **没有物理模型** → `simulateJava` 不会产生真实 step 响应。所以用 `sim_robot.py` 作 ground-truth 基底。
- commanded setpoint **当前不是 logged output** → harness 从稀疏 `/Tuning` 边沿**推断** target(`_step_source="inferred_edge"`)。加一行无害的 `Logger.recordOutput("Shooter/SetpointRPS", …)` 就能让它直接读 target(sim 上的 `--dense-setpoint` 模拟了这点)。
- shooter(`VelocityTorqueCurrentFOC`)和 hood(`MotionMagicTorqueCurrentFOC`)是**力矩电流域**;电压域的 SysId 前馈不能直接套进去。给 scope (a) 种增益时相关——已标注,尚未接线。
- hood 是 **MotionMagic profile-following,不是 step** → `capture_step_response` 会拒绝 hood/MotionMagic 的 key,除非 `allow_profile=True`。

---

## 测试

```bash
python -m pytest tests/ -v          # 20 个单测:metrics + step-resolution + wpilog + autotune
python -m tests.e2e_sim             # 完整 NT4 线 vs 解析 ground truth(需 sim --once)
python -m tests.e2e_mcp             # MCP 传输层端到端(需 sim --period 4)
python -m tests.demo_autotune       # NT 线上实时自整定(需 alphaharness-plant 在跑)
```

测试同时跑**干净**和**带噪声/量化**的信号——只测干净曲线会在一个线上永不出现的虚构上通过。`e2e_sim` 从 NT 上(`/GroundTruth/*`)读 sim 自报的 ground truth,所以对比不会跟实际 sim 参数漂移。

### 验证了什么 —— 以及诚实的边界

- **基底是一个完美二阶系统。** 测试证明 metric 层能从二阶数据还原二阶参数(overshoot 与闭式差 ~1 个百分点,ζ 差 ~0.02)。它们对真 shooter **什么都没说**——真 shooter *不是*二阶(前馈主导、编码器测速量化、子弹扰动、电机非线性)。合成二阶 sim 在构造上永远抓不到这点。
- **真数据上信 model-free 指标:** `overshoot_pct`、`rise_time`、`settle_time_*`、`steady_state_error`、`peak_current`、`saturated`。
- **`damping_ratio` / `damped_freq_hz` / `natural_freq_hz` 是派生的**,不是测量——用二阶公式从 overshoot + peak-time 算出来。e2e 里 ζ 对上 ground truth 是 overshoot 的*重述*,**不是**第三个独立验证。把它们当启发式的形状描述,在非二阶 plant 上可能无物理意义。
- **`capture_step_response` 假设干净 step:** 机构接近 idle、窗口内恰好一次 setpoint 变更。一个已经在转的 shooter 或被连点两下的 tunable 会让 step 解析错位(取首个边沿;`y0` 从 step 前样本推)。

---

## 路线图

- **(c) — v0:** 只读 NT-MCP + metric 层。✅
- **(b) — 离线 WPILOG 臂:** 同一套 metric 层 + step-resolution,用 `wpiutil.log.DataLogReader` 指向赛后 `.wpilog`(8810 已经在写)。✅ 工具 `list_wpilog_signals` / `analyze_wpilog_step`;不需机器人、不需实时连接。
- **(a) — 闭环,harness 与机器人两侧都建好并演示过:**
  - harness 侧:`set_gain` 写 `/Tuning/*`(人工门控,见上)。✅
  - 机器人侧:`LoggedTunableNumber → ifChanged → getConfigurator().apply()` 再配置 shim + `tuningMode` flag,在 8810 的 `AlphaHarness` git 分支上。✅(见下)
  - 演示过:AlphaHarness 写 `/Tuning/Shooter/kP`,真机器人**消费了它**(AdvantageKit mirror `/AdvantageKit/NetworkInputs/Tuning/Shooter/kP` 更新)。✅
  - **自主 optimize loop**(`autotune_shooter`):perturb → measure → score → `set_gain` → 循环,在 (kP, kD) 上做坐标模式搜索。✅ 在 NT 线上对闭环飞轮实时演示——把迟钝的 kP=3(SSE 14%)调到 kP=13/kD=0.2(0% overshoot,5% SSE),cost −57%,18 次 NT 评估。(`python -m tests.demo_autotune`)
  - **还没做的:** Phoenix6 `apply()` 本身只在 REAL 模式跑(sim 用 `ShooterIOSim`,没有 TalonFX),而且 autotuner 是在合成飞轮 / plant 模型上验证的——不是真 shooter。任何真硬件闭环前先走 Maple-Sim;replay 不能调 feedback gain(它会改变后续输入)。

## 自整定器

```bash
alphaharness-plant            # 一个闭环飞轮 NT server,响应随增益变化
python -m tests.demo_autotune # AlphaHarness 在 NT 线上实时调它
```

`autotune.py` 是无导数的坐标模式搜索,最小化 `cost = rise + 0.012·overshoot² + 0.05·|SSE%|`(用恒有定义的指标,所以即便 settle-to-band 无定义、曲面也保持平滑)。evaluator 可插拔:**同一个**优化器既能在进程内对 `plant.py` 跑(快测),也能经 `set_gain` + `command_step_and_capture` 在 NT 上跑(实时)。有界增益 + 评估预算把它挡在不稳定区之外;真硬件上受人工门控(tuningMode + Test 模式 enable,绝不 FMS)。

## 对真机器人验证

除了合成基底,AlphaHarness 还对 **8810 的真实机器人代码**(`~/Downloads/8810_work/2026_8810_main`,分支 `AlphaHarness`)在 `./gradlew simulateJava` 里验证过:

- **发现:** 连上后看到真实的 **367 个 topic** 的 AdvantageKit 树,包括 scope-a shim 的 `/AdvantageKit/RealOutputs/Shooter/SetpointRPS` + `/Tuning/Shooter/{kP..kV}`。(`python -m tests.probe_real_tree`)
- **写→消费:** `set_gain("/Tuning/Shooter/kP", 9.0)` → 机器人读到 → AdvantageKit 在 mirror 处记录了 `9.0`。(`python -m tests.probe_write_loop`)

机器人侧 shim 是 4 处小改(Constants 的 `tuningMode`、IO 的 `setShooterPID` hook、其 Phoenix6 实现、以及 `ShooterSubsystem.periodic` 里受门控的 `ifChanged` 再配置),`tuningMode=false` 时全是死代码。看 diff:`git -C ~/Downloads/8810_work/2026_8810_main show AlphaHarness`。

## 接到机器人(scope a 集成指南)

要让 AlphaHarness 在你的机器人上真正写增益、自整定,机器人侧需要一个受 `tuningMode` 守卫的 shim。下面以 8810 季后赛新车(`8810-2026-offseason`,AdvantageKit + 通用 `frc.lib.io.MotorIO` 架构、`Drum` 是飞轮)为例;任何 AdvantageKit + Phoenix6 机器人同理。

### 读取路径通常大部分已就位

- **AdvantageKit + NT4Publisher** → 所有 `@AutoLog` input 和 `Logger.recordOutput` 已实时在 NT 上,AlphaHarness 直接 `connect` 就能发现。
- setpoint 若已 log(如新车的 `Logger.recordOutput("Shooter/Drum/GoalRps", …)`)就是 AlphaHarness 要的稠密 setpoint 输出。
- 测速若经 IOInputs log(如 `MotorIOInputs.velocityRadPerSec`)也已在 NT 上。

> ⚠️ **单位坑**:`GoalRps` 是 **RPS**、`velocityRadPerSec` 是 **rad/s**。把 AlphaHarness 指向统一单位——最省事加一句 `Logger.recordOutput("Shooter/Drum/MeasuredRps", Units.radiansToRotations(inputs.velocityRadPerSec))`。单位混了指标全是垃圾。

### 要加的 3 块(`tuningMode=false` 时全是死代码)

**1. 全局 `tuningMode` flag**
```java
// Constants.java(或一个 TuningConstants)
public static final boolean tuningMode = false; // 比赛安全默认值
```

**2. 可调增益通道** —— 从 `2026_8810_main` 的 `frc/robot/util/` 港 `LoggedTunableNumber`(MIT,FRC 6328;包 `LoggedNetworkNumber` + 加 `/Tuning/` 前缀):
```java
private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Drum/kP", /* ShooterConstants 里的 drum kP */);
private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Drum/kD", /* ShooterConstants 里的 drum kD */);
// 按需再加 kI、kS、kV
```
→ topic `/Tuning/Shooter/Drum/kP` …,AlphaHarness 读写它们。

**3. 再配置 hook**(缺的那块 —— `withSlot0` 只在 boot 时 apply 一次)—— 在 IO 接口加 `setPID`,在 Phoenix6 实现里 re-apply Slot0,在 periodic 里受门控调(加在 `MotorIO`/`MotorSubsystem` 基类则所有机构白拿):
```java
// frc/lib/io/MotorIO.java
public default void setPID(double kP, double kI, double kD, double kS, double kV) {}

// frc/lib/io/MotorIOPhoenix6.java
@Override
public void setPID(double kP, double kI, double kD, double kS, double kV) {
  var slot0 = new Slot0Configs();
  slot0.kP = kP; slot0.kI = kI; slot0.kD = kD; slot0.kS = kS; slot0.kV = kV;
  tryUntilOk(5, () -> talon.getConfigurator().apply(slot0, 0.25)); // 不动 limits / MotionMagic
}

// Drum.periodic()(或 MotorSubsystem 基类,可复用)
if (Constants.tuningMode) {
  LoggedTunableNumber.ifChanged(hashCode(),
      v -> io.setPID(v[0], v[1], v[2], v[3], v[4]), kP, kI, kD, kS, kV);
}
```
**`ifChanged` 是承重墙**:`getConfigurator().apply()` 阻塞(~0.1–0.25 s),每 20 ms loop 都调会打爆 loop 预算——只在变了时 apply。

### 把 AlphaHarness 指过去
```
measurement_key = "Shooter/Drum/MeasuredRps"            # 加这个输出(见单位坑)
setpoint_key    = "/Tuning/Shooter/Drum/Setpoint"        # tuningMode 下 Drum 读的 tunable(自主用)
gains           = /Tuning/Shooter/Drum/{kP,kD,...}
```
- **自主**(AlphaHarness 经 `autotune_shooter` 自己命令 step):把 Drum setpoint 暴露成 `/Tuning/...` tunable,`tuningMode` 开时 Drum 读它。
- **辅助**(人手 spin up、AlphaHarness 只测量):不需写 setpoint —— 用 `capture_step_response`,机器人侧零改动。

### 域注记(有个 2026 代码没有的选择)
`MotorIOPhoenix6` 同时支持 `VelocityVoltage` 和 `VelocityTorqueCurrentFOC`。跑 **VOLTAGE** 模式,WPILib **SysId** 前馈(kS/kV,电压域)能直接作种子塞进去;跑 **TORQUE_CURRENT_FOC** 则电压域前馈不做转换塞不进——kS/kV 得经验调(AlphaHarness 能调)。

### Checklist
- [ ] 加 `MeasuredRps` 输出(单位修正)
- [ ] `tuningMode` flag
- [ ] 港 `LoggedTunableNumber` + 加 Drum 增益 tunable
- [ ] `MotorIO.setPID` + `MotorIOPhoenix6` 实现 + periodic 受门控 `ifChanged`
- [ ] (自主)Drum 在 tuningMode 下读一个 `/Tuning` setpoint
- [ ] Sim:`connect` → `autotune_shooter` 收敛
- [ ] 真车:人工 enable、Test 模式、限位设好后调参

---

## 安全(从 scope a 起重要)

`isFMSAttached()` → 拒绝一切调参。真车**永远人工 enable、agent 只建议**(LLM 不能 enable 机器人)。完全自主只活在 sim 里。软限位 + stator 限流 + motor-safety 心跳设在 IO 里、不在调参逻辑里,所以即使某个 loop 卡死它们也还在。

---

> Alphabots 8810 · `Alpha*` 系列:[AlphaSim](https://github.com/Alphabots-8810/AlphaSim)(离线弹道) · AlphaHarness(在线遥测 + 自整定)
