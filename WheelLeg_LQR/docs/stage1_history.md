# 第一阶段历史说明（当前入口与功能请看工程根目录 README.md）

第一阶段已实现 Step 1～9：四状态模型、离散 LQR、Python 基准验证、C 控制器、MinGW DLL、ctypes 闭环、曲线、动画和扰动。现在新增五连杆正/逆运动学、Jacobian、双侧五连杆二维/三维展示；完整五连杆动力学与关节电机控制尚未实现。

## 新启动方式：Ctrl+Shift+B → main.py

VS Code 打开外层 `LQR` 或内层 `WheelLeg_LQR` 文件夹均可，两个目录均配置了默认编译任务：

1. **Ctrl+Shift+B** 编译 C DLL（执行 `mingw32-make all`；源码未变化时会提示无需重编译）。
2. 在终端执行 **`python main.py`**：使用全局 Python 启动 C 闭环，打开五条控制曲线和同步的二维/三维五连杆动画。
3. **F5** 也可先编译再运行 `main.py`；三维窗口支持鼠标旋转查看左右腿。

```powershell
python main.py
python main.py --impulse 0.5 --impulse-time 3
python main.py --no-show                    # 不打开窗口，生成三张 PNG 和仿真数据
python main.py --leg-demo                   # 独立五连杆伸缩运动学演示
python main.py --leg-demo --duration 3 --gif leg_demo.gif
```

默认图片在工程 `outputs/` 中：`five_bar_balance.png`（曲线）、`five_bar_balance_2d.png`（二维）、`five_bar_balance_3d.png`（三维）。二维/三维静态图片显示初始姿态，动画展示全过程。外层 `main.py` 是转发入口，实际启动逻辑在内层 `WheelLeg_LQR/main.py`。

### 五连杆结构与当前建模范围

每侧闭链为 **A–B–C–D–E–A**：A/E 是机身上的主动髋关节，B/D 是被动膝关节，C 是轮轴；第五根杆 E–A 固定在机身上。三维中两侧各有一套机构，轮距 0.36 m。`config/leg_params.py` 中给出示例尺寸：AB/ED=0.13 m、BC/DC=0.20 m、AE=0.12 m，名义轮轴至髋中点高度 0.20 m。它是 RoboMaster 风格的机构拓扑示意，尺寸不代表某支战队的 CAD。

`simulation/five_bar.py` 使用圆交点计算正运动学，两个二连杆子链计算逆运动学，显式选择下方轮轴、向外膝关节的装配支路；不可达或奇异构型抛出异常。约束微分得到 `foot_velocity = J @ joint_velocity`，Jacobian 与有限差分结果核对。77 组足端位置已检查正逆闭合、全部五根杆长和 Jacobian。

```powershell
python -m simulation.five_bar
```

**默认平衡运行仍为四状态 C LQR + 等效倒立摆物理对象，五连杆保持固定关节姿态随车体运动。** 画图使用计算得到的真实闭链几何，但尚未模拟各杆质量、关节转矩、VMC 或变化腿长对 A/B 的影响。`--leg-demo` 独立展示 0.165～0.235 m 的腿伸缩，不运行平衡控制，不把几何动画冒充完整轮腿动力学。名义平衡姿态下所画机体质心与原物理对象的 0.25 m 质心高度一致。

本机使用**全局环境**，没有创建 venv/conda：Windows + `D:\Python310\python.exe`（Python 3.10）、`D:\Environment\mingw64\bin\gcc.exe`（GCC 15.2）。已验证 NumPy 2.2.6、SciPy 1.15.3、Matplotlib 3.10.8。

## 快速运行

在 PowerShell 中进入工程根目录（所有 `python -m` 命令都在此目录执行）：

```powershell
cd C:\Users\ydkfb032\Desktop\LQR\WheelLeg_LQR
python -m simulation.design_lqr
mingw32-make verify
python -m simulation.simulator --plot --animate
```

已有全局依赖无需安装；其他机器缺少依赖时执行 `python -m pip install -r requirements.txt`。需要 Python、GCC、mingw32-make 在 PATH 中，且 Python 和 GCC 的位数一致。本机已实际编译并加载 DLL。

使用 VS Code 打开 **LQR 或 WheelLeg_LQR 文件夹**。`Ctrl+Shift+B` 编译，运行任务 `LQR: verify` 验证；安装 Microsoft Python/Python Debugger 扩展后可 F5 启动仿真。`.vscode/settings.json` 已指定本机全局 Python/GCC 路径，换机器需修改。`main.c` 是独立 C 校验程序，不是 Python 调用入口；DLL 导出的是 `lqr.c` 的函数。

## 工程布局和边界

```text
WheelLeg_LQR/
├─ controller/
│  ├─ main.c                 # 独立 C 校验，示意 MCU 周期调用
│  ├─ lqr.c / lqr.h          # float、结构体、函数、无动态内存
│  ├─ lqr_gains.h            # Python 离线生成的 C 参数
│  └─ controller.dll         # GCC 编译结果
├─ simulation/
│  ├─ model.py               # 连续动力学、RK4、冲量响应
│  ├─ five_bar.py            # 五连杆正/逆运动学与 Jacobian
│  ├─ robot_view.py          # 双侧五连杆二维/三维视图
│  ├─ design_lqr.py          # 离线 ZOH + 手写 Riccati + SciPy 对照
│  ├─ design_data.py         # 读取设计参数和模型一致性检查
│  ├─ python_reference.py    # 仅 Step 3 与测试使用
│  ├─ c_controller.py        # ctypes ABI，在线不计算 K 或控制量
│  ├─ simulator.py           # 固定步长、传感器、扰动、日志
│  ├─ visualization.py       # 五条曲线和二维动画
│  └─ verify.py              # 数学、C ABI、闭环集成验证
├─ config/
│  ├─ robot_params.py        # SI 单位物理参数、Ts、限幅、Q/R 默认值
│  ├─ leg_params.py          # 五连杆几何尺寸
│  └─ lqr_design.json        # A/B/Ad/Bd/Q/R/P/K、特征值和设计标识
├─ outputs/                  # 自动生成 CSV、NPZ、PNG、GIF、验证报告
├─ .vscode/
├─ Makefile
├─ main.py                   # 默认 Python 启动入口
├─ requirements.txt
└─ README.md
```

在线数据流：

```text
Python Physics -- sensor state [p,v,theta,omega] --> C LQR_Update
       ^                                                |
       |________ saturated horizontal force [N] ________|
```

每个 tick 顺序：处理外部冲量 → 真值加传感器噪声 → ctypes 传入四状态与参考状态 → C 计算并限幅 → Python 在该输入保持不变的条件下做一次 RK4。传感器默认理想，可添加固定种子的高斯噪声。NPZ/CSV 保留真值、测量值和输入；最终时刻没有新的控制指令，因此 CSV 最后一行输入/测量为 NaN。

`Ts=0.001 s` 是严格的**仿真时间步长**，不是 Windows 实时调度保证。一次 10 秒仿真执行 10000 次 C 控制调用，实际墙钟运行可能快于或慢于 10 秒。动画约 30 fps 回放已计算的轨迹，不影响控制周期。C 是无内部状态的静态反馈，MCU 移植时由 1 kHz 定时器负责调用。

## Step 1：模型和符号

本版是**水平力驱动的轮轴等效底座倒立摆**。左右轮在侧视图中重合，腿长固定。不是完整的 RoboMaster 轮腿刚体模型；输入 `u` 是总水平驱动力 N，尚未包括轮电机对机体的反作用力矩、轮转动惯量、轮胎滑动、左右差速或腿部自由度。`wheel_radius` 目前用于画图，不能直接把输出当作实机电机转矩。

状态：

$$x=[p,\dot p,\theta,\dot\theta]^T.$$

`p` 向右为正，`theta=0` 直立、前倾为正，内部角度单位 rad；`u>0` 向右。质量 `M=1 kg` 为等效底座，机体 `m=5 kg`，轮轴至机体质心 `l=0.25 m`，机体质心转动惯量 `I=0.10 kg m²`，底座黏性阻尼 `b=0.10 N s/m`。

机体质心为 `(p+l sin(theta), l cos(theta))`（忽略不影响动力学的轮轴高度常数）。由动能、势能：

$$T=\tfrac12(M+m)\dot p^2+ml\cos\theta\,\dot p\dot\theta+\tfrac12(I+ml^2)\dot\theta^2,\quad V=mgl\cos\theta$$

得到可选的同结构非线性对象：

$$
\begin{bmatrix}M+m&ml\cos\theta\\ml\cos\theta&I+ml^2\end{bmatrix}
\begin{bmatrix}\ddot p\\\ddot\theta\end{bmatrix}
=\begin{bmatrix}u-b\dot p+ml\sin\theta\dot\theta^2\\mgl\sin\theta\end{bmatrix}.
$$

在直立处取 `sin(theta)≈theta`、`cos(theta)≈1`，忽略二阶及以上项。令 `J=I+ml²`、`D=(M+m)J-(ml)²`：

$$
A=\begin{bmatrix}
0&1&0&0\\
0&-bJ/D&-m^2gl^2/D&0\\
0&0&0&1\\
0&bml/D&(M+m)mgl/D&0
\end{bmatrix},\quad
B=\begin{bmatrix}0\\J/D\\0\\-ml/D\end{bmatrix},\quad\dot x=Ax+Bu.
$$

数值结果：

```text
A = [[0,  1,             0,            0],
     [0, -0.045205479, -16.797945205,   0],
     [0,  0,             0,            1],
     [0,  0.136986301,  80.630136986,   0]]
B = [[0], [0.452054795], [0], [-1.369863014]]
```

前倾时要向前驱动底座接住质心：正输入使 `theta_ddot` 减小，所以 `B[3]<0`。不要直接套用其他角度定义下的 K。开环有约 `+8.965 s^-1` 的不稳定极点。

验证命令：

```powershell
python -m simulation.model
```

检查平衡点、质量矩阵正定、非线性数值 Jacobian 与 A/B 一致、可控矩阵秩为 4。

## Step 2：离散 LQR 数学过程

零阶保持离散化：

$$A_d=e^{AT_s},\qquad B_d=\int_0^{T_s}e^{A\tau}B\,d\tau.$$

代码对增广矩阵求指数，直接读出 `Ad`、`Bd`，不使用要求 `A` 可逆的表达式：

$$\exp\left(T_s\begin{bmatrix}A&B\\0&0\end{bmatrix}\right)=\begin{bmatrix}A_d&B_d\\0&1\end{bmatrix}.$$

离散性能指标与默认权重：

$$J_{LQR}=\sum_{k=0}^{\infty}(x_k^TQx_k+u_k^TRu_k),\quad Q=\operatorname{diag}(20,2,300,5),\quad R=[0.1].$$

这里 Q、R 是直接选择的离散权重，不声称是连续成本精确离散化。各状态单位不同，权重数值不能脱离量纲直接比较。

从 `P0=Q` 开始手写迭代：

$$P_{i+1}=Q+A_d^TP_iA_d-A_d^TP_iB_d(R+B_d^TP_iB_d)^{-1}B_d^TP_iA_d.$$

相对 Frobenius 范数变化小于 `1e-12` 时停止；最多 200000 次，失败明确报错。矩阵保持数值对称，代码使用 `solve` 而非显式求逆。默认迭代 11748 次，DARE 相对残差约 `9.97e-13`，另与 SciPy `solve_discrete_are` 对照。

$$K=(R+B_d^TPB_d)^{-1}B_d^TPA_d,\qquad u=-K(x-x_{ref}).$$

默认结果：

```text
K = [-13.974559775, -18.884913861, -175.859179140, -23.514620989]
eig(Ad-Bd*K) = 0.986098237
               0.992108897
               0.999018190 +/- 0.000864821j
```

全部模长小于 1。该结论适用于线性、无饱和、精确状态反馈；非线性、传感器噪声、限幅用时域实验单独检验。命令会明确输出 **A、B、Ad、Bd、Q、R、P、K、闭环特征值** 并导出 JSON 与 C 头文件：

```powershell
python -m simulation.design_lqr
```

## Step 3：Python 基准

```powershell
python -m simulation.simulator --backend python --output outputs/python_baseline --plot
```

这是唯一在线使用 Python LQR 的入口，仅作阶段验证和 C 对照。默认 `--backend c` 不导入此控制器或设计求解器，DLL 缺失或不匹配时直接报错，不会悄悄回退 Python。

## Step 4～7：C、DLL、联合仿真

`RobotState` 四个 float；`LQRController` 包含 `K[4]` 和 `output_limit`。`LQR_Update` 在 C 内处理状态误差、点积和 `±40 N` 限幅。没有 malloc/new 或 Python 回调。NULL、非有限控制结果、无效限幅返回 0，这是演示版无效输入策略，尚未设计 MCU 故障管理。

```powershell
mingw32-make test-c
python -m simulation.simulator --initial-pitch 10 --output outputs/c_baseline --plot
python -m simulation.verify
```

不使用 Make 也可直接编译：

```powershell
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -shared -DLQR_BUILD_DLL controller/lqr.c -o controller/controller.dll
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic controller/main.c controller/lqr.c -o controller/controller_test.exe
.\controller\controller_test.exe
```

ctypes 明确声明结构体、指针、参数/返回类型，使用绝对 DLL 路径，并检查 C 结构体大小、周期和离线设计 ID。**修改模型/Ts/限幅后必须重新设计、编译；修改 Q/R 后也必须重新设计、编译。** 不一致会阻止仿真。Windows 已加载的 DLL 可能被进程锁住；重编译前关闭之前的仿真/动画窗口及其 Python 进程。

## Step 8：曲线、二维动画

本节保留第一阶段的倒立摆验证视图。新的双侧五连杆二维/三维展示请使用 `python main.py`。

```powershell
python -m simulation.simulator --plot --animate
# 只重放现有数据，无需重跑仿真
python -m simulation.visualization outputs/c_baseline.npz --animate
# 导出 GIF，无需 ffmpeg；导出速度取决于机器
python -m simulation.visualization outputs/disturbance.npz --gif outputs/disturbance.gif --no-show
```

图中依次显示位置、速度、俯仰角、角速度、控制力；角度显示为度，内部始终是 rad。二维侧视图显示轮、机体和质心，机体示意长度取 `2l`，不表示五连杆几何。PNG 与输入 NPZ 同名保存。

## Step 9：扰动、限幅、噪声与 Q/R

瞬时外力采用**冲量** `j=∫F dt`，单位 N·s，在轮轴高度施加，避免把人为角速度跳变误称为外力。线性模型用直立质量矩阵，非线性模型用当前角度的质量矩阵：

$$\begin{bmatrix}\Delta\dot p\\\Delta\dot\theta\end{bmatrix}=H(\theta)^{-1}\begin{bmatrix}j\\0\end{bmatrix}.$$

位置与角度连续，速度跳变。冲量在指定 tick 的传感器采样之前施加；时间必须落在 1 ms 网格上。默认试验 `t=3 s, j=0.5 N·s`，不是 `0.5 N` 持续力。

```powershell
python -m simulation.simulator --impulse 0.5 --impulse-time 3 --output outputs/disturbance --plot --animate
python -m simulation.simulator --nonlinear --impulse 0.5 --output outputs/nonlinear_disturbance --plot
python -m simulation.simulator --initial-pitch 15 --output outputs/saturation --plot
python -m simulation.simulator --sensor-std 0.0001 0.001 0.0001 0.001 --seed 42 --output outputs/sensor_noise --plot
python -m simulation.simulator --reference-pos 0.2 --output outputs/position_reference --plot
```

参考输入演示仅提供常值位置目标，其他三个参考状态为零。改变它们需考虑目标是否为物理可行平衡/轨迹；这里没有轨迹前馈或状态观测器。绝对俯仰角超过 45° 时仿真明确停止，避免把明显失去平衡的结果当作成功。

修改权重的完整工作流：

```powershell
# 更保守的驱动力代价
python -m simulation.design_lqr --q 20 2 300 5 --r 1.0
mingw32-make all
python -m simulation.simulator --output outputs/r_1 --plot

# 更高的俯仰角代价（示例，可自行实验）
python -m simulation.design_lqr --q 20 2 600 5 --r 0.1
mingw32-make all
python -m simulation.simulator --output outputs/q_pitch_600 --plot

# 恢复 robot_params.py 的默认 Q/R
python -m simulation.design_lqr
mingw32-make verify
```

也可以直接修改 `config/robot_params.py` 的 `Q_DIAG`/`R_VALUE`。命令行指定的 Q/R 保存在 JSON/头文件中，不会改写 Python 默认配置。`mingw32-make design` 总是恢复/使用 Python 配置中的权重。

## 已执行验证结果

默认 10°、10 秒、1 ms：

| 场景 | 峰值驱动力 | 最大位置偏移 | 10 s 俯仰角 |
|---|---:|---:|---:|
| C + 线性对象，R=0.1 | 30.693 N | 0.172533 m | 0.000247° |
| C + 非线性对象，R=0.1 | 30.693 N | 0.176409 m | 0.000252° |
| C + 线性对象 + 3 s 冲量 | 30.693 N | 0.172533 m | 0.001091° |
| C + 线性对象，R=1.0 | 24.089 N | 0.256735 m | -0.006876° |

交付时已恢复默认 R=0.1。C/Python 默认轨迹最大状态分量差约 `3.02e-8`（各分量单位分别为 m、m/s、rad、rad/s）。15° 工况确实达到 `40 N` 限幅后恢复。

`mingw32-make verify` 包含：C 自测、有限差分线性化、可控性、Riccati/SciPy 对照、RK4/精确 ZOH 单步对照、C ABI 与参数、1000 组随机状态/参考的 C/Python 输出对比、整段轨迹对比，以及非线性/扰动/负倾角/限幅/位置参考/噪声恢复测试。默认恢复标准要求**最后整整一秒**：位置误差 <5 mm、速度 <0.01 m/s、倾角 <0.1°、角速度 <0.2°/s，避免只检查最后一个采样点。改变权重后若未在 10 秒内满足门限，测试会失败，不代表所有这类增益都不稳定。

`outputs/verification.json` 保存默认验证指标，CSV/NPZ 为实验数据。RK4、传感器与场景均可复现；这不是实机性能保证，物理参数目前是合理示例值。

## Step 10 及后续路线

1. 先建立含轮电机反作用力矩、轮惯量的转矩输入模型，重新推导 A/B，明确总力矩/单轮力矩与正方向。
2. 将固定质心高度参数化为腿长，明确机体惯量和质心位置如何变化。
3. 单侧二维五连杆正/逆运动学及二维/三维展示已实现；后续加入关节限位、自碰撞和完整闭链动力学。
4. Jacobian 已实现；VMC、虚功验证以及虚拟腿力到关节力矩映射待实现。
5. 离线生成多个腿长下的 K(l)，在 C 中查表插值，并逐点验证插值闭环；稳定端点不自动保证所有插值点稳定。
6. 再加入左右腿状态、状态观测器/Kalman 和腿长变化动力学。

保持当前 `Python Physics → C Controller → Python Physics` 边界，届时显式升级状态结构和 ABI，不在当前四状态模型中塞入没有动力学的占位状态。

## 数学参考

- [University of Michigan CTMS：倒立摆建模](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)。本工程自行规定前倾正方向，使用质心坐标重新推导，符号可能与教材不同。
- [SciPy：离散代数 Riccati 方程](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html)。仅用于验证手写迭代。
- [SciPy：矩阵指数 expm](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.expm.html)。用于增广矩阵 ZOH 离散化。
- [RM2024 五连杆轮足机器人建模项目](https://github.com/WilliamGwok/RP_Balance)：机构背景参考，本工程运动学实现独立编写。
