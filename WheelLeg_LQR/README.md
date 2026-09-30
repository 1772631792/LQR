# WheelLeg 仿真上位机：C 底盘算法 + Python 物理对象

`main.py` 默认打开桌面上位机的 **MuJoCo 实景** 页。使用全局 Python + MinGW GCC，不使用 ROS/Gazebo。

**参考固件模式：C 负责在线 LQR / PID / VMC，以及离线 Riccati 求解；Python 负责物理对象、离线建模、离散化、拟合和验证。** `balance_chassis-main` 原文件保留，原始五连杆与增益控制函数直接编入 `firmware.dll`。前七个页面继续提供原来的简化模型教学实验，使用独立的 `controller.dll`，不与 MuJoCo 的数据混用。

## 参考固件 + MuJoCo（新增）

- **进入操控**：W/S 前后、A/D 转向、C 高低腿、G 快慢、Shift 越阶收放腿、Ctrl+C 零力、R 重置、Esc 停止。鼠标右键拖动第三人称相机，滚轮缩放；“全图 / 跟随”查看场地。仅画面获得焦点时响应键盘，失焦释放按键。
- **可调腿部机械限位**：MuJoCo 实景页可直接设置允许的最短/最长有效腿长（0.14–0.33 m）。程序据此为四个髋关节和四个被动膝关节生成左右对称的角度限位，避免受冲击后五连杆反折；默认 0.15–0.30 m。防翻车辅助默认开启，会在大倾角时降低驱动，并随车速抑制过急转向。
- **五连杆 / 串联腿共用设置**：主窗口右上角“⚙ 设置”可选择原五连杆闭链或二关节串联腿，配置大腿、小腿、轮距、有效腿长及力矩限幅。配置同时送入 MuJoCo 和生产代码导出；串联腿输入为每侧髋角/膝角，C 核心通过二连杆正运动学与雅可比 VMC 输出四路关节力矩。详见 `docs/serial_leg_control.md`。
- **浅色工作台界面**：主功能使用左侧分区导航；MuJoCo 页面默认只保留单行操控栏，让物理画面占据主要空间，分辨率、子步数与机械限位收进可折叠的“调整参数”。快捷键：`F5` 运行、`Esc` 停止、`Ctrl+E` 导出、`Ctrl+,` 打开设置。
- **两套独立生产代码**：“生产代码导出”可分别生成 `five_bar_control` 和 `serial_leg_control`。每个包只携带自己的运动学文件，没有运行时腿型判断，也不引用另一套腿型代码；生命周期、外部 PID、LQR、保护和测试在各包内完整存在。
- 地图有 **32 cm 飞坡平台**和另一车道的 **4 cm 单级台阶**。出生点可选飞坡、台阶、平地。双轮、四个髋电机、四个被动膝关节和两组闭链约束实际参与 MuJoCo 动力学。
- **预计算直行**：填写 0.1–120 秒，后台按高腿、0.7 m/s 计算该车道。完成后“播放记录”，支持循环。操控记录也可停止后回放，并导出 CSV、NPZ、曲线图到 `outputs/mujoco/`。实时记录单次最多 120 秒。
- 控制周期固定 **1 ms**；物理子步数可选 1/2/4，对应积分间隔 1/0.5/0.25 ms。渲染分辨率可选 800×450、960×540、1280×720。画面约 30 FPS，与 1 kHz 控制解耦；慢机器可降低画质或先算再播，不跳过物理步。
- **替代 MATLAB / 六状态矩阵**：修改 Q/R，点击 C 重算 30 个腿长节点，导出 `outputs/source_design/design.json` 和 `regenerated_gains.h`。可以复制单个矩阵，并勾选下一次使用重算参数。默认仍使用原固件参数。

已复现参考 `matlab.zip/GenerateGains.m` 的六状态建模、连续代价精确离散化、C DARE、五步无约束 MPC 静态增益及三次多项式拟合。默认参数下，与原始 LQR / MPC 系数表最大绝对差分别约 `2.39e-4` / `1.07e-5`；30 个节点以及工作腿长范围的拟合/混合增益均通过线性稳定性检查。这是替代该项目的增益设计流程，并非通用 MATLAB/Simulink 实现。

本机实测：纯物理+C 约 14–15 倍实时；800×450 渲染一起计算约 1.89 倍实时。GUI 已验证键盘输入、实时推进、预计算、循环回放、导出、剪贴板和重算参数接入。性能随机器、画质和子步数变化。详细映射、数学过程与物理假设见 [参考固件移植说明](docs/firmware_port.md)。

```powershell
mingw32-make design-source     # 完全不需要 MATLAB
mingw32-make verify-mujoco     # 接触、越障、渲染与真实 Tk 界面检查
```

## 启动

1. VS Code 打开 `LQR` 或 `WheelLeg_LQR` 文件夹。
2. **Ctrl+Shift+B** 编译 C DLL。
3. **`python main.py`** 打开上位机。F5 也会先编译再启动。
4. 在“MuJoCo 实景”点击进入操控，或预计算后播放。需要旧简化模型实验时，在“仿真设置”填写参数后使用顶部计算按钮。

```powershell
cd C:\Users\ydkfb032\Desktop\LQR\WheelLeg_LQR
mingw32-make all
python main.py
```

本机全局解释器 `D:\Python310\python.exe`，GCC `D:\Environment\mingw64\bin\gcc.exe`。Tkinter 8.6 已验证。依赖安装命令：`python -m pip install -r requirements.txt`，现有环境无需重复安装。

## 上位机选项卡

| 选项卡 | 内容 |
|---|---|
| 仿真设置 | 时长、控制周期、积分步长、初始状态、目标位置/腿长/航向、扰动、Q/R、PID 和力矩限幅 |
| C算法与增益 | C 返回的 A/B、Ad/Bd、P、K、迭代次数、三组腿长节点及本次参数 |
| 平衡曲线 | 位置、速度、俯仰、横滚、航向、驱动力；参考线和播放时间游标 |
| 腿长与电机 | 左右腿长、支撑力、左右轮电机转矩、四个髋关节转矩 |
| 二维机构 | 随设置切换为左/右五连杆或二连杆串联腿，随实际腿长和俯仰变化 |
| 三维底盘 | 双轮、机身和两套当前腿型，展示腿长、横滚、航向，支持鼠标旋转 |
| 参数导览 | 随腿型切换机构、关节名称和说明；11 组引出标注支持悬停、固定和曲线定位 |
| MuJoCo 实景 | 参考固件控制、真实地图、第三人称操控、预计算/回放、独立记录曲线、六状态增益重算 |
| 生产代码导出 | 生成 Simulink Coder 风格的 `U/Y/B/DW/P + initialize/step/terminate` C 模型；通过不透明实例指针和函数接口绑定工程自有 PID，并执行依赖扫描、独立编译和自测 |

底部支持播放/暂停、回到起点、时间拖动和 0.25～4 倍速。**循环播放默认开启**：到末尾自动从头继续，可取消勾选使播放在末尾停止；循环仅重播缓存数据，不重新计算。工具栏支持取消后台计算和导出。改参数只影响下一次计算，已计算的数据保持不变。

### 参数导览：把机构与曲线对应起来

新页面标出机体俯仰 `pitch`、横滚 `roll`、航向 `yaw`、水平位置 `p`、速度 `v`、左右有效腿长 `hL/hR`、左右支撑力、轮电机转矩和两侧髋关节电机。

- 悬停引出标签或对应机构部位，查看名称、符号、当前值、单位及解释。
- 单击固定右侧说明；“取消固定选择”恢复跟随悬停。
- 双击标注，或点击“查看对应曲线”，跳到“平衡曲线”/“腿长与电机”并用浅橙色高亮对应子图。时间游标保持当前位置。
- `pitch` 是机体前后倾角；`qA/qE` 是五连杆主动髋关节的几何角，两个概念不同。选择髋关节可查看当前 `qA/qE`；对应现有曲线是其 VMC **力矩**，不会误称为关节角曲线。
- 固定斜视图保持航向不旋转，方便认清左右；航向箭头为方向示意，实际角度显示在说明里，实际姿态可查看三维页。尚未计算时显示明确标记的示意姿态。

### 只复制矩阵参数

在“C算法与增益”选项卡选择腿长节点 `0.16/0.20/0.24 m` 和矩阵 `A/B/Ad/Bd/Q/R/P/K`：

- **复制此矩阵（C 数组）**：仅复制二维花括号数组初始化值，无变量名、日志或说明。例如 K 为 `{{k0, k1, k2, k3}}`，可粘贴到 C 二维数组初始化位置。
- **复制此节点全部矩阵（JSON）**：只包含上述八个矩阵的名称和数组，不包含 PID、运行时间、迭代日志等字段。

矩阵预览为只读，并明确显示行列数。复制内容始终来自**最近完成的实验**；修改设置但未重新计算不会污染旧结果。Q/R 是该实验的输入权重，其余设计结果由 C 求解。完整计算详情仍可在下方滚动查看。

## 三个时间参数不能混淆

| 参数 | 默认 | 意义 |
|---|---:|---|
| 仿真总时长 | 10 s | 模拟多少秒；与计算消耗的墙钟时间无关 |
| C 控制周期 Ts | 0.001 s | 每隔多少仿真秒采样并调用 C 控制器；改变后由 C 重新求增益 |
| 物理积分步长 | 0.001 s | RK4 步长；可小于 Ts，一个控制周期内输入零阶保持 |
| 播放帧率 | 20 FPS | 只影响显示；不改变物理计算或 C 控制频率 |

例如 `Ts=0.001 s`、积分步长 `0.0001 s`、总时长 `10 s`：执行 10000 次 C 更新和 100000 次 RK4 步进。**先在后台算完整段并缓存，再播放**。小步长即使计算慢，也不会要求图形与计算实时同步；播放按墙钟时间定位已缓存数据，来不及显示的中间画面跳过，不丢物理积分步骤。

Ts 支持 0.0001～0.01 s；Ts 必须是积分步长的整数倍，总时长必须是 Ts 的整数倍。最多每次 500 万积分步、100 万记录样本。绘图显示最多约 3000 个抽样点，CSV/NPZ 保留所有控制采样点。

## C 底盘控制结构

```text
Python 连续模型 A(h), B(h) / 上位机 Q,R,Ts
                  ↓ 初始化一次
C: 矩阵指数离散化 → Riccati 迭代 → K(0.16), K(0.20), K(0.24)

每个控制周期：
Python 物理状态 → C Chassis_Update
                      ├─ 实际腿长 → 增益插值 → LQR → 前后驱动力
                      ├─ 左右腿长 PID + 重力前馈 → 左右支撑力
                      ├─ 横滚角 PID → 支撑力差
                      ├─ 航向角 PID → 差速转向转矩
                      └─ 五连杆 Jacobian/VMC + 电机力矩限幅
                                          ↓
                      Python 变腿长物理模型 + RK4
```

俯仰角与俯仰角速度已经属于 LQR 状态，不再无依据叠加一个俯仰 PID。腿长 PID 使用长度误差和实测伸缩速度，横滚/航向 PID 使用角度误差和对应角速度；D 项取测量导数，避免目标阶跃造成导数冲击。积分有条件积分与限幅；腿长和航向积分还跟踪实际执行器饱和后的输出。没有增设电流环或关节位置 PID，因为目前没有模拟电机电气与传动状态。

`controller/chassis.c` 使用结构体和静态存储，单个 DLL 实例代表一个 MCU。每次实验重置所有积分状态。Python 每次加载 DLL 临时副本，仿真结束卸载，因此上位机保持打开时也可重编译原 DLL，下一次实验使用新版本。

### C 求解 LQR

`controller/lqr_design.c` 用固定 5×5 数组计算：

$$\exp\left(T_s\begin{bmatrix}A&B\\0&0\end{bmatrix}\right)=\begin{bmatrix}A_d&B_d\\0&1\end{bmatrix}.$$

采用缩放、Taylor 展开与平方恢复；不调用 Python/NumPy/SciPy 求矩阵指数。之后从 `P0=Q` 开始求：

$$P_{n+1}=Q+A_d^TP_nA_d-A_d^TP_nB_d(R+B_d^TP_nB_d)^{-1}B_d^TP_nA_d.$$

每步保持 P 对称，相对 Frobenius 变化 <1e-12 时收敛，最多 300000 次。最后：

$$K=(R+B_d^TPB_d)^{-1}B_d^TPA_d,\qquad u=-K(x-x_{ref}).$$

初始化矩阵运算用 `double` 防止小采样周期下的精度问题；周期控制和 PID 用 `float`。全程固定大小数组，没有动态分配。

### 五连杆 VMC 与执行器

每侧 `A-B-C-D-E-A`：A/E 主动髋关节，B/D 被动膝关节，C 轮轴。AB/ED=0.13 m，BC/DC=0.20 m，AE=0.12 m，轮距 0.36 m。

本版轴向伸缩目标是髋中点正下方轮轴，采用向外膝关节、下方轮轴的支路。C 自行计算该构型的几何与 Jacobian，映射：

$$\tau_{hip}=J^T[0,-F_{support}]^T.$$

任一关节转矩达到限幅后，按比例减小对应腿的**实际作用支撑力**，不是只截断显示数值。左右轮等效映射为：

$$\tau_L=r(F/2-T_{yaw}/w),\quad\tau_R=r(F/2+T_{yaw}/w).$$

两侧分别限幅，再还原实际 `F=(tauL+tauR)/r` 和 `Tyaw=(tauR-tauL)w/(2r)` 给物理对象。默认单轮 1.5 Nm、单髋关节 12 Nm。

## 物理对象：具有关联控制环的简化底盘

状态顺序：

```text
p, v, pitch, pitch_rate, h, h_rate, roll, roll_rate, yaw, yaw_rate
```

`h` 为平均腿长，质心高度参数 `l=h+0.05`。左右有效腿长为 `hL=h+w*roll/2`、`hR=h-w*roll/2`，这是小横滚角近似。前后/俯仰/腿长三自由度采用变长度倒立摆的质量矩阵：

$$
\begin{bmatrix}
M+m&ml\cos\theta&m\sin\theta\\
ml\cos\theta&I+ml^2&0\\
m\sin\theta&0&m
\end{bmatrix}
\begin{bmatrix}\ddot p\\\ddot\theta\\\ddot h\end{bmatrix}
=\begin{bmatrix}
F-b\dot p+ml\sin\theta\dot\theta^2-2m\cos\theta\dot h\dot\theta\\
mgl\sin\theta-2ml\dot h\dot\theta\\
F_L+F_R-mg\cos\theta+ml\dot\theta^2-c_h\dot h
\end{bmatrix}.
$$

横滚/航向采用惯性与阻尼方程：`Iroll*roll_ddot = (FL-FR)*w/2 - broll*roll_rate`；`Iyaw*yaw_ddot = Tyaw - byaw*yaw_rate`。因此腿长改变会实际影响俯仰与前进动力学，左右支撑力确实改变横滚，差动轮力矩确实改变航向。

**这是控制架构完整串联的简化仿真，不是完整实机动力学。** 仍假设地面持续接触、无轮胎滑动、连杆质量忽略；未加入轮电机对机体的反作用力矩、电机电气、离地碰撞、完整三维科氏耦合或障碍物。前进位置 p 与航向按简化模型分开处理，三维展示的转向不代表完整平面 x/y 行驶轨迹。几何是 RoboMaster 风格示例尺寸。不要把模型直接用于实机。

模型参数目前固定在 `chassis_sim.py` 和 C 中的对应常量；若修改质量、轮径、轮距、杆长等，必须同步修改两侧并重新验证。上位机暴露的是时序、目标和控制参数。

## 默认实验与数据

- 初始俯仰 10°、横滚 2°、腿长 0.20 m。
- 2 s 时目标腿长变为 0.23 m，航向目标变为 15°，位置目标默认仍为 0。
- 4 s 时对轮轴施加 0.5 N·s 水平冲量；速度跳变由当前质量矩阵计算。
- 10 s 结束，曲线显示收敛结果。超出实验时长的事件不会发生。

本机测试：10 秒默认仿真约 0.7 秒完成（不含图片导出，耗时随机器而变）。默认最后一秒通过位置 <5 mm、俯仰 <0.1°、腿长误差 <1 mm、横滚 <0.1°、航向误差 <0.3° 的检验。

无界面批处理：

```powershell
python main.py --headless
python main.py --headless --duration 10 --control-dt 0.001 --physics-dt 0.0001
```

输出 `outputs/chassis/`：

```text
chassis.csv                 # 全部控制采样点：状态、执行器输出、参考值
chassis.npz                 # 同样的数据 + JSON 元信息
design_and_settings.json    # 本次设置及 C 设计矩阵
balance.png
legs_motors.png
robot_2d.png
robot_3d.png
```

上位机“导出”保存当前时间点的二维/三维图；无界面默认保存初始姿态。终端时刻没有下一条控制指令，CSV 最后一行输出为 NaN。仅保存数据与参数，不自动导出动画视频。

## 工程位置

```text
main.py                       # 上位机/批处理启动
simulation/host_app.py         # Tk 选项卡、后台线程、时间轴、导出
simulation/chassis_sim.py      # ctypes ABI、简化物理对象、积分和记录
simulation/chassis_view.py     # 曲线与动态腿长二维/三维图
controller/lqr_design.c/.h     # C ZOH 与 Riccati
controller/chassis.c/.h        # C LQR、PID、增益插值、VMC、限幅
simulation/verify_chassis.py   # C 算法/物理/闭环验证
simulation/verify_host.py      # 实际 Tk 控件与后台线程验证
```

第一阶段四状态 Python 离线求 K 的脚本仍保留作教学对照，见 `docs/stage1_history.md`；它们不属于当前默认上位机路径。`python main.py --leg-demo` 仍可启动独立几何伸缩演示。

## 验证

```powershell
mingw32-make verify       # 第一阶段回归 + 五连杆 + 新底盘
mingw32-make verify-gui   # Tk启动、七个页签、循环/拖动、矩阵剪贴板、交互标注、导出、取消
```

新增科学验证包括：4 种采样周期、3 个腿长节点上的 C 矩阵指数/DARE 与 SciPy 对照；164 个中间腿长点闭环稳定性；C VMC 与 Python Jacobian 对照；实际力矩限幅引起的支撑力降低；腿长/横滚/航向/平衡/冲量恢复；积分步长收敛；控制周期改变；重复实验状态重置和取消。

参考：[SciPy DARE 数学定义](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html)、[Python Tkinter 线程模型](https://docs.python.org/3.10/library/tkinter.html)。
