# 参考固件到 MuJoCo 的映射

## 复用与边界

参考目录 `balance_chassis-main` 未修改。`controller/firmware_bridge.c` 编译时直接包含原始 `balance.h`、`linkNleg.h`、`lqr_calc.h`，复用 `Link2Leg`、`VMCProject`、`CalcLQR_MPC_Fusion` 和两组 12×4 增益系数。源仓库 MIT 许可及版权信息见 `balance_chassis-main/LICENSE`（Copyright 2022 NeoZng），保留在工程中。

`balance.c` 的底盘控制顺序通过桌面适配器重建，未把 RTOS、HAL、CAN、电机驱动、云台和射击任务搬入 DLL。`firmware_compat` 只补足源头文件依赖和 CMSIS 三角函数。在线代码使用过程式 C、静态结构体和固定数组，无堆分配。每个 Python Firmware 实例加载独立的临时 DLL 副本，避免静态状态串扰及阻塞 Ctrl+Shift+B 重编译。

| 来源 | 仿真实现 |
|---|---|
| `application/chassis/linkNleg.h` | 原始五连杆正运动学、速度和 Jacobian / VMC |
| `application/chassis/lqr_calc.h` | 原始六状态双输出增益调度与 LQR/MPC 融合 |
| `application/chassis/balance.c` | 腿长双环、横滚、航向、抗劈叉和越阶状态顺序在适配器重建 |
| `application/chassis/fly_detection.h` | 用 MuJoCo 接触反力替代固件的力估计，20 N 阈值 |
| `application/cmd/robot_cmd.c` | 参考 C、G、Shift、Ctrl+C；将 WSAD 改为直观的车辆前后与转向 |
| `matlab.zip/GenerateGains.m` | `simulation/source_design.py` + `controller/riccati6.c` |

## 坐标与输入输出

世界 X 前、Y 左、Z 上；轮轴沿 Y。MuJoCo 世界俯仰与固件 pitch 反号，适配器在传感器边界统一。每腿状态为 `[theta, theta_rate, distance, velocity, pitch, pitch_rate]`，其中 `theta = phi0 - π/2 - pitch`，不是单个髋电机角度。速度投影到车头水平朝向，积分得到沿行驶路径距离。

Python 每 1 ms 提交机身姿态、速度、左右主动关节角和角速度、接触反力；C 返回左右轮力矩与四个髋电机力矩，MuJoCo 电机执行后推进动力学。物理可在一个控制周期内做 1/2/4 次积分，期间保持力矩不变。Python 不执行在线 PID 或 LQR 决策。

仿真输入是理想关节/IMU 数据，尚未加入编码器量化、延时、滤波和状态观测器。接触法向来自物理引擎，不能将其当作已经实现了实车离地估计器。

## 控制链

- 每腿 wheel 输出采用 LQR；hip 输出按原函数使用 0.7 LQR + 0.3 MPC 静态增益。这里的 MPC 来自离线五步无约束问题，不是在线约束优化器。
- 离地时轮输出归零；hip 只保留虚拟腿角及角速度反馈。
- 腿长位置环 Kp=10，速度参考 ±2 m/s；腿长速度 PI：Kp=300、Ki=50、积分 ±50、输出 ±160 N。加上源代码的 BODY_MASS×g 前馈。
- 横滚 Kp=4000、限幅 ±400 N；航向外环 Kp=5、±3 rad/s，角速度环 Kp=3、±1.5 Nm。
- 抗劈叉 Kp=30、Kd=2、±60 Nm，转向前馈系数 3，速度斜坡 2.5 m/s²。
- C 低/高腿目标 .18/.26 m；G 慢/快速度 1.5/2.5 m/s。Shift 对应原越阶顺序：先伸腿至 .33 m，虚拟腿角超过约 23° 后 -120 N 缩腿 0.2 秒，再保持 .15 m，释放后复位。它不是通用弹跳轨迹规划。
- 最终模拟限幅：每个轮 ±8 Nm，每个髋 ±35 Nm。Ctrl+C 进入零力；姿态超过约 69° 时上位机停止本次计算并保留记录。

PID 的硬件滤波、原框架积分细节及电机标定未逐位复现。源代码 BODY_MASS 的单腿重力补偿语义存在整机/半机口径差异，当前保留原表达式，没有把稳定性解释成真实硬件质量标定正确。

## MuJoCo 模型

主动杆长 .135 m、从动杆长 .24 m、髋间距 .12 m、轮半径 .075 m、轮距 .52 m，取自参考固件。机身质量 7.645 kg、俯仰惯量 .16 kg·m²、每轮 .865 kg。五连杆用真实转动关节和足端 connect 闭链约束，两腿各两个主动、两个被动关节；轮子有独立转动关节。机身 freejoint 允许完整三维运动。

机身横滚/航向惯量、杆质量分配、摩擦、阻尼和接触柔顺性为仿真假设，尚无 CAD/实测标定。杆件关闭碰撞，轮胎和机壳参与碰撞；因此不能评估真实杆件撞台阶、结构强度或线缆干涉。地图飞坡长 1.6 m、高 .32 m，平台长 1 m，末端直接落地；独立台阶高 .04 m。

测试实际检查到轮胎与坡面、平台及台阶的接触；并非仅渲染障碍外观或沿预设轨迹移动。高腿、0.7 m/s 默认脚本完成飞坡和 4 cm 台阶，飞坡后可恢复落地。8 cm 台阶测试出现倾覆，所以不宣称支持该高度，也不将当前模型当作实车可直接部署的控制器验证。

## 代替 MATLAB 的数学过程

`source_design.matrices(h)` 对参考 GenerateGains 的三条线性动力学方程进行数值系数求解，生成 6×6 A 和 6×2 B。令 `F=[[A,B],[0,0]]`，`G=diag(Q,R)`。零阶保持的 `exp(F Ts)` 给出 Ad、Bd；通过块矩阵指数求解

`S = integral(exp(Fᵀ t) G exp(F t), t=0..Ts)`。

S 分块得到 Qd、Rd 和交叉代价 Nd。保留 Nd 是为了对应 MATLAB `lqrd` 的连续时间代价，而不是简单把 Q/R 直接当作离散权重。

离散指标：`J = sum(xᵀ Qd x + 2 xᵀ Nd u + uᵀ Rd u)`。

C 从 P=Qd 开始，固定内存迭代：

```
K = (Rd + Bdᵀ P Bd)^(-1) (Bdᵀ P Ad + Ndᵀ)
P_next = Qd + Adᵀ P Ad - (Adᵀ P Bd + Nd) K
```

每次对 P 做对称化，相对变化阈值 1e-12，最大迭代 300000；失败显式返回错误。Python 的 SciPy DARE 仅用于结果比对，不替代 C 求解。每个节点输出 A、B、Ad、Bd、Qd、Rd、Nd、P、K、迭代次数、闭环复特征值和谱半径，公共 Q/R 同时保存。

节点为 .10 至 .39 m，步距 .01 m。再按原脚本求五步无约束 MPC 静态矩阵，对两组增益分别做三次多项式拟合。额外在 .14–.33 m 取 100 点验证 float 系数调度及实际 0.7/0.3 混合后的线性特征值全部位于单位圆内。该检查不保证饱和、接触、飞行及姿态大角度时的全局稳定。

默认参数与固件 LQR 系数表最大绝对差约 2.3869e-4，MPC 约 1.0695e-5；这包括源表小数截断及数值实现差异。Q/R 修改后生成的是新控制器，差值不应再被解释为复现精度。完整输出在 `outputs/source_design/`，GUI 复制按钮只复制选中的二维矩阵初始化值。

## 实时、回放与验证

一个工作线程独占 MuJoCo 模型、OpenGL 渲染上下文及 C DLL；Tk 主线程处理输入、显示和剪贴板。1 kHz 控制与约 30 FPS 渲染分开调度，画面队列最多保留最新一帧，避免渲染积压。每批最多追赶 50 个控制步，不跳过积分；机器过慢时 wall 比率会小于 1。预计算无墙钟限速，完成后按 50 Hz 缓存姿态回放；回放保持原始机构初始几何，不调用控制器重新计算。

`physics` 为物理+C 的计算吞吐比，`wall` 为本次仿真时间/经过墙钟时间，FPS 为输出画面速率。回放时 wall 不表示动力学吞吐。记录里保存 18 个传感器量、20 个控制遥测量、6 个电机输出及时间，CSV 有列名；NPZ 额外保存 qpos、初始腿长、子步数和增益来源。

验证命令：

```
mingw32-make verify
python -m simulation.verify_host
mingw32-make verify-mujoco
mingw32-make design-source
```

`outputs/mujoco/verification.json` 保存实测接触、闭链误差、峰值倾角与性能；`upper_computer.png` 是本机实际 Tk 窗口截图。

全局 Python 使用 MuJoCo 3.3.7。本机原先 3.13 包存在 Windows DLL 初始化失败，已在全局环境替换并验证。为避免本机混用旧 MSVC DLL，`mujoco_bootstrap.py` 先加载 `config/windows_runtime` 中配套运行库；未替换 System32 文件。运行库来源为 `msvc-runtime 14.42.34433` wheel，附带其分发说明。更换机器时先安装 requirements，保留这组运行库及许可文件。
