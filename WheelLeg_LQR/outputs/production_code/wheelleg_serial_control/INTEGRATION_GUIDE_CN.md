# 轮腿控制生成代码接入与自定义 PID 指南

## 1. 文件职责

| 文件 | 作用 | 是否建议修改 |
|---|---|---|
| `include/wheel_leg.h` | 模型公开入口，声明 U/Y/B/DW/P 和三个生命周期函数 | 否 |
| `include/wheel_leg_types.h` | 输入、输出、中间信号、状态和参数结构 | 通常不改 |
| `include/wheel_leg_user_types.h` | 外部 PID 端口和用户自定义存储 | 可以，重复导出会保留 |
| `include/wheel_leg_pid_interface.h` | 外部 PID 函数签名，不包含 PID 算法 | 否 |
| `src/wheel_leg.c` | `initialize/step/terminate` 顶层调度 | 否 |
| `src/wheel_leg_user.c` | 用户控制插入点 | 可以，重复导出会保留 |
| `src/wlc_internal.c` | 五连杆/串联腿运动学、LQR/VMC、状态机数值核心 | 不建议修改 |
| `generated/wlc_generated_params.h` | UI 生成的质量、几何、限幅和增益 | 不要手改，重新导出 |

## 2. 最小调用顺序

初始化只调用一次：

```c
#include "wheel_leg.h"

void ChassisControlInit(void)
{
    wheel_leg_initialize();
}
```

固定控制周期内，例如 1 kHz 定时器或 RTOS 任务：

```c
void ChassisControl1kHz(void)
{
    /* 1. 将实车传感器填入模型输入，全部使用 SI 单位。 */
    wheel_leg_U.pitch_rad          = imu_pitch_rad;
    wheel_leg_U.pitch_rate_rad_s   = gyro_pitch_rad_s;
    wheel_leg_U.roll_rad           = imu_roll_rad;
    wheel_leg_U.roll_rate_rad_s    = gyro_roll_rad_s;
    wheel_leg_U.yaw_rad            = imu_yaw_rad;
    wheel_leg_U.yaw_rate_rad_s     = gyro_yaw_rad_s;
    wheel_leg_U.distance_m         = chassis_distance_m;
    wheel_leg_U.velocity_m_s       = chassis_velocity_m_s;

    wheel_leg_U.joint_angle_rad[0] = left_back_angle_rad;
    wheel_leg_U.joint_angle_rad[1] = left_front_angle_rad;
    wheel_leg_U.joint_angle_rad[2] = right_back_angle_rad;
    wheel_leg_U.joint_angle_rad[3] = right_front_angle_rad;

    wheel_leg_U.joint_rate_rad_s[0] = left_back_rate_rad_s;
    wheel_leg_U.joint_rate_rad_s[1] = left_front_rate_rad_s;
    wheel_leg_U.joint_rate_rad_s[2] = right_back_rate_rad_s;
    wheel_leg_U.joint_rate_rad_s[3] = right_front_rate_rad_s;

    wheel_leg_U.normal_force_n[0] = left_normal_force_n;
    wheel_leg_U.normal_force_n[1] = right_normal_force_n;
    wheel_leg_U.time_s             = system_time_s;

    /* 2. 填入目标，不是在这里填写最终电机力矩。 */
    wheel_leg_U.velocity_command_m_s  = target_velocity_m_s;
    wheel_leg_U.yaw_rate_command_rad_s = target_yaw_rate_rad_s;
    wheel_leg_U.leg_length_command_m   = target_leg_length_m;
    wheel_leg_U.jump_command           = jump_requested;
    wheel_leg_U.zero_force_command     = emergency_zero_force;

    /* 3. 执行且只执行一个控制周期。 */
    wheel_leg_step();

    /* 4. 把模型输出的 Nm 力矩交给你的电机控制器。 */
    Motor_SetTorqueNm(LEFT_WHEEL,  wheel_leg_Y.wheel_torque_nm[0]);
    Motor_SetTorqueNm(RIGHT_WHEEL, wheel_leg_Y.wheel_torque_nm[1]);
    Motor_SetTorqueNm(LEFT_BACK,   wheel_leg_Y.joint_torque_nm[0]);
    Motor_SetTorqueNm(LEFT_FRONT,  wheel_leg_Y.joint_torque_nm[1]);
    Motor_SetTorqueNm(RIGHT_BACK,  wheel_leg_Y.joint_torque_nm[2]);
    Motor_SetTorqueNm(RIGHT_FRONT, wheel_leg_Y.joint_torque_nm[3]);
}
```

停止模块时可以调用：

```c
wheel_leg_terminate();
```

## 3. 六路力矩映射

生成参数 `WLC_GENERATED_LEG_TOPOLOGY` 为 `0` 时输入是五连杆的后/前髋角，为 `1` 时输入是串联腿的髋角/膝角。两种腿型对外仍保持左右轮与四个关节的六路力矩接口。

| 生成代码输出 | 含义 | 单位 |
|---|---|---:|
| `wheel_leg_Y.wheel_torque_nm[0]` | 左轮目标力矩 | Nm |
| `wheel_leg_Y.wheel_torque_nm[1]` | 右轮目标力矩 | Nm |
| `wheel_leg_Y.joint_torque_nm[0]` | 左后关节目标力矩 | Nm |
| `wheel_leg_Y.joint_torque_nm[1]` | 左前关节目标力矩 | Nm |
| `wheel_leg_Y.joint_torque_nm[2]` | 右后关节目标力矩 | Nm |
| `wheel_leg_Y.joint_torque_nm[3]` | 右前关节目标力矩 | Nm |

如果电机驱动接收电流而不是 Nm，换算只能放在硬件适配层，不要写进模型核心：

```c
float TorqueNmToCurrentA(float torque_nm)
{
    return torque_nm / (MOTOR_KT_NM_PER_A * REDUCTION_RATIO * TRANSMISSION_EFFICIENCY);
}
```

必须根据实物标定电机力矩常数、减速比、效率、方向和零位。不要直接把 Nm 数值当作电流发送。

## 4. 接入你自己的 PID 结构体

只修改下面两个用户文件：

```text
include/wheel_leg_user_types.h
src/wheel_leg_user.c
```

它们在同一目录重复导出时会被保留。生成代码本身不提供 PID 算法，只提供以下统一端口：

```c
typedef float (*WheelLeg_PIDCalculateFn)(void *instance,
                                         float measure,
                                         float reference);
typedef void (*WheelLeg_PIDResetFn)(void *instance);
```

这使你的 PID 结构体可以保持原样。以 `balance_chassis-main` 的 `PIDInstance`、`PIDInit()`、`PIDCalculate()` 为例，在你的嵌入式工程适配文件中写：

```c
#include "wheel_leg.h"
#include "controller.h"  /* 你自己的 PIDInstance/PIDCalculate */

static PIDInstance position_pid;
static PIDInstance roll_pid;

static float AppPIDCalculate(void *instance,float measure,float reference)
{
    return PIDCalculate((PIDInstance *)instance,measure,reference);
}

static void AppPIDReset(void *instance)
{
    PIDInstance *pid=(PIDInstance *)instance;
    pid->Iout=0.0f;
    pid->ITerm=0.0f;
    pid->Last_Err=0.0f;
    pid->Last_Output=0.0f;
}

void ChassisControlInit(void)
{
    PID_Init_Config_s position_config={
        .Kp=2.0f,.Ki=0.2f,.Kd=0.05f,.MaxOut=1.5f,
        .IntegralLimit=0.5f,
        .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit,
    };
    PID_Init_Config_s roll_config={
        .Kp=10.0f,.Ki=0.0f,.Kd=0.5f,.MaxOut=5.0f,
        .Improve=PID_Derivative_On_Measurement|PID_DerivativeFilter,
        .Derivative_LPF_RC=0.01f,
    };
    wheel_leg_initialize();
    PIDInit(&position_pid,&position_config);
    PIDInit(&roll_pid,&roll_config);
    wheel_leg_bind_pid(0u,&position_pid,AppPIDCalculate,AppPIDReset);
    wheel_leg_bind_pid(1u,&roll_pid,AppPIDCalculate,AppPIDReset);
}
```

如果你以后换另一套 PID，只需重写 `AppPIDCalculate()` 的这一层类型转换，生成代码不变。

端口和生成控制环对应关系：

| 端口 | 宏 | 调用参数 `(measure, reference)` |
|---:|---|---|
| 0 | `WLC_PID_LEFT_LENGTH` | 左腿实际长度、目标长度；输出腿速目标 |
| 1 | `WLC_PID_LEFT_SPEED` | 左腿实际伸缩速度、腿速目标；输出支撑力修正 |
| 2 | `WLC_PID_RIGHT_LENGTH` | 右腿实际长度、目标长度；输出腿速目标 |
| 3 | `WLC_PID_RIGHT_SPEED` | 右腿实际伸缩速度、腿速目标；输出支撑力修正 |
| 4 | `WLC_PID_ROLL` | 实际横滚角、0；输出左右腿支撑力差 |
| 5 | `WLC_PID_YAW_ANGLE` | 实际航向角、目标航向角；输出航向角速度目标 |
| 6 | `WLC_PID_YAW_RATE` | 实际航向角速度、目标角速度；输出差速转向力矩 |
| 7 | `WLC_PID_ANTI_SPLIT` | 左右虚拟腿角之差、0；输出抗劈叉髋力矩 |

你可以只绑定需要替换的控制环。没有绑定的端口继续使用生成核心的默认回退计算，因此可以逐个 PID 验证，不要求一次改完。

### 4.1 PID 放在目标输入前

例如用位置 PID 产生速度目标，在 `WheelLeg_UserBeforeStep()` 中写：

```c
void WheelLeg_UserBeforeStep(ExtU_wheel_leg_T *u,
                             const P_wheel_leg_T *p,
                             DW_wheel_leg_T *dw)
{
    const float target_position_m = p->user.parameter[0];
    (void)dw;
    u->velocity_command_m_s = wheel_leg_call_pid(
        0u,u->distance_m,target_position_m);
}
```

### 4.2 PID 叠加到最终力矩

例如增加横滚 PID，在 `WheelLeg_UserAfterControl()` 中写：

```c
void WheelLeg_UserAfterControl(const ExtU_wheel_leg_T *u,
                               const B_wheel_leg_T *b,
                               const P_wheel_leg_T *p,
                               DW_wheel_leg_T *dw,
                               ExtY_wheel_leg_T *y)
{
    float correction;
    (void)b;
    (void)p;(void)dw;
    correction = wheel_leg_call_pid(1u,u->roll_rad,0.0f);

    y->joint_torque_nm[0] += correction;
    y->joint_torque_nm[1] += correction;
    y->joint_torque_nm[2] -= correction;
    y->joint_torque_nm[3] -= correction;

    /* 自定义叠加后必须再次执行最终力矩限幅。 */
}
```

默认预留端口 `0`～`7`，但 PID 实例及其积分、滤波、DWT 和故障状态全部保存在你自己的 `PIDInstance` 中。生成代码只保存实例指针和调用函数，不复制 PID 状态。

## 5. 中间信号在哪里读取

`wheel_leg_B` 对应 Simulink 生成代码中的 block signals：

```c
wheel_leg_B.leg_length_m[2];
wheel_leg_B.virtual_leg_angle_rad[2];
wheel_leg_B.leg_rate_m_s[2];
wheel_leg_B.virtual_leg_rate_rad_s[2];
wheel_leg_B.support_force_n[2];
wheel_leg_B.virtual_hip_torque_nm[2];
wheel_leg_B.target_velocity_m_s;
wheel_leg_B.target_distance_m;
wheel_leg_B.target_yaw_rad;
```

自定义控制优先读取这些明确的中间信号，不要访问 `wheel_leg_DW.core` 内部数组。

## 6. 调用周期和保护

- `wheel_leg_step()` 必须按生成参数中的固定周期调用；默认是 0.001 s。
- 输入角度全部为 rad，角速度为 rad/s，长度为 m，速度为 m/s，力为 N，力矩为 Nm。
- 电机方向与数组映射必须先在拆载、小电流状态逐个确认。
- 应用层必须另外实现 CAN/传感器超时、看门狗、物理急停和掉线归零。
- 自定义 PID 修改最终力矩后必须重新限幅，并建议增加力矩变化率限制。
- 第一次实物测试必须使用保护架、低力矩和可切断电源的急停。

## 7. 编译文件

将以下源文件加入新的嵌入式工程：

```text
src/wheel_leg.c
src/wheel_leg_user.c
src/wlc_internal.c
```

增加头文件搜索路径：

```text
include/
generated/
```

核心只依赖标准 C 数学函数。若工具链需要，需要链接 `libm`；ARM GCC 通常在链接选项中加入 `-lm`。
