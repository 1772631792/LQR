#include "shoot.h"
#include "robot_def.h"

#include "bsp_dwt.h"
#include "dji_motor.h"
#include "general_def.h"
#include "message_center.h"
#include "stdbool.h"

static DJIMotorInstance *friction_l, *friction_r, *loader;  // 拨盘电机

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;  // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data;  // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit(void)
{
    // 左摩擦轮
    Motor_Init_Config_s friction_l_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 20
                .Ki = 2,  // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16384,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};

    friction_l = DJIMotorInit(&friction_l_config);

    // 左摩擦轮
    Motor_Init_Config_s friction_r_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 20
                .Ki = 2,  // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 16384,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_r = DJIMotorInit(&friction_r_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 0.2,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 50,
                .MaxOut = 9999999.f,
            },
            .speed_PID = {
                .Kp = 10, // 10
                .Ki = 2,  // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 8000,
                .MaxOut = 10000,
                .DeadBand = 0.1,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 步兵使用M2006,英雄使用M3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

uint8_t speederr_cnt = 0;
static void SpeedAdapt(float real_S, float min_S, float max_S, float *fix, float up_num, float down_num)
{
    if (real_S < min_S && real_S > 8)
        speederr_cnt++;
    else if (real_S >= min_S && real_S <= max_S)
        speederr_cnt = 0;
    if (speederr_cnt == 1)  // 射速偏低
    {
        speederr_cnt = 0;
        *fix += up_num;
    }
    if (real_S > max_S)  // 射速偏高
        *fix -= down_num;
}

float temp_fix_num = 0;
static void TempCtrlSpeed(void)
{
    float temp_scope = 35;
    float temp_low = 35;
    float res = 0;
    float temp_real = 0;

    temp_real = ((float)friction_l->measure.temperature + (float)friction_r->measure.temperature) / 2;

    if (temp_real >= temp_low)
        res = (temp_real - temp_low) / temp_scope * (-168);
    if (temp_real < temp_low)
        res = 0;
    if (temp_real > temp_low + temp_scope)
        res = -168;

    temp_fix_num = res;
}

uint16_t prev_shoot_num = 0;
bool if_speed_update = false;
float fix_num = 0;
static void FricSpeedFix(void)
{
    float realspeed = shoot_cmd_recv.bullet_speed;
    if (prev_shoot_num != shoot_cmd_recv.bullet_cnt)
    {
        prev_shoot_num = shoot_cmd_recv.bullet_cnt;
        if_speed_update = true;
    }
    else
        if_speed_update = false;

    if (if_speed_update)
    {
        SpeedAdapt(realspeed, 23.0f, 24.0f, &fix_num, 25, 50);
    }
    TempCtrlSpeed();
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else  // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    FricSpeedFix();

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP);  // 切换到速度环
            DJIMotorSetRef(loader, 0);              // 同时设定参考值为0,这样停止的速度最快
            break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
        case LOAD_1_BULLET:                         // 激活能量机关/干扰对方用,英雄用.
            DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 切换到角度环			// 控制量增加一发弹丸的角度
            DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE);
            hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
            dead_time = 700;                        // 完成1发弹丸发射的时间
            break;
        // 三连发,如果不需要后续可能删除
        case LOAD_3_BULLET:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                             // 切换到速度环
            DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE);  // 增加3发
            hibernate_time = DWT_GetTimeline_ms();                                             // 记录触发指令的时间
            dead_time = 700;                                                                   // 完成3发弹丸发射的时间
            break;
            // 连发模式,对速度闭环
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            // 1秒一颗:1秒转60度, 1秒6颗:1秒转360度
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * ONE_BULLET_PER_SECOND);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
            // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
            // 也有可能需要从switch-case中独立出来
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle - 0.66f * ONE_BULLET_DELTA_ANGLE);
            hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
            dead_time = 500;
            break;
        default:
            while (1)
                ;  // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }


    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
        DJIMotorSetRef(friction_l, 6200 + fix_num + temp_fix_num);
        DJIMotorSetRef(friction_r, 6200 + fix_num + temp_fix_num);
    }
    else  // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }
    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
