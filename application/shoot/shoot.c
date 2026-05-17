#include "shoot.h"
#include <math.h>
#include "robot_board.h"
#include "robot_params.h"
#include "robot_types.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "tool.h"
#include "referee_UI.h"
#include "led.h"
#include "DMmotor.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r; // 拨盘电机
// float DM_4310_speed_target = 0;                                                                                             // DM电机目标
// int DM_enable_flag         = 0;                                                                                             // DM电机指令,1转0停
static Publisher_t *shoot_pub;
static Subscriber_t *shoot_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;         // 来自cmd的发射控制信息
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static int16_t hit_cnt = 3000;
// static int16_t shoot_delay=3000;
// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
float d_watch;
int32_t shoot_count;                     // 已发弹量
#define DEBOUNCE_DELAY_MS 50             // 消抖延迟时间，单位：毫秒
#define BUTTON_PRESSED    GPIO_PIN_RESET // 按键按下时引脚电平，通常为低电平
#define BUTTON_RELEASED   GPIO_PIN_SET   // 按键释放时引脚电平，通常为高电平
int load_speed           = 15000;
#define SHOOT_BULLET_SPEED_LIMIT_MPS        25.0f
#define SHOOT_BULLET_SPEED_TARGET_MPS       24.8f
#define SHOOT_BULLET_SPEED_VALID_MIN_MPS    5.0f
#define SHOOT_BULLET_SPEED_VALID_MAX_MPS    35.0f
#define SHOOT_FRIC_SPEED_TARGET_DEFAULT     35500.0f
#define SHOOT_FRIC_SPEED_TARGET_MIN         30000.0f
#define SHOOT_FRIC_SPEED_TARGET_MAX         36500.0f
#define SHOOT_FRIC_SPEED_ADJUST_K           550.0f
#define SHOOT_FRIC_SPEED_DOWN_STEP_MAX      1500.0f
#define SHOOT_FRIC_SPEED_UP_STEP_MAX        250.0f
#define SHOOT_OVERSPEED_FEED_HOLD_MS        300.0f
float shoot_speed_target = SHOOT_FRIC_SPEED_TARGET_DEFAULT, shoot2_speed_target = SHOOT_FRIC_SPEED_TARGET_DEFAULT, limit_speed_target = 400;
// 定义按键状态
typedef enum {
    BUTTON_STATE_IDLE,     // 按键未按下状态
    BUTTON_STATE_DEBOUNCE, // 按键消抖中
    BUTTON_STATE_PRESSED   // 按键已稳定按下
} ButtonState;

// 定义按键状态变量
static ButtonState buttonState   = BUTTON_STATE_IDLE;
static uint32_t lastDebounceTime = 0; // 记录消抖开始时间
void Laser_Init()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // 使能GPIOC时钟
    GPIOC->MODER &= ~(0x3 << (2 * 8));   // 清除PC8的模式位
    GPIOC->MODER |= (0x1 << (2 * 8));    // 配置为输出模式 (01)
    GPIOC->OTYPER &= ~(0x1 << 8);        // 配置为推挽输出（0）
    GPIOC->OSPEEDR |= (0x3 << (2 * 8));  // 设置输出速度为高速
    GPIOC->ODR &= (0x1 << 8);            // 设置PC8为高电平
}
void Laser_on()
{
    GPIOC->ODR |= (0x1 << 8); // 设置PC8为高电平
}
void Laser_off()
{
    GPIOC->ODR &= ~(0x1 << 8); // 设置PC8为高电平
}

// 按键读取函数
bool Button_ReadState(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
}

// 按键消抖处理函数
void Button_Debounce(void)
{
    bool buttonReading   = Button_ReadState();   // 读取当前按键状态
    uint32_t currentTime = DWT_GetTimeline_ms(); // 获取当前时间，单位为毫秒

    switch (buttonState) {
        case BUTTON_STATE_IDLE:
            if (buttonReading == BUTTON_PRESSED) {
                // 按键按下，进入消抖状态
                buttonState      = BUTTON_STATE_DEBOUNCE;
                lastDebounceTime = currentTime; // 记录按键按下的时间
            }
            break;

        case BUTTON_STATE_DEBOUNCE:
            // 如果按键保持按下状态超过消抖延迟时间
            if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
                if (buttonReading == BUTTON_PRESSED) {
                    // 按键稳定按下，进入按下状态
                    buttonState = BUTTON_STATE_PRESSED;
                    shoot_count++;
                    C_board_LEDSet(0xFF0000); // printf("Button Pressed!\n");  // 执行按键按下时的操作
                } else {
                    // 如果按键松开，返回空闲状态
                    buttonState = BUTTON_STATE_IDLE;
                }
            }
            break;

        case BUTTON_STATE_PRESSED:
            // 检测按键是否松开
            if (buttonReading == BUTTON_RELEASED) {
                // 按键释放，回到空闲状态
                buttonState = BUTTON_STATE_IDLE;
                C_board_LEDSet(0x00FF00); // printf("Button Released!\n");  // 执行按键释放时的操作
            }
            break;
    }
}
float friction_feedforwardl = 100, friction_feedforwardr2 = -100;

DJIMotorInstance *loader;

static float loader_offset_angle = 25;
static float current_angle;

void ShootInit()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 0.3,
                .Ki            = 0,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 15000,
                .MaxOut        = 10000,
            },

        },

        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
            // .feedforward_flag   = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508};

    friction_config.can_init_config.tx_id                                = 5; // 左摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL;
    // friction_config.controller_param_init_config.current_feedforward_ptr = &friction_feedforwardr2;
    friction_l                                                           = DJIMotorInit(&friction_config);
    // 三摩擦轮外加电机
    friction_config.can_init_config.tx_id                             = 6; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    // friction_config.controller_param_init_config.current_feedforward_ptr = &friction_feedforwardl;
    friction_r                                                      = DJIMotorInit(&friction_config);
    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
    // 限位电机
    // Motor_Init_Config_s limit_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan1,
    //         .tx_id      = 1,
    //     },
    //     .controller_param_init_config = {
    //         .speed_PID = {
    //             .Kp            = 1,  // 10
    //             .Ki            = 10, // 1
    //             .Kd            = 0,
    //             .Improve       = PID_Integral_Limit,
    //             .IntegralLimit = 10000,
    //             .MaxOut        = 10000,
    //         },

    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED, .outer_loop_type = SPEED_LOOP, .close_loop_type = SPEED_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
    //     },
    //     .motor_type = M2006,
    // };
    // limit = DJIMotorInit(&limit_config);
    // DJIMotorStop(limit);
    Laser_Init();
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 6,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp     = 120,//50, // 11
                .Ki     = 0,
                .Kd     = 3,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 150000, // 200,
            },
            .speed_PID = {
                .Kp            = 1, // 10
                .Ki            = 0, // 1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 3000,
                .MaxOut        = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, 
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = ANGLE_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type    = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };

    loader = DJIMotorInit(&loader_config);

    DJIMotorStop(loader);


#endif
    shoot_cmd_recv.shoot_mode = SHOOT_ON; // 初始化后摩擦轮进入准备模式,也可将右拨杆拨至上一次来手动开启

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // DJIMotorStop(friction_l);
    // DJIMotorStop(friction_r);
    // DJIMotorStop(loader);
    // Laser_Init();
}

float Block_Time;              // 堵转时间
float Reverse_Time;            // 反转时间
float current_record[6] = {0}; // 第五个为最近的射速 第六个为平均射速
float Block_Status;            // 拨弹盘状态
/**
 * @brief 堵转，弹速检测
 *
 */
// static void Load_Reverse()
// {
//     // 获取拨弹盘转速
//     current_record[0] = current_record[1];
//     current_record[1] = current_record[2];
//     current_record[2] = current_record[3];
//     current_record[3] = current_record[4];
//     // current_record[4] = loader->measure.real_current; // 第五个为最近的拨弹盘電流
//     current_record[5] = (current_record[0] + current_record[1] + current_record[2] + current_record[3] + current_record[4]) / 5.0;

//     if (current_record[5] < -9900) {
//         Block_Time++;
//     }
//     if (Reverse_Time >= 1) {
//         shoot_cmd_recv.load_mode = LOAD_REVERSE;
//         // shoot_cmd_recv.shoot_aim_angle=loader->measure.total_angle+10;
//         Reverse_Time++;

//         // 反转时间
//         if (Reverse_Time >= 200) {
//             Reverse_Time = 0;
//             Block_Time   = 0;
//         }
//     }

//     // 电流较大恢复正转
//     if (loader->measure.real_current > 2000) {
//         Reverse_Time = 0;
//         Block_Time   = 0;
//     }

//     else {
//         // 堵转时间5*发射任务周期（5ms）= 25ms
//         if (Block_Time > 200) {
//             Reverse_Time = 1;
//         }
//     }
// }

int heat_control    = 25; // 热量控制
float local_heat    = 0;  // 本地热量
int One_bullet_heat = 10; // 打一发消耗热量

// 热量控制算法
static float last_loader_feed_ms = -SHOOT_FIRE_MIN_INTERVAL_MS;
static float fric_last_shot_ms = -SHOOT_FIRE_MIN_INTERVAL_MS;
static float fric_drop_lockout_until_ms = 0.0f;
float shoot_fric_speed_drop_debug = 0.0f;
float shoot_fric_speed_avg_debug = 0.0f;

static void Shoot_Fric_data_process(void)
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#if SHOOT_FRIC_DROP_COUNT_ENABLE
    static uint8_t bullet_waiting_confirm2 = 0u;
    static float speed_filtered = 0.0f;
    static float speed_filtered_last = 0.0f;
    const float now_ms = DWT_GetTimeline_ms();
    const float speed_avg = 0.5f * (fabsf(friction_l->measure.speed_aps) +
                                    fabsf(friction_r->measure.speed_aps));
    const float target = 0.5f * (fabsf(shoot_speed_target) + fabsf(shoot2_speed_target));
    float derivative;

    if (shoot_cmd_recv.friction_mode == FRICTION_OFF) {
        bullet_waiting_confirm2 = 0u;
        speed_filtered = speed_avg;
        speed_filtered_last = speed_filtered;
        return;
    }

    speed_filtered += SHOOT_FRIC_DROP_LPF_ALPHA * (speed_avg - speed_filtered);
    derivative = speed_filtered - speed_filtered_last;
    speed_filtered_last = speed_filtered;
    d_watch = derivative;
    shoot_fric_speed_drop_debug = derivative;
    shoot_fric_speed_avg_debug = speed_avg;

    if (now_ms < fric_drop_lockout_until_ms)
        return;

    if (bullet_waiting_confirm2 == 0u &&
        target > 1.0f &&
        speed_avg > target * SHOOT_FRIC_READY_RATIO &&
        derivative < -SHOOT_FRIC_DROP_TRIGGER_APS) {
        bullet_waiting_confirm2 = 1u;
    } else if (bullet_waiting_confirm2 != 0u &&
               (derivative > SHOOT_FRIC_RECOVER_TRIGGER_APS ||
                (target > 1.0f && speed_avg > target * SHOOT_FRIC_READY_RATIO))) {
        local_heat += One_bullet_heat;
        shoot_count++;
        fric_last_shot_ms = now_ms;
        fric_drop_lockout_until_ms = now_ms + SHOOT_FRIC_DROP_LOCKOUT_MS;
        bullet_waiting_confirm2 = 0u;
    }
    return;
#else
    /*----------------------------------变量常量------------------------------------------*/
    static bool bullet_waiting_confirm = false; // 等待比较器确认

    //获得第一级任一摩擦轮的转速
    float data = friction_l->measure.speed_aps; // 获取摩擦轮转速

    static uint16_t data_histroy[MAX_HISTROY]; // 做循环队列
    static uint8_t head = 0, rear = 0;         // 队列下标
    float moving_average[2];                   // 移动平均滤波
    uint8_t data_num;                          // 循环队列元素个数
    float derivative;                          // 微分
    /*-----------------------------------逻辑控制-----------------------------------------*/
    data = abs(data);
    /*入队*/
    data_histroy[head] = data;
    head++;
    head %= MAX_HISTROY;
    /*判断队列数据量*/
    data_num = (head - rear + MAX_HISTROY) % MAX_HISTROY;
    if (data_num >= Fliter_windowSize + 1) // 队列数据量满足要求
    {
        moving_average[0] = 0;
        moving_average[1] = 0;
        /*同时计算两个滤波*/
        for (uint8_t i = rear, j = rear + 1, index = rear; index < rear + Fliter_windowSize; i++, j++, index++) {
            i %= MAX_HISTROY;
            j %= MAX_HISTROY;
            moving_average[0] += data_histroy[i];
            moving_average[1] += data_histroy[j];
        }
        moving_average[0] /= Fliter_windowSize;
        moving_average[1] /= Fliter_windowSize;
        /*滤波求导*/
        derivative = moving_average[1] - moving_average[0];
        d_watch    = derivative;
        /*导数比较*/
        if (derivative < -300) {
            bullet_waiting_confirm = true;
        } else if (derivative > 30) {
            if (bullet_waiting_confirm == true) {
                local_heat += One_bullet_heat; // 确认打出
                shoot_count++;
                bullet_waiting_confirm = false;
            }
        }
        rear++;
        rear %= MAX_HISTROY;
    }
#endif
#endif
}

static float CalculateNextAngle(float current_angle)
{
    const float angle_step = PI / 3.0f;
    float target_angle, temp;
    uint16_t number;//与正方向相差几个PI/3
    temp = loader_offset_angle - current_angle + 0.2;
    number = (uint16_t)(temp / angle_step);
    target_angle = loader_offset_angle - ((float)(number + 1) * angle_step);
    return target_angle;
}

static int one_bullet;
static ramp_t fric_on_ramp, fric2_on_ramp;
static ramp_t fric_off_ramp, fric2_off_ramp;
static ramp_t limit_on_ramp;
static ramp_t limit_off_ramp;
float fric_speed = 0, limit_speed = 0, fric2_speed = 0; // 摩擦轮转速参考值
uint32_t shoot_heat_count[2];
int load_mode_private = 1;
float diff1, diff2;
uint8_t loadmode;
float pid_kp = 1;
static float shoot_adaptive_fric_target = SHOOT_FRIC_SPEED_TARGET_DEFAULT;
static float last_bullet_speed_sample = 0.0f;
static float last_bullet_speed_process_ms = 0.0f;
static float bullet_speed_feed_hold_until_ms = 0.0f;

loader_mode_e last_load_mode = LOAD_STOP;
loader_state_e loader_state = LOAD_UNINIT;
float loader_initial_offset = 0;
float loader_pitch_offset = 0;
float loader_offset = 0;
int32_t load_count = 0;
float cool_down_time = 0;
float load_time_ms = 0;

loader_mode_e last_mode;
// friction_mode_e friction_text_mode = FRICTION_OFF;
// shoot_mode_e shoot_text_mode       = SHOOT_OFF;
// static int cnt                     = 100;

static float ShootClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static uint8_t ShootIsFeedMode(loader_mode_e mode)
{
    return (uint8_t)(mode == LOAD_1_BULLET || mode == LOAD_3_BULLET || mode == LOAD_BURSTFIRE);
}

static float ShootLoaderTargetAngle(int32_t count)
{
    return (float)count * SHOOT_LOADER_ANGLE_PER_BULLET + loader_initial_offset + loader_offset + loader_pitch_offset;
}

static uint8_t ShootLoaderCanStep(float now_ms)
{
    return (uint8_t)((now_ms - last_loader_feed_ms) >= SHOOT_FIRE_MIN_INTERVAL_MS);
}

static uint8_t ShootFrictionReady(void)
{
#if SHOOT_FIRE_FRICTION_GATE_ENABLE && (defined(ONE_BOARD) || defined(GIMBAL_BOARD))
    const float target = 0.5f * (fabsf(shoot_speed_target) + fabsf(shoot2_speed_target));
    const float speed_avg = 0.5f * (fabsf(friction_l->measure.speed_aps) + fabsf(friction_r->measure.speed_aps));

    shoot_fric_speed_avg_debug = speed_avg;

    if (shoot_cmd_recv.friction_mode == FRICTION_OFF)
        return 0u;

    if ((DWT_GetTimeline_ms() - fric_last_shot_ms) < SHOOT_FIRE_MIN_INTERVAL_MS)
        return 0u;

    if (target > 1.0f && speed_avg < target * SHOOT_FRIC_READY_RATIO)
        return 0u;
#endif
    return 1u;
}

static void ShootBulletSpeedLimitUpdate(void)
{
    const float bullet_speed = shoot_cmd_recv.bullet_speed;
    const float now_ms = DWT_GetTimeline_ms();
    float target_delta;

    if (bullet_speed < SHOOT_BULLET_SPEED_VALID_MIN_MPS ||
        bullet_speed > SHOOT_BULLET_SPEED_VALID_MAX_MPS) {
        return;
    }

    if (bullet_speed > last_bullet_speed_sample - 0.02f &&
        bullet_speed < last_bullet_speed_sample + 0.02f) {
        if (bullet_speed < SHOOT_BULLET_SPEED_LIMIT_MPS ||
            now_ms - last_bullet_speed_process_ms < 500.0f) {
            return;
        }
    }

    target_delta = (SHOOT_BULLET_SPEED_TARGET_MPS - bullet_speed) * SHOOT_FRIC_SPEED_ADJUST_K;
    target_delta = ShootClampFloat(target_delta,
                                   -SHOOT_FRIC_SPEED_DOWN_STEP_MAX,
                                   SHOOT_FRIC_SPEED_UP_STEP_MAX);

    if (bullet_speed >= SHOOT_BULLET_SPEED_LIMIT_MPS) {
        target_delta = -SHOOT_FRIC_SPEED_DOWN_STEP_MAX;
        bullet_speed_feed_hold_until_ms = now_ms + SHOOT_OVERSPEED_FEED_HOLD_MS;
    }

    shoot_adaptive_fric_target = ShootClampFloat(shoot_adaptive_fric_target + target_delta,
                                                 SHOOT_FRIC_SPEED_TARGET_MIN,
                                                 SHOOT_FRIC_SPEED_TARGET_MAX);
    shoot_speed_target = shoot_adaptive_fric_target;
    shoot2_speed_target = shoot_adaptive_fric_target;
    last_bullet_speed_sample = bullet_speed;
    last_bullet_speed_process_ms = now_ms;
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    //调试内容：自动打开摩擦轮/拨弹盘
    // if (cnt > 0) {
    //     cnt--;
    // } else {
    //     friction_text_mode = FRICTION_ON;
    // }
    // diff1 = fabs(friction_l->measure.speed_rpm + friction_r->measure.speed_rpm);
    // diff2 = fabs(friction_l2->measure.speed_rpm + friction_r2->measure.speed_rpm);
    static float shoot_speed=0, shoot2_speed=0, limit_shoot_speed;
    
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    ShootBulletSpeedLimitUpdate();
    if (ShootIsFeedMode(shoot_cmd_recv.load_mode) &&
        DWT_GetTimeline_ms() < bullet_speed_feed_hold_until_ms) {
        shoot_cmd_recv.load_mode = LOAD_STOP;
    }
    if (ShootIsFeedMode(shoot_cmd_recv.load_mode) && ShootFrictionReady() == 0u) {
        shoot_cmd_recv.load_mode = LOAD_STOP;
    }
    shoot_cmd_recv.shoot_rate = 8;//射频切换
    // shoot_cmd_recv.friction_mode = friction_text_mode;
    // shoot_cmd_recv.shoot_mode    = shoot_text_mode;
    // loadmode                     = shoot_cmd_recv.load_mode;
    Button_Debounce();
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
        DJIMotorStop(friction_l);
        // DJIMotorStop(friction_r);
        // DJIMotorStop(friction_up);
        // DJIMotorStop(friction_l2);
        // DJIMotorStop(friction_r2);
        DJIMotorStop(friction_r);
        // DJIMotorStop(limit);
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
        DJIMotorStop(loader);
        // DM_enable_flag = 0;
        // DMMotorStop(loader);
#endif
    } else // 恢复运行
    {
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
        DJIMotorEnable(friction_l);
        // DJIMotorEnable(friction_r);
        // DJIMotorEnable(friction_up);
        // DJIMotorEnable(friction_l2);
        // DJIMotorEnable(friction_r2);
        DJIMotorEnable(friction_r);
        // DJIMotorEnable(limit);
// DJIMotorStop(friction_r);
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
        DJIMotorEnable(loader);
        // DM_enable_flag = 1;
        // DMMotorEnable1(loader);
        // if(loader->measure.state == 0)
        // {
        //     loader->ctrl.pos_set = loader->measure.pos;
        //     DMMotorEnableMode(loader);
        //     CANTransmit(loader->motor_can_instance, 0.2);
        // }
#endif
    }
    // 调试内容；模拟单发
    // if(shoot_cmd_recv.load_mode==LOAD_1_BULLET) shoot_text_mode=1;
    // if (shoot_text_mode == 1) {
    //     hit_cnt--;
    //     if (hit_cnt == 0) {
    //         shoot_text_mode = 0;
    //         hit_cnt         = 3000;
    //     }
    // }
// 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
// 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
// if (hibernate_time + dead_time > DWT_GetTimeline_ms())
//     return;
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // Load_Reverse();
    if(loader_state == LOAD_UNINIT && loader->measure.ecd != 0) //确认已连接到拨盘
    {
        loader_initial_offset = loader->measure.total_angle;
        load_count = 0;
        loader_state = LOAD_REINIT;
    }
    cool_down_time = 1000 / shoot_cmd_recv.shoot_rate;
#endif
    //    if(load_mode_private)shoot_aim_angle=loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE;
    //    if(!load_mode_private)shoot_cmd_recv.load_mode = LOAD_STOP;
    //    load_mode_private=shoot_cmd_recv.load_mode;
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换

    switch (shoot_cmd_recv.load_mode) {
        // 停止拨盘
        case LOAD_STOP:
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
            Laser_off();
            // DJIMotorSetRef(limit, -20000);
            // ramp_init(&limit_on_ramp, 300);
            shoot_heat_count[0] = shoot_count; // shoot_cmd_recv.shoot_count;
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
            // DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            // DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            shoot_heat_count[0] = shoot_cmd_recv.shoot_count;
            // DM_enable_flag=0;
#endif
            shoot_heat_count[1] = shoot_heat_count[0];
            one_bullet          = 0;
            
            last_mode = LOAD_STOP;
            break;
        // 激活能量机关
        case LOAD_1_BULLET:

            // #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
            hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
            dead_time      = 150;
            if (shoot_cmd_recv.friction_mode == FRICTION_OFF) break;
            //    if (shoot_cmd_recv.Shoot_Once_Flag)
            //     {
            // 		DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
            //         DJIMotorSetRef(loader,shoot_cmd_recv.shoot_aim_angle);
            //     }
            // 	else
            // 	{
            // 		DJIMotorOuterLoop(loader, SPEED_LOOP);
            //         DJIMotorSetRef(loader, 0);
            // 	}
            //     break;

            // 速度环发射
            // shoot_heat_count[1] = shoot_count; // shoot_cmd_recv.shoot_count;
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
            // if (shoot_heat_count[1] - shoot_heat_count[0] >= 1) {
            //     one_bullet = 1;
            // }
            if(last_load_mode == LOAD_STOP && ShootLoaderCanStep(DWT_GetTimeline_ms()))
            {
                if((ShootLoaderTargetAngle(load_count - 1)
                - loader->measure.total_angle)<0)
                {
                    load_count++ ;
                    last_loader_feed_ms = DWT_GetTimeline_ms();
                }
            }
            DJIMotorSetRef(loader, ShootLoaderTargetAngle(load_count));
            // switch (one_bullet) {
            //     case 1:
            //         DJIMotorSetRef(loader, 5000);
            //         break;
            //     case 0:
            //         DJIMotorSetRef(loader, 5000);
            //         break;
            // }
            // break;
            
#endif
//             if (shoot_heat_count[1] - shoot_heat_count[0] >= 1) {
//                 one_bullet = 1;
//             }
//             switch (one_bullet) {
//                 case 1:
// #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
//                     // DJIMotorSetRef(loader, 0);
// #endif
// #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
//                     limit_speed = (limit_shoot_speed + (0 - limit_shoot_speed) * ramp_calc(&limit_off_ramp));
//                     ramp_init(&limit_on_ramp, 300);
//                     // DJIMotorSetRef(limit, limit_speed);
//                     // DJIMotorSetRef(limit, -20000);
//                     limit_shoot_speed = limit_speed;
// #endif
//                     break;
//                 case 0:
// #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)

//                     // 速度环
//                     // DJIMotorSetRef(loader, -load_speed);
//                     // 切换到角度环
//                     //     if (shoot_cmd_recv.Shoot_Once_Flag)
//                     // {
//                     // 	DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
//                     //     DJIMotorSetRef(loader,shoot_cmd_recv.shoot_aim_angle);
//                     // }
//                     // else
//                     // {
//                     // 	DJIMotorOuterLoop(loader, SPEED_LOOP);
//                     //     DJIMotorSetRef(loader, 0);
//                     // }

// #endif
// #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
//                     limit_speed = (limit_shoot_speed + (limit_speed_target - limit_shoot_speed) * ramp_calc(&limit_on_ramp));
//                     // ramp_init(&limit_off_ramp, 300);
//                     // DJIMotorSetRef(limit, limit_speed);
//                     limit_shoot_speed = limit_speed;
// #endif
//                     break;
//             }
            // last_mode = LOAD_1_BULLET;
            break;
        // 连发模式
        case LOAD_3_BULLET:
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
            hibernate_time = DWT_GetTimeline_ms();
            dead_time      = 150;
            if (shoot_cmd_recv.friction_mode == FRICTION_OFF) break;
            if(last_load_mode == LOAD_STOP && ShootLoaderCanStep(DWT_GetTimeline_ms()))
            {
                if((ShootLoaderTargetAngle(load_count - 1)
                - loader->measure.total_angle)<0)
                {
                    load_count += 3;
                    last_loader_feed_ms = DWT_GetTimeline_ms();
                }
            }
            DJIMotorSetRef(loader, ShootLoaderTargetAngle(load_count));
#endif
            break;
        case LOAD_BURSTFIRE:
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
            if (shoot_cmd_recv.friction_mode == FRICTION_OFF) break;
            // DJIMotorOuterLoop(loader, SPEED_LOOP);
            if((DWT_GetTimeline_ms() - load_time_ms) > cool_down_time)
            {
                if((ShootLoaderTargetAngle(load_count - 1)
                - loader->measure.total_angle)<0)
                {
                    load_count++ ;
                    last_loader_feed_ms = DWT_GetTimeline_ms();
                }
                load_time_ms = DWT_GetTimeline_ms();
            }
            DJIMotorSetRef(loader, ShootLoaderTargetAngle(load_count));
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
#endif
            break;

        case LOAD_REVERSE:
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
            // DJIMotorOuterLoop(loader, SPEED_LOOP);
            // DJIMotorSetRef(loader, -20000);
            DJIMotorSetRef(loader,loader->measure.total_angle - SHOOT_LOADER_ANGLE_PER_BULLET );
            
// x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
#endif
            break;
        default:
            while (1); // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        fric_speed  = (shoot_speed + (shoot_speed_target - shoot_speed) * ramp_calc(&fric_on_ramp));
        fric2_speed = (shoot2_speed + (shoot2_speed_target - shoot2_speed) * ramp_calc(&fric2_on_ramp));
        ramp_init(&fric_off_ramp, 3000);
        ramp_init(&fric2_off_ramp, 3000);
        Laser_on();
    } else if (shoot_cmd_recv.friction_mode == FRICTION_OFF) {
        fric_speed  = (shoot_speed + (0 - shoot_speed) * ramp_calc(&fric_off_ramp));
        fric2_speed = (shoot2_speed + (0 - shoot2_speed) * ramp_calc(&fric2_off_ramp));
        ramp_init(&fric_on_ramp, 8000);
        ramp_init(&fric2_on_ramp, 8000);
        Laser_off();
    }

    DJIMotorSetRef(friction_l, fric_speed);
    // DJIMotorSetRef(friction_r, fric_speed);
    // DJIMotorSetRef(friction_up, fric_speed);
    // DJIMotorSetRef(friction_l2, fric2_speed);
    // DJIMotorSetRef(friction_r2, fric2_speed);
    DJIMotorSetRef(friction_r, fric2_speed);
    shoot_speed  = fric_speed;
    shoot2_speed = fric2_speed;
#endif
    last_load_mode = shoot_cmd_recv.load_mode;
    // 反馈数据
    memcpy(&shoot_feedback_data.shooter_local_heat, &local_heat, sizeof(float));
    memcpy(&shoot_feedback_data.shooter_heat_control, &heat_control, sizeof(int));
    // memcpy(&shoot_feedback_data.loader_angle, &loader->measure.total_angle, sizeof(float));
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM14) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM6) {
        /*-------------------------------------------热量控制部分---------------------------------------------*/
        local_heat -= (shoot_cmd_recv.shooter_heat_cooling_rate / 1000.0f); // 1000Hz冷却
        if (local_heat < 0) {
            local_heat = 0;
        }
        // local_heat = 100 ;
        if (shoot_cmd_recv.shooter_referee_heat - shoot_cmd_recv.shooter_cooling_limit >= 15) // 裁判系统判断已经超了热量
        {
            local_heat = shoot_cmd_recv.shooter_referee_heat;
        }
        Shoot_Fric_data_process();
    }
    /* USER CODE END Callback 1 */
}
