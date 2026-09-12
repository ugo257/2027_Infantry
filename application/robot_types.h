#pragma once
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

/* Forward declarations to avoid depending on low-level headers here */
typedef struct INS_Instance INS_Instance;

typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

typedef enum {
    CHASSIS_ZERO_FORCE = 0,
    CHASSIS_ROTATE,
    CHASSIS_NO_FOLLOW,
    CHASSIS_FOLLOW_GIMBAL_YAW,
    CHASSIS_REVERSE_ROTATE,
    CHASSIS_CLIMB,
    CHASSIS_CLIMB_RETRACT,
    CHASSIS_CLIMB_WITH_PUSH,
    CHASSIS_CLIMB_WITH_PULL,
    CHASSIS_MECANUM_FORCE,
    CHASSIS_FLY_SLOPE,
} chassis_mode_e;

typedef enum {
    LEG_ACTIVE_SUSPENSION = 0,
    LEG_CLIMB,
    LEG_CLIMB_RETRACT,
} leg_mode_e;

typedef enum {
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_GYRO_MODE,
    GIMBAL_MOTOR_MODE,
} gimbal_mode_e;

typedef enum {
    none_version_control = 0,
    version_control,
} nuc_mode_e;

typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;

typedef enum {
    FRICTION_OFF = 0,
    FRICTION_ON,
} friction_mode_e;

typedef enum {
    LOAD_STOP = 0,
    LOAD_REVERSE,
    LOAD_1_BULLET,
    LOAD_BURSTFIRE,
    LOAD_3_BULLET,
} loader_mode_e;

typedef enum {
    LOAD_UNINIT = 0,
    LOAD_REINIT,
    LOAD_INFRARED_INIT,
} loader_state_e;

typedef enum {
    SUPERCAP_UNUSE = 0,
    SUPERCAP_USE,
} SuperCap_Mode_e;

typedef struct {
    uint32_t send_ID;
    uint32_t receive_ID;
    uint8_t error_state;
    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;
    float PMAX;
    float VMAX;
    float TMAX;
    float pos;
    float vel;
    float tor;
    uint8_t MOS_temperature;
    uint8_t temperature;
} DM4310_t;

extern DM4310_t DM4310;

#pragma pack(1)

typedef struct
{
    uint16_t power_buffer;
    float chassis_power;
    uint8_t level;
    uint16_t power_limit;
    uint8_t SuperCap_flag_from_user;
    float vx;
    float vy;
    float wz;
    float offset_angle;
    float gimbal_error_angle;
    chassis_mode_e chassis_mode;
    float vw_set;
    float wz_K;
    float leg_length_cmd;
    uint8_t mecanum_force_enable;
    int8_t sync_belt_cmd;
} Chassis_Ctrl_Cmd_s;

typedef struct
{
    uint16_t power_buffer;
    float chassis_power;
    uint8_t level;
    uint16_t power_limit;
    uint8_t SuperCap_flag_from_user;
    uint16_t vx;
    uint16_t vy;
    uint16_t wz;
    uint16_t offset_angle;
    uint16_t gimbal_error_angle;
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s_half_float;

typedef struct
{
    float yaw;
    float pitch;
    float yaw_version;
    float pitch_version;
    float yaw_speedFeed;
    float yaw_kp;
    float yaw_kd;
    float yaw_speedKp;
    gimbal_mode_e gimbal_mode;
    nuc_mode_e nuc_mode;
    uint8_t ifSpeedFeed;
} Gimbal_Ctrl_Cmd_s;

typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    loader_mode_e load_mode_last;
    friction_mode_e friction_mode;
    uint8_t rest_heat;
    uint16_t shooter_heat_cooling_rate;
    uint16_t shooter_referee_heat;
    uint16_t shooter_cooling_limit;
    float shoot_rate;
    float bullet_speed;
    float shoot_aim_angle;
    uint8_t Shoot_Once_Flag;
    int32_t shoot_count;
} Shoot_Ctrl_Cmd_s;

typedef struct
{
    uint8_t ui_send_flag;
    chassis_mode_e chassis_mode;
    uint16_t chassis_attitude_angle;
    friction_mode_e friction_mode;
    uint8_t rune_mode;
    uint8_t SuperCap_mode;
    float supercap_voltage;
    float Chassis_Ctrl_power;
    uint8_t cap_online_flag;
    uint16_t Chassis_power_limit;
    uint16_t Shooter_heat;
    uint8_t robot_level;
    uint16_t Heat_Limit;
    uint16_t nuc_flag;
    int8_t sync_belt_state;
    uint8_t vision_work_mode;
    uint8_t climb_mode;
} UI_Cmd_s;

typedef struct
{
    float real_vx;
    float real_vy;
    float real_wz;
    uint8_t cap_online_flag;
    float cap_voltage;
    uint8_t cap_energy;
    uint16_t capget_power_limit;
    float chassis_real_power;
    float chassis_power_output;
    float chassis_voltage;
    float chassis_imu_data[3];
    float chassis_gyro[3];
    float chassis_accel[3];
    float chassis_pitch;
    float chassis_cmd_vx;
    float chassis_cmd_vy;
    float chassis_cmd_wz;
    float chassis_body_vx_cmd;
    float chassis_body_vy_cmd;
    float wheel_ref[4];
    float wheel_pid_output[4];
    float wheel_post_limit_current[4];
    float wheel_real_current[4];
    float wheel_speed_aps[4];
    uint8_t wheel_online_flag[4];
    float leg_length_l;
    float leg_length_r;
    float leg_length_avg;
    float leg_length_target;
    float leg_angle_l;
    float leg_angle_r;
    float leg_angle_target_l;
    float leg_angle_target_r;
    float leg_joint_pos_l;
    float leg_joint_pos_r;
    float leg_joint_vel_l;
    float leg_joint_vel_r;
    float leg_joint_torque_l;
    float leg_joint_torque_r;
    float leg_joint_torque_l_unified;
    float leg_joint_torque_r_unified;
    float leg_joint_torque_avg;
    float leg_torque_ff_l;
    float leg_torque_ff_r;
    float leg_id_torque;
    float leg_fit_torque_ff;
    float leg_fit_length_dot;
    uint8_t leg_id_active;
    uint8_t leg_id_phase;
} Chassis_Upload_Data_s;

typedef struct
{
    uint16_t yaw;
} Chassis_Upload_Data_s_half_float;

typedef struct
{
    INS_Instance *gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    uint16_t yaw_ecd;
    uint16_t pitch_ecd;
    float yaw_total_angle;
    float pitch_total_angle;
    float yaw_angle_pidout;
    float yaw_angleP;
    float yaw_angleD;
    float yaw_speedP;
    float yaw_motor_real_current;
    float speedFeed;
} Gimbal_Upload_Data_s;

typedef struct
{
    int shooter_heat_control;
    float shooter_local_heat;
    float loader_angle;
} Shoot_Upload_Data_s;

typedef enum
{
    red = 0,
    blue,
} version_color;

typedef struct
{
    float vx;
    float vy;
    float wz;
    float yaw_control;
    float yaw_angle;
    float yaw_gyro;
    float offset_angle;
    float gimbal_error_angle;
    int32_t shoot_count;
    float nuc_yaw;
    float yaw_vel;              // 视觉提供的yaw速度（用于自瞄前馈）
    float yaw_acc;              // vision yaw reference acceleration, deg/s^2
    float leg_length_cmd;
    gimbal_mode_e gimbal_mode;
    chassis_mode_e chassis_mode;
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    nuc_mode_e nuc_mode;
    uint8_t reset_flag;
    uint8_t UI_SendFlag;
    uint8_t superCap_flag;
    int8_t sync_belt_cmd;
    uint8_t vision_work_mode;
    uint8_t yaw_test_double_up;
} Chassis_Ctrl_Cmd_s_uart;

typedef struct
{
    float yaw_motor_single_round_angle;
    float yaw_total_angle;
    float yaw_ecd;
    float chassis_pitch_angle;
    float chassis_yaw_gyro;
    float initial_speed;
    float yaw_motor_real_current;
    float yaw_angle_pidout;
    version_color color;
} Chassis_Upload_Data_s_uart;

typedef struct
{
    uint8_t eee;
} UI_Upload_Data_s;

#pragma pack()

#define Chassis_Ctrl_Cmd_s_uart_size   sizeof(Chassis_Ctrl_Cmd_s_uart)
#define Chassis_Upload_Data_s_uart_size sizeof(Chassis_Upload_Data_s_uart)

extern uint8_t nuc_rx[8];

#endif // ROBOT_TYPES_H
