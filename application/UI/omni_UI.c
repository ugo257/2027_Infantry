// 应用层头文件
#include "robot_board.h"
#include "robot_types.h"
#include "omni_UI.h"
// 模块头文件
#include "rm_referee.h"
#include "referee_protocol.h"
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "message_center.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "UI_interface.h"

// 视觉工作模式定义
#define UI_VISION_WORK_IDLE        0u   // 视觉工作模式：空闲
#define UI_VISION_WORK_AIM         1u   // 视觉工作模式：瞄准
#define UI_VISION_WORK_SMALL_RUNE  2u   // 视觉工作模式：小能量机关
#define UI_VISION_WORK_BIG_RUNE    3u   // 视觉工作模式：大能量机关
#define UI_SHOOT_RETICLE_Y_OFFSET  -50u // 射击准星Y轴偏移量
#define UI_SHOOT_RETICLE_X         (SCREEN_LENGTH / 2u)  // 射击准星X坐标（屏幕中心）
#define UI_SHOOT_RETICLE_Y         (SCREEN_WIDTH / 2u + UI_SHOOT_RETICLE_Y_OFFSET)  // 射击准星Y坐标（屏幕中心偏移）
#define UI_SHOOT_ARC_RADIUS_X      383u // 射击弧形半径X
#define UI_SHOOT_ARC_RADIUS_Y      386u // 射击弧形半径Y
#define UI_STATE_CIRCLE_X          1480u // 状态指示圆X坐标
#define UI_STATE_LABEL_X           1510u // 状态文本标签X坐标
#define UI_STATE_START_Y           420u  // 状态指示起始Y坐标
#define UI_STATE_Y_STEP            45u   // 状态指示Y方向间距
#define UI_STATE_LABEL_SIZE        15u   // 状态文本字号
#define UI_STATE_LABEL_WIDTH       2u    // 状态文本线宽
#define UI_STATE_CIRCLE_SIZE       7u    // 主要状态指示圆半径
#define UI_STATE_CIRCLE_SMALL_SIZE 6u    // 次要状态指示圆半径
#define UI_STATE_Y(index)          (UI_STATE_START_Y + UI_STATE_Y_STEP * (index))  // 状态项Y坐标


// UI模块消息发布者指针
static Publisher_t *ui_pub;
// UI模块消息订阅者指针
static Subscriber_t *ui_sub;
// UI命令接收结构体
static UI_Cmd_s ui_cmd_recv;
// UI上传数据结构体
static UI_Upload_Data_s ui_feedback_data;

// 调试用超级电容电压变量
float cap_debug;
// 调试用热量变量
float heat_debug;
// 外部声明的差值变量
extern float diff2;
// 裁判系统数据指针
referee_info_t *referee_data_for_ui;

// UI能量机关变量
uint8_t UI_rune;
// UI序列号
uint8_t UI_Seq;

// 射击准星线条数据数组
static Graph_Data_t shoot_line[5];
// 基准线图形数据数组
static Graph_Data_t benchmark[5];
// 状态圆圈数据数组
static Graph_Data_t state_circle[10];
// 状态字符串数据数组
static String_Data_t Char_State[10];
// 数值标签字符串数据数组
static String_Data_t Value_Label[3];
// 电容电压弧形数据
static Graph_Data_t Cap_voltage_arc;
// 偏差弧形数据
static Graph_Data_t Deviation_arc;
// 允许射击热量弧形数据
static Graph_Data_t Allow_heat_arc;
// 电容电压数据显示
static Graph_Data_t Cap_voltage;
// 总电压数据显示
static Graph_Data_t total_voltage;
// 射击局部热量数据显示
static Graph_Data_t Shoot_Local_Heat;

// 电容电压、总热量、热量临时变量
float capV, totalheat, heattemp;

// 获取总热量函数，根据机器人等级返回对应的总热量值
static float getTotal_heat(uint8_t level)
{
    switch (level) {
        case 1: return 100.0f;
        case 2: return 140.0f;
        case 3: return 180.0f;
        case 4: return 220.0f;
        case 5: return 260.0f;
        case 6: return 300.0f;
        case 7: return 340.0f;
        case 8: return 380.0f;
        case 9: return 420.0f;
        case 10: return 500.0f;
        default: return 100.0f;
    }
}

// 允许射击热量函数，根据总热量和当前热量返回允许射击热量值
static float allow_shoot_heat(float total, uint16_t temp)
{
    return total - (float)temp;
}

// 限制值函数，将输入值限制在指定范围内
static float clampf_ui(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static void UI_StaticInit(void)
{
    if (referee_data_for_ui == NULL)
        return;  // 如果裁判系统数据为空，则直接返回

    const uint32_t reticle_x = UI_SHOOT_RETICLE_X;  // 准星X坐标
    const uint32_t reticle_y = UI_SHOOT_RETICLE_Y;  // 准星Y坐标

    // 删除所有已存在的UI数据
    UIDelete(&referee_data_for_ui->referee_id, UI_Data_Del_ALL, 0);

    // 绘制射击准星十字线
    UILineDraw(&shoot_line[0], "l0", UI_Graph_ADD, 9, UI_Color_Green, 2, reticle_x - 45, reticle_y, reticle_x - 12, reticle_y);  // 左横线
    UILineDraw(&shoot_line[1], "l1", UI_Graph_ADD, 9, UI_Color_Green, 2, reticle_x + 12, reticle_y, reticle_x + 45, reticle_y);  // 右横线
    UILineDraw(&shoot_line[2], "l2", UI_Graph_ADD, 9, UI_Color_Green, 2, reticle_x, reticle_y - 45, reticle_x, reticle_y - 12);  // 上竖线
    UILineDraw(&shoot_line[3], "l3", UI_Graph_ADD, 9, UI_Color_Green, 2, reticle_x, reticle_y + 12, reticle_x, reticle_y + 45);  // 下竖线
    UICircleDraw(&shoot_line[4], "l4", UI_Graph_ADD, 9, UI_Color_White, 1, reticle_x, reticle_y, 6);  // 中心圆点
    // 绘制基准线（边界参考线）
    UILineDraw(&benchmark[0], "b0", UI_Graph_ADD, 8, UI_Color_White, 10, reticle_x - UI_SHOOT_ARC_RADIUS_X, reticle_y - 10, reticle_x - UI_SHOOT_ARC_RADIUS_X, reticle_y + 10);  // 左边界线
    UILineDraw(&benchmark[1], "b1", UI_Graph_ADD, 9, UI_Color_White, 10, reticle_x + UI_SHOOT_ARC_RADIUS_X, reticle_y - 10, reticle_x + UI_SHOOT_ARC_RADIUS_X, reticle_y + 10);  // 右边界线
    UIArcDraw(&benchmark[2], "b2", UI_Graph_ADD, 7, UI_Color_White, 313, 316, 10, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);  // 右上角弧线
    UIArcDraw(&benchmark[3], "b3", UI_Graph_ADD, 6, UI_Color_White, 223, 226, 10, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);  // 左上角弧线
    UIArcDraw(&benchmark[4], "b4", UI_Graph_ADD, 5, UI_Color_White, 44, 47, 10, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);    // 左下角弧线

    // 绘制状态文本标签
    UICharDraw(&Char_State[0], "t0", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(0), "Rotate");   // 旋转模式标签
    UICharDraw(&Char_State[1], "t1", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(1), "Climb");    // 攀爬模式标签
    UICharDraw(&Char_State[2], "t2", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(2), "Friction"); // 摩擦轮模式标签
    UICharDraw(&Char_State[3], "t3", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(3), "Cap");      // 电容模式标签
    UICharDraw(&Char_State[4], "t4", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(4), "Belt+");    // 皮带正转标签
    UICharDraw(&Char_State[5], "t5", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(5), "Belt-");    // 皮带反转标签
    UICharDraw(&Char_State[6], "t6", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(6), "Belt0");    // 皮带停止标签
    UICharDraw(&Char_State[7], "t7", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(7), "Aim");      // 瞄准模式标签
    UICharDraw(&Char_State[8], "t8", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(8), "Small");    // 小能量机关标签
    UICharDraw(&Char_State[9], "t9", UI_Graph_ADD, 7, UI_Color_Orange, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, UI_STATE_LABEL_X, UI_STATE_Y(9), "Big");      // 大能量机关标签
    // 刷新所有字符显示
    for (uint8_t i = 0; i < 10; i++)
        UICharRefresh(&referee_data_for_ui->referee_id, Char_State[i]);

    // 绘制状态指示圆圈
    UICircleDraw(&state_circle[0], "c0", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(0), UI_STATE_CIRCLE_SIZE);  // 旋转模式指示
    UICircleDraw(&state_circle[1], "c1", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(1), UI_STATE_CIRCLE_SIZE);  // 攀爬模式指示
    UICircleDraw(&state_circle[2], "c2", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(2), UI_STATE_CIRCLE_SIZE);  // 摩擦轮模式指示
    UICircleDraw(&state_circle[3], "c3", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(3), UI_STATE_CIRCLE_SIZE);  // 电容状态指示
    UICircleDraw(&state_circle[4], "c4", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(4), UI_STATE_CIRCLE_SMALL_SIZE);     // 皮带正转指示
    UICircleDraw(&state_circle[5], "c5", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(5), UI_STATE_CIRCLE_SMALL_SIZE);     // 皮带反转指示
    UICircleDraw(&state_circle[6], "c6", UI_Graph_ADD, 9, UI_Color_Green, UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(6), UI_STATE_CIRCLE_SMALL_SIZE);     // 皮带停止指示
    UICircleDraw(&state_circle[7], "c7", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(7), UI_STATE_CIRCLE_SMALL_SIZE);     // 瞄准模式指示
    UICircleDraw(&state_circle[8], "c8", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(8), UI_STATE_CIRCLE_SMALL_SIZE);     // 小能量机关指示
    UICircleDraw(&state_circle[9], "c9", UI_Graph_ADD, 9, UI_Color_White, UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(9), UI_STATE_CIRCLE_SMALL_SIZE);     // 大能量机关指示

    // 绘制各种弧形和数值显示
    UIArcDraw(&Cap_voltage_arc, "pow", UI_Graph_ADD, 9, UI_Color_Pink, 271, 273, 5, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);      // 电容电压弧形
    UIFloatDraw(&Cap_voltage, "of5", UI_Graph_ADD, 9, UI_Color_Pink, UI_STATE_LABEL_SIZE, 3, UI_STATE_LABEL_WIDTH, 430, 560, (int32_t)(ui_cmd_recv.supercap_voltage * 1000.0f));  // 电容电压数值
    UIFloatDraw(&total_voltage, "of0", UI_Graph_ADD, 9, UI_Color_Green, UI_STATE_LABEL_SIZE, 0, UI_STATE_LABEL_WIDTH, 900, 660, 24000);      // 总电压数值
    UIArcDraw(&Allow_heat_arc, "hea", UI_Graph_ADD, 9, UI_Color_Green, 273, 300, 5, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);  // 允许射击热量弧形
    UIFloatDraw(&Shoot_Local_Heat, "of3", UI_Graph_ADD, 7, UI_Color_Green, UI_STATE_LABEL_SIZE, 3, UI_STATE_LABEL_WIDTH, 1440, 560, (ui_cmd_recv.Shooter_heat) * 1000);  // 局部射击热量
    UIArcDraw(&Deviation_arc, "dev", UI_Graph_ADD, 4, UI_Color_Pink, 8, 43, 5, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);      // 偏差弧形

    // 绘制数值文本标签
    UICharDraw(&Value_Label[0], "v0", UI_Graph_ADD, 7, UI_Color_Pink, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, 360, 560, "Cap");
    UICharDraw(&Value_Label[1], "v1", UI_Graph_ADD, 7, UI_Color_Green, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, 1360, 560, "Heat");
    UICharDraw(&Value_Label[2], "v2", UI_Graph_ADD, 7, UI_Color_Green, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, 820, 660, "Volt");
    // 刷新数值文本标签
    for (uint8_t i = 0; i < 3; i++)
        UICharRefresh(&referee_data_for_ui->referee_id, Value_Label[i]);

    // 批量刷新UI图形
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, shoot_line[0], shoot_line[1], shoot_line[2], shoot_line[3], shoot_line[4]);  // 刷新射击准星
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, state_circle[0], state_circle[1], state_circle[2], state_circle[3], state_circle[4], state_circle[5], state_circle[6]);  // 刷新前7个状态圆
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, state_circle[7], state_circle[8], state_circle[9], Cap_voltage, Shoot_Local_Heat);  // 刷新后5个元素
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, Cap_voltage_arc, Allow_heat_arc, benchmark[0], benchmark[1], benchmark[2], benchmark[3], benchmark[4]);  // 刷新弧形和基准线
    UIGraphRefresh(&referee_data_for_ui->referee_id, 2, total_voltage, Deviation_arc);  // 刷新总电压和偏差
}

void UI_Init(void)
{
    ui_pub = PubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    ui_sub = SubRegister("ui_cmd", sizeof(UI_Cmd_s));

    UI_StaticInit();
}

/*
 * UI动态刷新函数
 * 实时更新UI界面，根据机器人状态动态改变UI元素的颜色和数值
 */
void UIDynamicRefresh(void)
{
    if (referee_data_for_ui == NULL)
        return;  // 如果裁判系统数据为空，则直接返回

    const uint32_t reticle_x = UI_SHOOT_RETICLE_X;  // 准星X坐标
    const uint32_t reticle_y = UI_SHOOT_RETICLE_Y;  // 准星Y坐标

    // 获取UI命令消息
    SubGetMessage(ui_sub, (void *)&ui_cmd_recv);
    if (ui_cmd_recv.ui_send_flag == 0) {  // 如果UI发送标志为0，则重新初始化静态UI
        UI_StaticInit();
    }

    // 更新底盘模式指示（旋转模式）
    UICircleDraw(&state_circle[0], "c0", UI_Graph_Change, 9,
                 (ui_cmd_recv.chassis_mode == CHASSIS_ROTATE) ? UI_Color_Main : UI_Color_White,  // 旋转模式激活时显示主色，否则白色
                 UI_STATE_CIRCLE_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(0), UI_STATE_CIRCLE_SIZE);
    // 更新攀爬模式指示
    UICircleDraw(&state_circle[1], "c1", UI_Graph_Change, 9,
                 ui_cmd_recv.climb_mode ? UI_Color_Cyan : UI_Color_White,  // 攀爬模式激活时显示青色，否则白色
                 UI_STATE_CIRCLE_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(1), UI_STATE_CIRCLE_SIZE);
    // 更新摩擦轮模式指示
    UICircleDraw(&state_circle[2], "c2", UI_Graph_Change, 9,
                 (ui_cmd_recv.friction_mode == FRICTION_ON) ? UI_Color_Main : UI_Color_White,  // 摩擦轮开启时显示主色，否则白色
                 UI_STATE_CIRCLE_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(2), UI_STATE_CIRCLE_SIZE);
    // 更新电容在线状态指示
    UICircleDraw(&state_circle[3], "c3", UI_Graph_Change, 9,
                 (ui_cmd_recv.cap_online_flag == 0u) ? UI_Color_Pink : UI_Color_Green,  // 电容离线时显示粉色，否则绿色
                 UI_STATE_CIRCLE_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(3), UI_STATE_CIRCLE_SIZE);

    // 更新同步皮带状态指示
    UICircleDraw(&state_circle[4], "c4", UI_Graph_Change, 9,
                 (ui_cmd_recv.sync_belt_state > 0) ? UI_Color_Green : UI_Color_White,  // 皮带正转时显示绿色，否则白色
                 UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(4), UI_STATE_CIRCLE_SMALL_SIZE);
    UICircleDraw(&state_circle[5], "c5", UI_Graph_Change, 9,
                 (ui_cmd_recv.sync_belt_state < 0) ? UI_Color_Orange : UI_Color_White,  // 皮带反转时显示橙色，否则白色
                 UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(5), UI_STATE_CIRCLE_SMALL_SIZE);
    UICircleDraw(&state_circle[6], "c6", UI_Graph_Change, 9,
                 (ui_cmd_recv.sync_belt_state == 0) ? UI_Color_Green : UI_Color_White,  // 皮带停止时显示绿色，否则白色
                 UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(6), UI_STATE_CIRCLE_SMALL_SIZE);

    // 处理视觉工作模式
    uint8_t vision_mode = ui_cmd_recv.vision_work_mode;
    if (vision_mode == UI_VISION_WORK_IDLE && ui_cmd_recv.nuc_flag)  // 如果是空闲模式但NUC标志有效，则改为瞄准模式
        vision_mode = UI_VISION_WORK_AIM;
    // 更新瞄准模式指示
    UICircleDraw(&state_circle[7], "c7", UI_Graph_Change, 9,
                 (vision_mode == UI_VISION_WORK_AIM) ? UI_Color_Green : UI_Color_White,  // 瞄准模式激活时显示绿色，否则白色
                 UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(7), UI_STATE_CIRCLE_SMALL_SIZE);
    // 更新小能量机关模式指示
    UICircleDraw(&state_circle[8], "c8", UI_Graph_Change, 9,
                 (vision_mode == UI_VISION_WORK_SMALL_RUNE) ? UI_Color_Green : UI_Color_White,  // 小能量机关模式激活时显示绿色，否则白色
                 UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(8), UI_STATE_CIRCLE_SMALL_SIZE);
    // 更新大能量机关模式指示
    UICircleDraw(&state_circle[9], "c9", UI_Graph_Change, 9,
                 (vision_mode == UI_VISION_WORK_BIG_RUNE) ? UI_Color_Green : UI_Color_White,  // 大能量机关模式激活时显示绿色，否则白色
                 UI_STATE_CIRCLE_SMALL_SIZE, UI_STATE_CIRCLE_X, UI_STATE_Y(9), UI_STATE_CIRCLE_SMALL_SIZE);

    // 处理超级电容电压显示
    float cap_ratio = (ui_cmd_recv.supercap_voltage - SUPERCAP_MIN_VOLTAGE) /
                      (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE);  // 计算电容电压比例
    cap_ratio = clampf_ui(cap_ratio, 0.0f, 1.0f);  // 限制电压比例在0到1之间
    uint32_t cap_color = UI_Color_Pink;  // 默认颜色为粉色
    if (ui_cmd_recv.supercap_voltage >= SUPERCAP_HIGHER_THRESHOLD_VOLTAGE)  // 如果电压高于高阈值
        cap_color = UI_Color_Green;  // 显示绿色
    else if (ui_cmd_recv.supercap_voltage >= SUPERCAP_LOWER_THRESHOLD_VOLTAGE)  // 如果电压高于低阈值
        cap_color = UI_Color_Orange;  // 显示橙色
    // 更新电容电压弧形显示，角度限制在右上基准线内
    UIArcDraw(&Cap_voltage_arc, "pow", UI_Graph_Change, 9, cap_color,
              271, 272 + (uint32_t)(40.0f * cap_ratio), 5, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);
    // 更新电容数值标签颜色
    UICharDraw(&Value_Label[0], "v0", UI_Graph_Change, 7, cap_color, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, 360, 560, "Cap");
    UICharRefresh(&referee_data_for_ui->referee_id, Value_Label[0]);
    // 更新电容电压数值显示
    UIFloatDraw(&Cap_voltage, "of5", UI_Graph_Change, 9, cap_color, UI_STATE_LABEL_SIZE, 3, UI_STATE_LABEL_WIDTH, 430, 560,
                (int32_t)(ui_cmd_recv.supercap_voltage * 1000.0f));

    // 处理射击热量显示
    totalheat = (ui_cmd_recv.Heat_Limit > 0u) ? (float)ui_cmd_recv.Heat_Limit :  // 如果有热量限制则使用该限制
                getTotal_heat(ui_cmd_recv.robot_level);  // 否则根据机器人等级获取最大热量
    heattemp = clampf_ui(allow_shoot_heat(totalheat, ui_cmd_recv.Shooter_heat), 0.0f, totalheat);  // 计算剩余可射击热量并限幅
    uint32_t heat_color = (heattemp > 99.0f) ? UI_Color_Green : UI_Color_Purplish_red;  // 剩余热量充足时显示绿色，不足时显示紫红色
    uint32_t heat_start = 266u - (uint32_t)(3.8f * 0.1f * heattemp / 5.0f);  // 计算热量弧形起始角度
    // 更新热量数值标签颜色
    UICharDraw(&Value_Label[1], "v1", UI_Graph_Change, 7, heat_color, UI_STATE_LABEL_SIZE, UI_STATE_LABEL_WIDTH, 1360, 560, "Heat");
    UICharRefresh(&referee_data_for_ui->referee_id, Value_Label[1]);
    // 更新射击局部热量数值显示
    UIFloatDraw(&Shoot_Local_Heat, "of3", UI_Graph_Change, 7, heat_color, UI_STATE_LABEL_SIZE, 3, UI_STATE_LABEL_WIDTH, 1440, 560,
                (int32_t)(1000.0f * heattemp));
    // 更新允许射击热量弧形显示
    UIArcDraw(&Allow_heat_arc, "hea", UI_Graph_Change, 9, heat_color,
              heat_start, 266, 5, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, 392);
    // 更新偏差弧形显示
    UIArcDraw(&Deviation_arc, "dev", UI_Graph_Change, 4, UI_Color_Pink, 8, 43, 5, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);

    // 批量刷新UI图形
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, state_circle[0], state_circle[1], state_circle[2], state_circle[3], state_circle[4], state_circle[5], state_circle[6]);  // 刷新前7个状态圆
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, state_circle[7], state_circle[8], state_circle[9], Cap_voltage, Shoot_Local_Heat);  // 刷新后5个元素
    UIGraphRefresh(&referee_data_for_ui->referee_id, 2, Cap_voltage_arc, Allow_heat_arc);  // 刷新电容电压和允许射击热量弧形
    UIGraphRefresh(&referee_data_for_ui->referee_id, 1, Deviation_arc);  // 刷新偏差弧形

    // 发布UI反馈消息
    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}
