// app
#include "robot_board.h"
#include "robot_types.h"
#include "omni_UI.h"
// module
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

#define UI_VISION_WORK_IDLE        0u
#define UI_VISION_WORK_AIM         1u
#define UI_VISION_WORK_SMALL_RUNE  2u
#define UI_VISION_WORK_BIG_RUNE    3u
#define UI_SHOOT_RETICLE_Y_OFFSET  -50u
#define UI_SHOOT_RETICLE_X         (SCREEN_LENGTH / 2u)
#define UI_SHOOT_RETICLE_Y         (SCREEN_WIDTH / 2u + UI_SHOOT_RETICLE_Y_OFFSET)
#define UI_SHOOT_ARC_RADIUS_X      383u
#define UI_SHOOT_ARC_RADIUS_Y      386u

static Publisher_t *ui_pub;
static Subscriber_t *ui_sub;
static UI_Cmd_s ui_cmd_recv;
static UI_Upload_Data_s ui_feedback_data;

float cap_debug;
float heat_debug;
extern float diff2;
referee_info_t *referee_data_for_ui;

uint8_t UI_rune;
uint8_t UI_Seq;

static Graph_Data_t shoot_line[7];
static Graph_Data_t benchmark[5];
static Graph_Data_t state_circle[10];
static String_Data_t Char_State[10];
static Graph_Data_t Cap_voltage_arc;
static Graph_Data_t Deviation_arc;
static Graph_Data_t Allow_heat_arc;
static Graph_Data_t Cap_voltage;
static Graph_Data_t total_voltage;
static Graph_Data_t Shoot_Local_Heat;

float capV, totalheat, heattemp;

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

static float allow_shoot_heat(float total, uint16_t temp)
{
    return total - (float)temp;
}

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
        return;

    const uint32_t reticle_x = UI_SHOOT_RETICLE_X;
    const uint32_t reticle_y = UI_SHOOT_RETICLE_Y;

    UIDelete(&referee_data_for_ui->referee_id, UI_Data_Del_ALL, 0);

    UILineDraw(&shoot_line[0], "l0", UI_Graph_ADD, 9, UI_Color_Green, 2, reticle_x - 45, reticle_y, reticle_x - 12, reticle_y);
    UILineDraw(&shoot_line[1], "l1", UI_Graph_ADD, 9, UI_Color_Green, 2, reticle_x + 12, reticle_y, reticle_x + 45, reticle_y);
    UILineDraw(&shoot_line[2], "l2", UI_Graph_ADD, 9, UI_Color_Green, 2, reticle_x, reticle_y - 45, reticle_x, reticle_y - 12);
    UILineDraw(&shoot_line[3], "l3", UI_Graph_ADD, 9, UI_Color_Green, 2, reticle_x, reticle_y + 12, reticle_x, reticle_y + 45);
    UICircleDraw(&shoot_line[4], "l4", UI_Graph_ADD, 9, UI_Color_White, 1, reticle_x, reticle_y, 6);
    UILineDraw(&shoot_line[5], "l5", UI_Graph_ADD, 9, UI_Color_White, 2, reticle_x - 90, reticle_y + 75, reticle_x - 35, reticle_y + 75);
    UILineDraw(&shoot_line[6], "l6", UI_Graph_ADD, 9, UI_Color_White, 2, reticle_x + 35, reticle_y + 75, reticle_x + 90, reticle_y + 75);

    UILineDraw(&benchmark[0], "b0", UI_Graph_ADD, 8, UI_Color_White, 10, reticle_x - UI_SHOOT_ARC_RADIUS_X, reticle_y - 10, reticle_x - UI_SHOOT_ARC_RADIUS_X, reticle_y + 10);
    UILineDraw(&benchmark[1], "b1", UI_Graph_ADD, 9, UI_Color_White, 10, reticle_x + UI_SHOOT_ARC_RADIUS_X, reticle_y - 10, reticle_x + UI_SHOOT_ARC_RADIUS_X, reticle_y + 10);
    UIArcDraw(&benchmark[2], "b2", UI_Graph_ADD, 7, UI_Color_White, 313, 316, 10, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);
    UIArcDraw(&benchmark[3], "b3", UI_Graph_ADD, 6, UI_Color_White, 223, 226, 10, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);
    UIArcDraw(&benchmark[4], "b4", UI_Graph_ADD, 5, UI_Color_White, 44, 47, 10, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);

    UICharDraw(&Char_State[0], "t0", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 580, 125, "Rotate");
    UICharDraw(&Char_State[1], "t1", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 720, 125, "Climb");
    UICharDraw(&Char_State[2], "t2", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 880, 125, "Friction");
    UICharDraw(&Char_State[3], "t3", UI_Graph_ADD, 7, UI_Color_Orange, 23, 4, 1270, 125, "Cap");
    UICharDraw(&Char_State[4], "t4", UI_Graph_ADD, 7, UI_Color_Orange, 18, 3, 520, 220, "Belt+");
    UICharDraw(&Char_State[5], "t5", UI_Graph_ADD, 7, UI_Color_Orange, 18, 3, 620, 220, "Belt-");
    UICharDraw(&Char_State[6], "t6", UI_Graph_ADD, 7, UI_Color_Orange, 18, 3, 720, 220, "Belt0");
    UICharDraw(&Char_State[7], "t7", UI_Graph_ADD, 7, UI_Color_Orange, 18, 3, 900, 220, "Aim");
    UICharDraw(&Char_State[8], "t8", UI_Graph_ADD, 7, UI_Color_Orange, 18, 3, 980, 220, "Small");
    UICharDraw(&Char_State[9], "t9", UI_Graph_ADD, 7, UI_Color_Orange, 18, 3, 1085, 220, "Big");
    for (uint8_t i = 0; i < 10; i++)
        UICharRefresh(&referee_data_for_ui->referee_id, Char_State[i]);

    UICircleDraw(&state_circle[0], "c0", UI_Graph_ADD, 9, UI_Color_White, 10, 620, 160, 10);
    UICircleDraw(&state_circle[1], "c1", UI_Graph_ADD, 9, UI_Color_White, 10, 760, 160, 10);
    UICircleDraw(&state_circle[2], "c2", UI_Graph_ADD, 9, UI_Color_White, 10, 960, 160, 10);
    UICircleDraw(&state_circle[3], "c3", UI_Graph_ADD, 9, UI_Color_White, 10, 1300, 160, 10);
    UICircleDraw(&state_circle[4], "c4", UI_Graph_ADD, 9, UI_Color_White, 8, 555, 250, 8);
    UICircleDraw(&state_circle[5], "c5", UI_Graph_ADD, 9, UI_Color_White, 8, 655, 250, 8);
    UICircleDraw(&state_circle[6], "c6", UI_Graph_ADD, 9, UI_Color_Green, 8, 755, 250, 8);
    UICircleDraw(&state_circle[7], "c7", UI_Graph_ADD, 9, UI_Color_White, 8, 925, 250, 8);
    UICircleDraw(&state_circle[8], "c8", UI_Graph_ADD, 9, UI_Color_White, 8, 1025, 250, 8);
    UICircleDraw(&state_circle[9], "c9", UI_Graph_ADD, 9, UI_Color_White, 8, 1110, 250, 8);

    UIArcDraw(&Cap_voltage_arc, "pow", UI_Graph_ADD, 9, UI_Color_Green, 271, 273, 7, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);
    UIFloatDraw(&Cap_voltage, "of5", UI_Graph_ADD, 9, UI_Color_Pink, 15, 3, 3, 555, 550, (int32_t)(ui_cmd_recv.supercap_voltage * 1000.0f));
    UIFloatDraw(&total_voltage, "of0", UI_Graph_ADD, 9, UI_Color_Green, 10, 0, 4, 860, 660, 24000);
    UIArcDraw(&Allow_heat_arc, "hea", UI_Graph_ADD, 9, UI_Color_Green, 273, 300, 7, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);
    UIFloatDraw(&Shoot_Local_Heat, "of3", UI_Graph_ADD, 7, UI_Color_Green, 20, 3, 3, 1100, 600, (ui_cmd_recv.Shooter_heat) * 1000);
    UIArcDraw(&Deviation_arc, "dev", UI_Graph_ADD, 4, UI_Color_Pink, 48, 87, 7, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);

    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, shoot_line[0], shoot_line[1], shoot_line[2], shoot_line[3], shoot_line[4], shoot_line[5], shoot_line[6]);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, state_circle[0], state_circle[1], state_circle[2], state_circle[3], state_circle[4], state_circle[5], state_circle[6]);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, state_circle[7], state_circle[8], state_circle[9], Cap_voltage, Shoot_Local_Heat);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, Cap_voltage_arc, Allow_heat_arc, benchmark[0], benchmark[1], benchmark[2], benchmark[3], benchmark[4]);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 2, total_voltage, Deviation_arc);
}

void UI_Init(void)
{
    ui_pub = PubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    ui_sub = SubRegister("ui_cmd", sizeof(UI_Cmd_s));

    UI_StaticInit();
}

void UIDynamicRefresh(void)
{
    if (referee_data_for_ui == NULL)
        return;

    const uint32_t reticle_x = UI_SHOOT_RETICLE_X;
    const uint32_t reticle_y = UI_SHOOT_RETICLE_Y;

    SubGetMessage(ui_sub, (void *)&ui_cmd_recv);
    if (ui_cmd_recv.ui_send_flag == 0) {
        UI_StaticInit();
    }

    UICircleDraw(&state_circle[0], "c0", UI_Graph_Change, 9,
                 (ui_cmd_recv.chassis_mode == CHASSIS_ROTATE) ? UI_Color_Main : UI_Color_White,
                 10, 620, 160, 10);
    UICircleDraw(&state_circle[1], "c1", UI_Graph_Change, 9,
                 ui_cmd_recv.climb_mode ? UI_Color_Cyan : UI_Color_White,
                 10, 760, 160, 10);
    UICircleDraw(&state_circle[2], "c2", UI_Graph_Change, 9,
                 (ui_cmd_recv.friction_mode == FRICTION_ON) ? UI_Color_Main : UI_Color_White,
                 10, 960, 160, 10);
    UICircleDraw(&state_circle[3], "c3", UI_Graph_Change, 9,
                 (ui_cmd_recv.cap_online_flag == 0u) ? UI_Color_Pink : UI_Color_Green,
                 10, 1300, 160, 10);

    UICircleDraw(&state_circle[4], "c4", UI_Graph_Change, 9,
                 (ui_cmd_recv.sync_belt_state > 0) ? UI_Color_Green : UI_Color_White,
                 8, 555, 250, 8);
    UICircleDraw(&state_circle[5], "c5", UI_Graph_Change, 9,
                 (ui_cmd_recv.sync_belt_state < 0) ? UI_Color_Orange : UI_Color_White,
                 8, 655, 250, 8);
    UICircleDraw(&state_circle[6], "c6", UI_Graph_Change, 9,
                 (ui_cmd_recv.sync_belt_state == 0) ? UI_Color_Green : UI_Color_White,
                 8, 755, 250, 8);

    uint8_t vision_mode = ui_cmd_recv.vision_work_mode;
    if (vision_mode == UI_VISION_WORK_IDLE && ui_cmd_recv.nuc_flag)
        vision_mode = UI_VISION_WORK_AIM;
    UICircleDraw(&state_circle[7], "c7", UI_Graph_Change, 9,
                 (vision_mode == UI_VISION_WORK_AIM) ? UI_Color_Green : UI_Color_White,
                 8, 925, 250, 8);
    UICircleDraw(&state_circle[8], "c8", UI_Graph_Change, 9,
                 (vision_mode == UI_VISION_WORK_SMALL_RUNE) ? UI_Color_Green : UI_Color_White,
                 8, 1025, 250, 8);
    UICircleDraw(&state_circle[9], "c9", UI_Graph_Change, 9,
                 (vision_mode == UI_VISION_WORK_BIG_RUNE) ? UI_Color_Green : UI_Color_White,
                 8, 1110, 250, 8);

    float cap_ratio = (ui_cmd_recv.supercap_voltage - SUPERCAP_MIN_VOLTAGE) /
                      (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE);
    cap_ratio = clampf_ui(cap_ratio, 0.0f, 1.0f);
    uint32_t cap_color = UI_Color_Pink;
    if (ui_cmd_recv.supercap_voltage >= SUPERCAP_HIGHER_THRESHOLD_VOLTAGE)
        cap_color = UI_Color_Green;
    else if (ui_cmd_recv.supercap_voltage >= SUPERCAP_LOWER_THRESHOLD_VOLTAGE)
        cap_color = UI_Color_Orange;
    UIArcDraw(&Cap_voltage_arc, "pow", UI_Graph_Change, 9, cap_color,
              271, 272 + (uint32_t)(60.0f * cap_ratio), 7, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);
    UIFloatDraw(&Cap_voltage, "of5", UI_Graph_Change, 9, cap_color, 15, 3, 3, 555, 550,
                (int32_t)(ui_cmd_recv.supercap_voltage * 1000.0f));

    totalheat = (ui_cmd_recv.Heat_Limit > 0u) ? (float)ui_cmd_recv.Heat_Limit :
                getTotal_heat(ui_cmd_recv.robot_level);
    heattemp = clampf_ui(allow_shoot_heat(totalheat, ui_cmd_recv.Shooter_heat), 0.0f, totalheat);
    uint32_t heat_color = (heattemp > 99.0f) ? UI_Color_Green : UI_Color_Purplish_red;
    uint32_t heat_start = 266u - (uint32_t)(3.8f * 0.1f * heattemp / 5.0f);
    UIFloatDraw(&Shoot_Local_Heat, "of3", UI_Graph_Change, 7, heat_color, 20, 3, 3, 1100, 600,
                (int32_t)(1000.0f * heattemp));
    UIArcDraw(&Allow_heat_arc, "hea", UI_Graph_Change, 9, heat_color,
              heat_start, 266, 7, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, 392);
    UIArcDraw(&Deviation_arc, "dev", UI_Graph_Change, 4, UI_Color_Pink, 48, 87, 7, reticle_x, reticle_y, UI_SHOOT_ARC_RADIUS_X, UI_SHOOT_ARC_RADIUS_Y);

    UIGraphRefresh(&referee_data_for_ui->referee_id, 7, state_circle[0], state_circle[1], state_circle[2], state_circle[3], state_circle[4], state_circle[5], state_circle[6]);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 5, state_circle[7], state_circle[8], state_circle[9], Cap_voltage, Shoot_Local_Heat);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 2, Cap_voltage_arc, Allow_heat_arc);
    UIGraphRefresh(&referee_data_for_ui->referee_id, 1, Deviation_arc);

    PubPushMessage(ui_pub, (void *)&ui_feedback_data);
}
