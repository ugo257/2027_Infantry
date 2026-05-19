#include "unicomm.h"

#include <string.h>

#define CTRL_FLOAT_BASE_INDEX        2u
#define CTRL_FLAGS_BASE_INDEX        (CTRL_FLOAT_BASE_INDEX + 52u)
#define CTRL_FLAGS_COUNT             11u
#define CTRL_CHECKSUM_INDEX          (CTRL_FLAGS_BASE_INDEX + CTRL_FLAGS_COUNT)

#define UPLOAD_FLOAT_BASE_INDEX      2u
#define UPLOAD_COLOR_INDEX           (sizeof(Chassis_Upload_Data_s_uart) + 1u)
#define UPLOAD_CHECKSUM_INDEX        (sizeof(Chassis_Upload_Data_s_uart) + 2u)

void UniCommFloatToBytes(float value, uint8_t *bytes)
{
    uint32_t raw = 0;
    memcpy(&raw, &value, sizeof(float));
    bytes[0] = (uint8_t)(raw & 0xFFu);
    bytes[1] = (uint8_t)((raw >> 8) & 0xFFu);
    bytes[2] = (uint8_t)((raw >> 16) & 0xFFu);
    bytes[3] = (uint8_t)((raw >> 24) & 0xFFu);
}

float UniCommBytesToFloat(const uint8_t *bytes)
{
    uint32_t raw = 0;
    raw |= ((uint32_t)bytes[0]);
    raw |= ((uint32_t)bytes[1] << 8);
    raw |= ((uint32_t)bytes[2] << 16);
    raw |= ((uint32_t)bytes[3] << 24);

    float value = 0.0f;
    memcpy(&value, &raw, sizeof(float));
    return value;
}

static void int32_to_bytes_be(int32_t value, uint8_t *bytes)
{
    bytes[0] = (uint8_t)((value >> 24) & 0xFFu);
    bytes[1] = (uint8_t)((value >> 16) & 0xFFu);
    bytes[2] = (uint8_t)((value >> 8) & 0xFFu);
    bytes[3] = (uint8_t)(value & 0xFFu);
}

static int32_t bytes_to_int32_be(const uint8_t *bytes)
{
    return ((int32_t)bytes[0] << 24) |
           ((int32_t)bytes[1] << 16) |
           ((int32_t)bytes[2] << 8) |
           ((int32_t)bytes[3]);
}

uint8_t UniCommChecksum(const uint8_t *data, uint16_t len)
{
    uint8_t checksum = 0u;
    for (uint16_t i = 0; i < len; ++i) {
        checksum = (uint8_t)(checksum + data[i]);
    }
    return checksum;
}

uint16_t UniCommPackChassisCtrl(const Chassis_Ctrl_Cmd_s_uart *src, uint8_t *dst, uint16_t dst_len)
{
    if (src == NULL || dst == NULL || dst_len < UNICOMM_CTRL_FRAME_LEN) {
        return 0u;
    }

    memset(dst, 0, UNICOMM_CTRL_FRAME_LEN);
    dst[0] = UNICOMM_FRAME_HEAD;
    dst[1] = UNICOMM_FRAME_TYPE_CTRL_CMD;

    UniCommFloatToBytes(src->vx, dst + CTRL_FLOAT_BASE_INDEX + 0u);
    UniCommFloatToBytes(src->vy, dst + CTRL_FLOAT_BASE_INDEX + 4u);
    UniCommFloatToBytes(src->wz, dst + CTRL_FLOAT_BASE_INDEX + 8u);
    UniCommFloatToBytes(src->yaw_control, dst + CTRL_FLOAT_BASE_INDEX + 12u);
    UniCommFloatToBytes(src->yaw_angle, dst + CTRL_FLOAT_BASE_INDEX + 16u);
    UniCommFloatToBytes(src->yaw_gyro, dst + CTRL_FLOAT_BASE_INDEX + 20u);
    UniCommFloatToBytes(src->offset_angle, dst + CTRL_FLOAT_BASE_INDEX + 24u);
    UniCommFloatToBytes(src->gimbal_error_angle, dst + CTRL_FLOAT_BASE_INDEX + 28u);
    int32_to_bytes_be(src->shoot_count, dst + CTRL_FLOAT_BASE_INDEX + 32u);
    UniCommFloatToBytes(src->nuc_yaw, dst + CTRL_FLOAT_BASE_INDEX + 36u);
    UniCommFloatToBytes(src->yaw_vel, dst + CTRL_FLOAT_BASE_INDEX + 40u);  // 视觉yaw速度前馈

    UniCommFloatToBytes(src->yaw_acc, dst + CTRL_FLOAT_BASE_INDEX + 44u);
    UniCommFloatToBytes(src->leg_length_cmd, dst + CTRL_FLOAT_BASE_INDEX + 48u);

    dst[CTRL_FLAGS_BASE_INDEX + 0u] = (uint8_t)src->nuc_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 1u] = src->UI_SendFlag;
    dst[CTRL_FLAGS_BASE_INDEX + 2u] = src->superCap_flag;
    dst[CTRL_FLAGS_BASE_INDEX + 3u] = src->reset_flag;
    dst[CTRL_FLAGS_BASE_INDEX + 4u] = (uint8_t)src->friction_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 5u] = (uint8_t)src->shoot_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 6u] = (uint8_t)src->load_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 7u] = (uint8_t)src->chassis_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 8u] = (uint8_t)src->gimbal_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 9u] = (uint8_t)src->sync_belt_cmd;
    dst[CTRL_FLAGS_BASE_INDEX + 10u] = src->vision_work_mode;

    dst[CTRL_CHECKSUM_INDEX] = UniCommChecksum(dst, CTRL_CHECKSUM_INDEX);
    return UNICOMM_CTRL_FRAME_LEN;
}

bool UniCommUnpackChassisCtrl(const uint8_t *src, uint16_t src_len, Chassis_Ctrl_Cmd_s_uart *dst)
{
    if (src == NULL || dst == NULL || src_len < UNICOMM_CTRL_FRAME_LEN) {
        return false;
    }
    if (src[0] != UNICOMM_FRAME_HEAD || src[1] != UNICOMM_FRAME_TYPE_CTRL_CMD) {
        return false;
    }
    if (UniCommChecksum(src, CTRL_CHECKSUM_INDEX) != src[CTRL_CHECKSUM_INDEX]) {
        return false;
    }

    dst->vx = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 0u);
    dst->vy = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 4u);
    dst->wz = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 8u);
    dst->yaw_control = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 12u);
    dst->yaw_angle = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 16u);
    dst->yaw_gyro = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 20u);
    dst->offset_angle = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 24u);
    dst->gimbal_error_angle = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 28u);
    dst->shoot_count = bytes_to_int32_be(src + CTRL_FLOAT_BASE_INDEX + 32u);
    dst->nuc_yaw = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 36u);
    dst->yaw_vel = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 40u);  // 视觉yaw速度前馈

    dst->yaw_acc = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 44u);
    dst->leg_length_cmd = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 48u);

    dst->nuc_mode = (nuc_mode_e)src[CTRL_FLAGS_BASE_INDEX + 0u];
    dst->UI_SendFlag = src[CTRL_FLAGS_BASE_INDEX + 1u];
    dst->superCap_flag = src[CTRL_FLAGS_BASE_INDEX + 2u];
    dst->reset_flag = src[CTRL_FLAGS_BASE_INDEX + 3u];
    dst->friction_mode = (friction_mode_e)src[CTRL_FLAGS_BASE_INDEX + 4u];
    dst->shoot_mode = (shoot_mode_e)src[CTRL_FLAGS_BASE_INDEX + 5u];
    dst->load_mode = (loader_mode_e)src[CTRL_FLAGS_BASE_INDEX + 6u];
    dst->chassis_mode = (chassis_mode_e)src[CTRL_FLAGS_BASE_INDEX + 7u];
    dst->gimbal_mode = (gimbal_mode_e)src[CTRL_FLAGS_BASE_INDEX + 8u];
    dst->sync_belt_cmd = (int8_t)src[CTRL_FLAGS_BASE_INDEX + 9u];
    dst->vision_work_mode = src[CTRL_FLAGS_BASE_INDEX + 10u];
    return true;
}

uint16_t UniCommPackChassisUpload(const Chassis_Upload_Data_s_uart *src, uint8_t *dst, uint16_t dst_len)
{
    if (src == NULL || dst == NULL || dst_len < UNICOMM_UPLOAD_FRAME_LEN) {
        return 0u;
    }

    memset(dst, 0, UNICOMM_UPLOAD_FRAME_LEN);
    dst[0] = UNICOMM_FRAME_HEAD;
    dst[1] = UNICOMM_FRAME_TYPE_UPLOAD;

    UniCommFloatToBytes(src->yaw_motor_single_round_angle, dst + UPLOAD_FLOAT_BASE_INDEX + 0u);
    UniCommFloatToBytes(src->yaw_total_angle, dst + UPLOAD_FLOAT_BASE_INDEX + 4u);
    UniCommFloatToBytes(src->yaw_ecd, dst + UPLOAD_FLOAT_BASE_INDEX + 8u);
    UniCommFloatToBytes(src->chassis_pitch_angle, dst + UPLOAD_FLOAT_BASE_INDEX + 12u);
    UniCommFloatToBytes(src->chassis_yaw_gyro, dst + UPLOAD_FLOAT_BASE_INDEX + 16u);
    UniCommFloatToBytes(src->initial_speed, dst + UPLOAD_FLOAT_BASE_INDEX + 20u);
    UniCommFloatToBytes(src->yaw_motor_real_current, dst + UPLOAD_FLOAT_BASE_INDEX + 24u);
    UniCommFloatToBytes(src->yaw_angle_pidout, dst + UPLOAD_FLOAT_BASE_INDEX + 28u);

    dst[UPLOAD_COLOR_INDEX] = (uint8_t)src->color;
    dst[UPLOAD_CHECKSUM_INDEX] = UniCommChecksum(dst, UPLOAD_CHECKSUM_INDEX);
    return UNICOMM_UPLOAD_FRAME_LEN;
}

bool UniCommUnpackChassisUpload(const uint8_t *src, uint16_t src_len, Chassis_Upload_Data_s_uart *dst)
{
    if (src == NULL || dst == NULL || src_len < UNICOMM_UPLOAD_FRAME_LEN) {
        return false;
    }
    if (src[0] != UNICOMM_FRAME_HEAD || src[1] != UNICOMM_FRAME_TYPE_UPLOAD) {
        return false;
    }
    if (UniCommChecksum(src, UPLOAD_CHECKSUM_INDEX) != src[UPLOAD_CHECKSUM_INDEX]) {
        return false;
    }

    dst->yaw_motor_single_round_angle = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 0u);
    dst->yaw_total_angle = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 4u);
    dst->yaw_ecd = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 8u);
    dst->chassis_pitch_angle = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 12u);
    dst->chassis_yaw_gyro = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 16u);
    dst->initial_speed = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 20u);
    dst->yaw_motor_real_current = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 24u);
    dst->yaw_angle_pidout = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 28u);
    dst->color = (version_color)src[UPLOAD_COLOR_INDEX];
    return true;
}
