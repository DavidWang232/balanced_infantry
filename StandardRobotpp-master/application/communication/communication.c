/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       communication.c/h
  * @brief      这里是机器人通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-14-2024     Penguin         1. done
  *  V1.1.0     2025-03-10      Harry_Wong      1. 初步完成自定义内容通信
  *  V1.2.0     Apr-01-2025     Penguin         1. 重构与优化
  *
  @verbatim
  ==============================================================================
  关于Usart1 和 Uart2 之间的关系：Usart1是内部配置，Uart2是C板上的外侧标注，两者为对应关系。

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "communication.h"

#include "CRC8_CRC16.h"
#include "bsp_uart.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "fifo.h"
#include "gimbal.h"
#include "robot_param.h"
#include "signal_generator.h"
#include "uart2_typedef.h"
#include "usart.h"
#include "usb_debug.h"

/*******************************************************************************/
/* Macro Definitions                                                           */
/*******************************************************************************/

#define UART2_OFFLINE_TIME 200  // ms

#define USART_RX_BUF_LENGHT 512
#define USART1_FIFO_BUF_LENGTH 1024

#define USART1_DEBUG_PACKAGE_NUM 10
#define USART1_DEBUG_SEND_DURATION 5  // ms
// VOFA+ RawData 帧尾：IEEE 754 NaN = 0x7F800000 (小端: 0x00 0x00 0x80 0x7F)
#define VOFA_RAWDATA_TAIL_SIZE 4
static const uint8_t VOFA_RAWDATA_TAIL[VOFA_RAWDATA_TAIL_SIZE] = {0x00, 0x00, 0x80, 0x7F};

typedef struct
{
    uint8_t name[10];
    float data;
} __attribute__((packed)) Usart1DebugPackage_t;

/*******************************************************************************/
/* Variable Definitions                                                        */
/*******************************************************************************/
uint32_t TASK_DURATION;
uint32_t LAST_TASK_TIME;

// clang-format off
// data time record
LastTime_t LastSendTime;
LastTime_t LastReceiveTime;


// receive data
Data_Rc_s     Receive_Data_Rc;
// receive data buffer
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
static fifo_s_t usart1_fifo;
uint8_t usart1_fifo_buf[USART1_FIFO_BUF_LENGTH];

static RC_ctrl_t ykq_rc_ctrl;
static uint32_t ykq_rc_last_receive_time = 0;
static Usart1DebugPackage_t usart1_debug_packages[USART1_DEBUG_PACKAGE_NUM];
// 数据区 + 4字节帧尾
static uint8_t usart1_debug_tx_buf[USART1_DEBUG_PACKAGE_NUM * sizeof(float) + VOFA_RAWDATA_TAIL_SIZE];
static uint32_t usart1_debug_last_send_time = 0;
// clang-format on

static void Usart1DebugInit(void);
static bool Usart1TxIdle(void);
static void Usart1SendDebugData(void);

/*******************************************************************************/
/* Main Functions                                                              */
/*     Usart1Init                                                              */
/*     USART1_IRQHandler                                                       */
/*******************************************************************************/

// 4pin Uart串口初始化
void Usart1Init(void)
{
    fifo_s_init(&usart1_fifo, usart1_fifo_buf, USART1_FIFO_BUF_LENGTH);
    usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);
    Usart1DebugInit();
}

static void Usart1DebugInit(void)
{
    memset(usart1_debug_packages, 0, sizeof(usart1_debug_packages));
    memset(usart1_debug_tx_buf, 0, sizeof(usart1_debug_tx_buf));
    // 预填帧尾，避免每次发送时重复写入
    memcpy(
        &usart1_debug_tx_buf[USART1_DEBUG_PACKAGE_NUM * sizeof(float)],
        VOFA_RAWDATA_TAIL, VOFA_RAWDATA_TAIL_SIZE);

    for (uint8_t i = 0; i < USART1_DEBUG_PACKAGE_NUM; i++) {
        usart1_debug_packages[i].name[0] = '\0';
    }
}

static bool Usart1TxIdle(void) { return true; }

static void Usart1SendDebugData(void)
{
    if ((HAL_GetTick() - usart1_debug_last_send_time) < USART1_DEBUG_SEND_DURATION) {
        return;
    }

    if (!Usart1TxIdle()) {
        return;
    }

    usart1_debug_last_send_time = HAL_GetTick();
    for (uint8_t i = 0; i < USART1_DEBUG_PACKAGE_NUM; i++) {
        float val = usart1_debug_packages[i].data;
        memcpy(&usart1_debug_tx_buf[i * sizeof(float)], &val, sizeof(float));
    }

    usart1_tx_dma_enable(usart1_debug_tx_buf, sizeof(usart1_debug_tx_buf));
}
// 4pin Uart口中断处理函数
//双缓冲
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if (USART1->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[0], this_time_rx_len);
            LastReceiveTime.Interrupt = HAL_GetTick();
        } else {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[1], this_time_rx_len);
            LastReceiveTime.Interrupt = HAL_GetTick();
        }
    }
}

/*******************************************************************************/
/* YKQ RC Data Parse Functions                                                 */
/*     YkqRcUnpack                                                             */
/*     YkqRcDataParse                                                          */
/*******************************************************************************/

/**
  * @brief          YKQ遥控器数据解析函数
  * @param[in]      frame: YKQ遥控器原始数据帧
  * @param[out]     rc_ctrl: 转换后的RC_ctrl_t结构体
  * @retval         none
  */
static void YkqRcDataParse(const uint8_t * frame, RC_ctrl_t * rc_ctrl)
{
    if (frame == NULL || rc_ctrl == NULL) {
        return;
    }

    // 按ykq.md定义的位偏移直接从21字节原始帧中解包，避免结构体字段错位带来的解析误差。
    // frame[0]=0xA9, frame[1]=0x53
    uint16_t ch0 = (uint16_t)(frame[2] | ((frame[3] & 0x07) << 8));
    uint16_t ch1 = (uint16_t)((frame[3] >> 3) | ((frame[4] & 0x3F) << 5));
    uint16_t ch2 = (uint16_t)((frame[4] >> 6) | (frame[5] << 2) | ((frame[6] & 0x01) << 10));
    uint16_t ch3 = (uint16_t)((frame[6] >> 1) | ((frame[7] & 0x0F) << 7));

    // 设置到RC_ctrl_t中（转换为[-660, 660]范围）
    rc_ctrl->rc.ch[0] = (int16_t)ch0 - YKQ_RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] = (int16_t)ch1 - YKQ_RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] = (int16_t)ch2 - YKQ_RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] = (int16_t)ch3 - YKQ_RC_CH_VALUE_OFFSET;

    // 拨轮11位：bit65~75 => frame[8]的bit1~7 + frame[9]的bit0~3
    uint16_t wheel = (uint16_t)((frame[8] >> 1) | ((frame[9] & 0x0F) << 7));
    rc_ctrl->rc.ch[4] = (int16_t)wheel - YKQ_RC_CH_VALUE_OFFSET;

    // 挡位2位：bit60~61 => frame[7]的bit4~5，C:0 N:1 S:2
    uint8_t gear = (frame[7] >> 4) & 0x03;
    // 显式映射：s[0]=2->down, s[0]=3->mid, s[0]=1->up
    if (gear == 0) {
        rc_ctrl->rc.s[0] = RC_SW_DOWN;
    } else if (gear == 1) {
        rc_ctrl->rc.s[0] = RC_SW_MID;
    } else if (gear == 2) {
        rc_ctrl->rc.s[0] = RC_SW_UP;
    } else {
        rc_ctrl->rc.s[0] = RC_SW_MID;
    }

    // 自定义左键：bit63 => frame[7]的bit7（右键是bit64，在frame[8]的bit0）
    uint8_t custom_left = (frame[7] >> 7) & 0x01;
    rc_ctrl->rc.s[1] = custom_left ? RC_SW_UP : RC_SW_DOWN;

    // 鼠标XYZ：bit80开始，按小端16位
    rc_ctrl->mouse.x = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
    rc_ctrl->mouse.y = (int16_t)((uint16_t)frame[12] | ((uint16_t)frame[13] << 8));
    rc_ctrl->mouse.z = (int16_t)((uint16_t)frame[14] | ((uint16_t)frame[15] << 8));

    // 鼠标按键在frame[16]
    uint8_t mouse_key_byte = frame[16];
    rc_ctrl->mouse.press_l = (mouse_key_byte & 0x03);
    rc_ctrl->mouse.press_r = ((mouse_key_byte >> 2) & 0x03);

    // 键盘16位在frame[17..18]，小端
    rc_ctrl->key.v = (uint16_t)(frame[17] | (frame[18] << 8));
}

/**
  * @brief          YKQ遥控器数据解包
  * @param[in]      void
  * @retval         none
  */
static void YkqRcUnpack(void)
{
    uint8_t byte = 0;
    static uint8_t frame_buf[YKQ_RC_FRAME_LENGTH];
    static uint8_t frame_index = 0;
    static uint8_t sof1_found = 0;

    while (fifo_s_used(&usart1_fifo)) {
        byte = fifo_s_get(&usart1_fifo);

        if (!sof1_found) {
            // 查找第一个帧头
            if (byte == YKQ_RC_SOF1) {
                frame_buf[0] = byte;
                frame_index = 1;
                sof1_found = 1;
            }
        } else if (frame_index == 1) {
            // 检查第二个帧头
            if (byte == YKQ_RC_SOF2) {
                frame_buf[1] = byte;
                frame_index = 2;
            } else {
                // 帧头错误，重新开始
                sof1_found = 0;
                frame_index = 0;
            }
        } else if (frame_index < YKQ_RC_FRAME_LENGTH) {
            // 继续填充数据
            frame_buf[frame_index] = byte;
            frame_index++;

            // 接收完整帧
            if (frame_index == YKQ_RC_FRAME_LENGTH) {
                // 校验CRC16
                if (verify_CRC16_check_sum(frame_buf, YKQ_RC_FRAME_LENGTH)) {
                    // CRC校验成功，解析数据
                    YkqRcDataParse(frame_buf, &ykq_rc_ctrl);
                    ykq_rc_last_receive_time = HAL_GetTick();
                }
                // 重置状态，继续接收下一帧
                sof1_found = 0;
                frame_index = 0;
            }
        }
    }
}

/*******************************************************************************/
/* Uart2 Task Loop Function                                                    */
/*     Uart2TaskLoop                                                           */
/*******************************************************************************/

void Uart2TaskLoop(void)
{
    TASK_DURATION = HAL_GetTick() - LAST_TASK_TIME;
    LAST_TASK_TIME = HAL_GetTick();
    // 处理YKQ遥控器数据
    YkqRcUnpack();
    Usart1SendDebugData();
}

void ModifyUsart1DebugDataPackage(uint8_t index, float data, const char * name)
{
    if (index >= USART1_DEBUG_PACKAGE_NUM || name == NULL) {
        return;
    }

    usart1_debug_packages[index].data = data;

    if (usart1_debug_packages[index].name[0] != '\0') {
        return;
    }

    uint8_t i = 0;
    while (name[i] != '\0' && i < sizeof(usart1_debug_packages[index].name)) {
        usart1_debug_packages[index].name[i] = (uint8_t)name[i];
        i++;
    }
}

/**
  * @brief          获取YKQ遥控器数据指针
  * @param[in]      none
  * @retval         YKQ遥控器数据指针
  */
const RC_ctrl_t * get_remote_control_tuchaun_point(void) { return &ykq_rc_ctrl; }

/**
  * @brief          获取YKQ遥控器是否离线
  * @retval         true:离线，false:在线
  */
bool GetYkqRcOffline(void)
{
    if (HAL_GetTick() - ykq_rc_last_receive_time > 200) {  // 200ms超时
        return true;
    }
    return false;
}

/*------------------------------ End of File ------------------------------*/
