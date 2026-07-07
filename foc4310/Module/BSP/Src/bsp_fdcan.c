/**
 * @file bsp_fdcan.c
 * @brief FDCAN 驱动层实现
 * @note  统一管理 FDCAN RX 回调, 按 rx_id 分发给注册模块
 */

#include "bsp_fdcan.h"

static uint8_t idx = 0;
static bsp_fdcan_t *fdcan_instances[DEVICE_FDCAN_CNT] = {NULL};

/**
 * @brief 初始化 FDCAN 外设 (启动 + 激活中断)
 * @note  仅需调用一次
 */
void bsp_fdcan_init(bsp_fdcan_t *fdcan) {
    HAL_FDCAN_Start(fdcan->hfdcan);
    HAL_FDCAN_ActivateNotification(fdcan->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
 * @brief 为 fdcan 设备添加滤波器
 * @note  每注册一个设备需调用一次
 */
void bsp_fdcan_filter_init(bsp_fdcan_t *fdcan) {
    FDCAN_FilterTypeDef Filter;
    Filter.IdType       = FDCAN_STANDARD_ID;
    Filter.FilterIndex  = 0;
    Filter.FilterType   = FDCAN_FILTER_MASK;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Filter.FilterID1    = fdcan->rx_id;
    Filter.FilterID2    = 0x7FF;

    HAL_FDCAN_ConfigFilter(fdcan->hfdcan, &Filter);
    HAL_FDCAN_ConfigGlobalFilter(fdcan->hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

/**
 * @brief 注册 FDCAN 设备
 * @param fdcan 设备实例指针
 * @param rx_id 期望接收的 CAN ID
 * @param device_instance 回调时传回的模块实例
 * @param callback 接收回调
 */
void bsp_fdcan_register(bsp_fdcan_t *fdcan, uint32_t rx_id,
                        void *device_instance, fdcan_rx_callback callback) {
    if (idx >= DEVICE_FDCAN_CNT) return;

    fdcan->rx_id           = rx_id;
    fdcan->device_instance = device_instance;
    fdcan->device_callback = callback;

    fdcan_instances[idx++] = fdcan;

    // 添加该设备 ID 的滤波器
    bsp_fdcan_filter_init(fdcan);
}

/**
 * @brief FDCAN 发送
 * @param fdcan 设备实例 (需预先填充 tx_id / fdcan_tx_buff / tx_len)
 */
void bsp_fdcan_tx(bsp_fdcan_t *fdcan) {
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier          = fdcan->tx_id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = (uint32_t)fdcan->tx_len << 16,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    HAL_FDCAN_AddMessageToTxFifoQ(fdcan->hfdcan, &TxHeader, fdcan->fdcan_tx_buff);
}

/**
 * @brief FDCAN RX FIFO0 回调 — 按 rx_id 分发给对应 bsp_fdcan_t
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    static FDCAN_RxHeaderTypeDef RxHeader;
    static uint8_t rx_data[8];

    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rx_data);

        for (uint8_t i = 0; i < idx; ++i) {
            if (fdcan_instances[i]->hfdcan == hfdcan &&
                fdcan_instances[i]->rx_id == RxHeader.Identifier &&
                fdcan_instances[i]->device_callback != NULL) {
                fdcan_instances[i]->device_callback(
                    fdcan_instances[i]->device_instance,
                    rx_data, RxHeader.DataLength >> 16);
            }
        }
    }
}
