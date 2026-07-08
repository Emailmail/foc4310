/**
 * @file bsp_fdcan.c
 * @brief FDCAN 驱动层实现
 * @note  统一管理 FDCAN RX 回调, 按 rx_id 分发给注册模块
 */

#include "bsp_fdcan.h"

static uint8_t idx = 0;
static bsp_fdcan_t *fdcan_instances[DEVICE_FDCAN_CNT] = {NULL};

/**
 * @brief 初始化 FDCAN 外设 (全局滤波策略 + 启动 + 激活中断)
 * @note  仅需调用一次。ConfigGlobalFilter 只能在 READY 状态调用,
 *        故必须在 HAL_FDCAN_Start 之前;设备滤波器(ConfigFilter)
 *        在 register 中配置,BUSY 状态也允许,所以 register 可在 init 之后调用。
 */
void bsp_fdcan_init(bsp_fdcan_t *fdcan) {
    // 拒绝所有不匹配标准/扩展 ID 及远程帧 (必须在 Start 前)
    HAL_FDCAN_ConfigGlobalFilter(fdcan->hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(fdcan->hfdcan);
    HAL_FDCAN_ActivateNotification(fdcan->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
 * @brief 为 fdcan 设备添加接收滤波器 (占用其独立的 filter_idx 槽)
 * @note  由 register 调用;filter_idx 需先被赋值
 */
void bsp_fdcan_filter_init(bsp_fdcan_t *fdcan) {
    FDCAN_FilterTypeDef Filter;
    Filter.IdType       = FDCAN_STANDARD_ID;
    Filter.FilterIndex  = fdcan->filter_idx;
    Filter.FilterType   = FDCAN_FILTER_MASK;
    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Filter.FilterID1    = fdcan->rx_id;
    Filter.FilterID2    = 0x7FF;   // 掩码全 1 → 精确匹配 rx_id

    HAL_FDCAN_ConfigFilter(fdcan->hfdcan, &Filter);
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
    // 滤波器槽由 CubeMX 的 StdFiltersNbr 决定, 超出会越界写 message RAM
    if (idx >= fdcan->hfdcan->Init.StdFiltersNbr) return;

    fdcan->rx_id           = rx_id;
    fdcan->device_instance = device_instance;
    fdcan->device_callback = callback;
    fdcan->filter_idx      = idx;    // 每个设备独占一个硬件滤波器槽

    fdcan_instances[idx++] = fdcan;

    // 添加该设备 ID 的滤波器
    bsp_fdcan_filter_init(fdcan);
}

/**
 * @brief FDCAN 发送
 * @param fdcan 设备实例 (需预先填充 tx_id / fdcan_tx_buff / tx_len)
 */
void bsp_fdcan_tx(bsp_fdcan_t *fdcan) {
    uint8_t len = (fdcan->tx_len > 8) ? 8 : fdcan->tx_len;  // 缓冲区仅 8 字节, classic CAN 上限
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier          = fdcan->tx_id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = (uint32_t)len,
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
                    rx_data, RxHeader.DataLength);
            }
        }
    }
}
