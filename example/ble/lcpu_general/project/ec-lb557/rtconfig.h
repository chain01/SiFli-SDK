/*Auto generated configuration*/
#ifndef RT_CONFIG_H
#define RT_CONFIG_H

#define SOC_SF32LB55X 1
#define CORE "LCPU"
#define CPU "Cortex-M33"
#define BSP_USING_RTTHREAD 1
#define RT_USING_COMPONENTS_INIT 1
#define RT_USING_USER_MAIN 1
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_DEVICE_IPC 1
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL 1
#define RT_SERIAL_USING_DMA 1
#define RT_SERIAL_RB_BUFSZ 64
#define RT_SERIAL_DEFAULT_BAUDRATE 1000000
#define RT_USING_HWMAILBOX 1
#define RT_USING_WDT 1
#define WDT_TIMEOUT 30
#define RT_USING_LIBC 1
#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32 1
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK 1
#define RT_USING_HOOK 1
#define RT_USING_IDLE_HOOK 1
#define RT_IDEL_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 512
#define RT_USING_TIMER_SOFT 1
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 512
#define RT_DEBUG 1
#define RT_USING_SEMAPHORE 1
#define RT_USING_MUTEX 1
#define RT_USING_EVENT 1
#define RT_USING_MAILBOX 1
#define RT_USING_MESSAGEQUEUE 1
#define RT_USING_MEMPOOL 1
#define RT_USING_MEMHEAP 1
#define RT_USING_SMALL_MEM 1
#define RT_USING_HEAP 1
#define RT_USING_DEVICE 1
#define RT_USING_CONSOLE 1
#define RT_CONSOLEBUF_SIZE 256
#define RT_CONSOLE_DEVICE_NAME "uart5"
#define RT_VER_NUM 0x30103
#define BSP_USING_DMA 1
#define BSP_USING_UART 1
#define BSP_USING_UART3 1
#define BSP_UART3_RX_USING_DMA 1
#define BSP_USING_UART5 1
#define BSP_UART5_RX_USING_DMA 1
#define BSP_USING_WDT 1
#define BSP_USING_BOARD_EC_LB557XXX 1
#define LXT_FREQ 32768
#define LXT_LP_CYCLE 200
#define BLE_TX_POWER_VAL 0
#define BSP_LB55X_CHIP_ID 2
#define ASIC 1
#define BSP_USING_FULL_ASSERT 1
#define BLUETOOTH 1
#define ROM_ATT_BUF_SIZE 3084
#define ROM_LOG_SIZE 4096
#define BLE_ACTIVITY_6 1
#define ROM_LOG_PORT "uart3"
#define USING_IPC_QUEUE 1
#define USING_IPC_QUEUE_DEVICE_WRAPPER 1
#define AUDIO_PATH_USING_HCI 1
#define AUDIO_SPEAKER_USING_I2S2 1
#define AUDIO_MIC_USING_PDM 1
#define BF0_LCPU 1
#define LCPU_ROM 1
#define CUSTOM_MEM_MAP 1
#endif
