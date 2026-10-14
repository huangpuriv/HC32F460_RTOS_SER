#ifndef UART4_DMA_H
#define UART4_DMA_H
#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"    /* USART4_ERROR_IRQn / USART4_RX_IRQn : 工程预留向量 */

/*******************************************************************************
 * USART4 收发模式选择 (宏定义自由切换)
 *
 *   USART_NORMAL   = 0 : 中断方式, 逐字节收发 (原方式)
 *   USART4_DMA_RTO = 1 : RX DMA + 硬件 RTO 超时判帧 + TX DMA (默认)
 *
 * 切换方法(任选其一):
 *   1) 改下面的 UART4_MODE 默认值;
 *   2) Keil -> Options for Target -> C/C++ -> Define 里加
 *      UART4_MODE=USART_NORMAL   (工程级宏, 会覆盖第 1 条的默认值)
 ******************************************************************************/
#define USART_NORMAL            0
#define USART4_DMA_RTO          1

#ifndef UART4_MODE
#define UART4_MODE              USART4_DMA_RTO      /* 默认: DMA 收发 + RTO 不定长帧 */
#endif

/*------------------------- 硬件资源 (两种模式共用) ----------------------------*/
#define USART4_UNIT             M4_USART4
#define USART4_BAUDRATE         (115200UL)

/* PB14: 功能组 Func_Grp2 / Func36 = USART4_TX
 * PB15: 功能组 Func_Grp2 / Func37 = USART4_RX
 * (见 HC32F460 数据手册 表2-2 与引脚功能表, PB14/PB15 属 Func_Grp2) */
#define USART4_RX_PORT          PortB
#define USART4_RX_PIN           Pin15
#define USART4_RX_FUNC          Func_Usart4_Rx

#define USART4_TX_PORT          PortB
#define USART4_TX_PIN           Pin14
#define USART4_TX_FUNC          Func_Usart4_Tx

#define USART4_CLK              PWC_FCG1_PERIPH_USART4

#define USART4_RI_NUM           INT_USART4_RI
#define USART4_EI_NUM           INT_USART4_EI
#define USART4_RTO_NUM          INT_USART4_RTO

/* 中断向量分配 (重要):
 * 原来用的 Int025/Int026/Int027 已被按键唤醒 KEY_WK0/WK1(System_PowerDown.c)
 * 占用, enIrqRegistration() 对已分配的向量会直接返回错误 -> USART4 中断全都不生效。
 * 这里改用工程为 USART4 预留的向量:
 *   NORMAL   模式: Int018 = 接收错误(REI), Int019 = 接收(RI)
 *   DMA_RTO  模式: Int018 = 接收错误(REI), Int019 = 接收超时(RTOI)
 */
#define USART4_ER_IRQn          USART4_ERROR_IRQn   /* Int018_IRQn */
#define USART4_RX_IT_IRQn       USART4_RX_IRQn      /* Int019_IRQn */

/* 收发单帧缓冲上限 (两种模式共用) */
#define UART4_RX_BUF_SIZE       (256u)
#define UART4_TX_BUF_SIZE       (256u)

/*------------------------------------------------------------------------------
 * 收到一帧后, 应用层如何处理。仅 DMA_RTO 模式有效(RTO 判帧才有"一帧"的概念)。
 *
 *   UART4_APP_ECHO    = 0 : 原样回显 "RCV[长度]:数据\r\n"  (UART4 通路自测用)
 *   UART4_APP_MANAGER = 1 : "Energy Controller <-> F460 Manager" APDU 应答器,
 *                           由 App/Manager_Protocol.c 实现  (默认)
 *
 * 切换方法(任选其一):
 *   1) 改下面的 UART4_APP_MODE 默认值;
 *   2) Keil -> Options for Target -> C/C++ -> Define 里加
 *      UART4_APP_MODE=UART4_APP_ECHO   (工程级宏, 会覆盖默认值, 不改源码)
 *---------------------------------------------------------------------------*/
#define UART4_APP_ECHO          (0)
#define UART4_APP_MANAGER       (1)

#ifndef UART4_APP_MODE
#define UART4_APP_MODE          UART4_APP_MANAGER
#endif

/*--------------------------- DMA + RTO 模式参数 -------------------------------*/
#if (UART4_MODE == USART4_DMA_RTO)

#define UART4_RX_DMA_UNIT       (M4_DMA1)           /* RX DMA 单元 */
#define UART4_RX_DMA_CH         (DmaCh2)            /* RX DMA 通道 (2) */
#define UART4_TX_DMA_UNIT       (M4_DMA1)           /* TX DMA 单元 */
#define UART4_TX_DMA_CH         (DmaCh3)            /* TX DMA 通道 (3) */
#define UART4_DMA_CLK           PWC_FCG0_PERIPH_DMA1

/*---------------- RTO(接收超时) : USART4 对应 Timer0 Unit2 的 B 通道 --------------
 * 参考手册 25.4.1.6: 超时计数器用 Timer0 的计数器, 计数时钟为 USART 波特率时钟
 *   USART1->TMR0 U1A   USART2->TMR0 U1B   USART3->TMR0 U2A   USART4->TMR0 U2B
 *
 * 超时时间单位 = "接收位数"(bit)。手册 25.4.1.6 原文:
 *   CMPA<B>R = RTB * 2^CKDIVA<B> - 4   (计数时钟不分频)
 *   CMPA<B>R = RTB * 2^CKDIVA<B> - 2   (计数时钟 2 分频)
 *   CMPA<B>R = RTB * 2^CKDIVA<B> - 1   (计数时钟 4 分频及以上)
 * 所以 CKDIVB=0 时 K = 4, CMPBR = RTB - 4。RTB 上限 65535 (CMPBR 是 16 位)。
 *
 * ---- RTB 怎么选 (重要) ----
 * 一个 8N1 字符 = 10 bit。115200bps 下 1 bit = 8.68us, 所以:
 *     RTB = 10  -> 87us    (1 个字符; 连续字节之间就会误判)
 *     RTB = 40  -> 347us   (4 个字符, Modbus RTU 的 3.5 字符量级)
 *     RTB = 64  -> 556us   (当前默认, 6.4 个字符)
 *     RTB = 1150 -> 10ms   (USB 转串口适配器的 latency timer 量级)
 * RI 事件在停止位中间附近产生, 所以两个字节的 RI 间隔正好是 10 bit,
 * 而实际静默时间比 RTB 少约 0.5 bit。
 *
 * 注意: 若对端是 PC + USB 转串口(CH340/FT232 等), 帧内字节间隙可能到几百 us
 * ~ 几 ms, 单靠 RTO 判帧会把一帧拆成多帧。本协议(68 + 长度域 + FCS)有明确的
 * 长度字段, 更稳妥的办法是打开下面的 UART4_RTO_REASSEMBLE, 在应用层把短帧续起来。
 */
#define UART4_RTO_TMR_UNIT      (M4_TMR02)          /* USART4 专用的超时计数器 */
#define UART4_RTO_TMR_CH        (Tim0_ChannelB)
#define UART4_RTO_TMR_CLK       PWC_FCG2_PERIPH_TIM02
#define UART4_RTO_CKDIV         (Tim0_ClkDiv0)      /* CKDIVB = 0, 计数时钟不分频 */
#define UART4_RTO_BITS          (64u)               /* 超时 = 64 个接收位时间 */
#define UART4_RTO_CMP_VALUE     (UART4_RTO_BITS - 4u)

/* TMR0_BCONR 通道B(高16位) 控制位 (手册 22.5.3)
 * b16 CSTB  b18 INTENB  b20~23 CKDIVB  b24 SYNSB  b26 ASYNCLKB  b28 HSTAB  b30 HCLEB
 * 其中除 CSTB 外全部由 TIMER0_BaseInit()/TIMER0_HardTriggerInit() 设置,
 * 本文件只在 RTO 中断里清 CSTB 停表, 因此只定义这一个位。 */
#define UART4_RTO_BCONR_CSTB    (1ul << 16)

/* ---- AOS 硬件触发源 (手册 11.3.1 专用触发源 + 22.3.3 硬件触发动作) ----
 * Timer0 的硬件触发源必须由 TMR0_HTSSR.TRGSEL 写明事件编号, 不能留 0x1FF!
 * UART4 的 RTO 用"接收数据寄存器满(RI)"事件:
 *   每收到一个字节 -> RI 事件 -> AOS -> Timer0 计数器清零并重新启动;
 *   若在 RTB 个位时间内没有新字节, 计数器到达 CMPBR -> USART4 的 SR.RTOF 置位。
 *
 * 硬件限制(手册 11.4.7): TMR0_HTSSR 是**一个独立寄存器, 为 2 个单元的 Timer0 共有**,
 * 只能填一个事件编号。所以 USART1~USART4 里同一时刻只允许一个用硬件 RTO。
 * (工程里 PeripheralDriver/Hw_Uart3_DMA.c 也写了 USART3 + TMR02 A 通道的 RTO 版本,
 *  但它没有加入 uvprojx, 不参与编译; 若将来要用, 必须与本文件二选一。)
 * 前置条件: PWC_FCG0 的 AOS 时钟必须使能 (手册 22.3.3)。 */
#define UART4_RTO_TMR_TRIG_EVT  EVT_USART4_RI

/*---------------- 应用层拼包 (RX 帧续接) -----------------
 * 0 = 关(默认): 完全按 RTO 判帧, 一次超时就是一帧 —— 与原来的行为一致。
 * 1 = 开: RTO 到点时若这一帧"头是 68、第三字节是 00、但已收长度 < 长度域+8",
 *         就认为还没收全, 不清 DMA、接着往后收(最多续 UART4_RTO_REASSEMBLE_MAX 次),
 *         从而容忍帧内超过 RTB 的字节间隙。只有 68/00 开头的帧才会触发本逻辑,
 *         其它数据一字节都不受影响。
 * 打开后 UART4_FrameProc() 里会有续帧日志, 可据此判断对端间隙有多大。
 */
#ifndef UART4_RTO_REASSEMBLE
#define UART4_RTO_REASSEMBLE    (0)
#endif
#define UART4_RTO_REASSEMBLE_MAX (4u)

/* 停 RX/TX DMA 后等通道真正空闲(CHSTAT.CHACT 归零)的循环上限:
 * 手册 14.6 注2 明确"传输过程中写通道配置寄存器无效", 所以必须先等通道停稳。
 * CHACT 通常几个周期就归零, 这个上限只是防御。 */
#define UART4_DMA_IDLE_WAIT     (2000ul)

/*--------------------------- 运行统计 (排障用) -------------------------------*
 * 全部由中断/发送路径累加, 任务里 UART4_FrameProc() 会在有变化时打印一行。
 *   DropCnt     上一帧还没被任务取走, 新帧被迫丢弃的次数 (任务轮询太慢)
 *   OverflowCnt 一帧超过 UART4_RX_BUF_SIZE 被截断的次数
 *   ErrCnt      接收错误(ORE/FE/PE)次数
 *   DmaStallCnt  DMA 通道停不下来、或地址/计数两种监视寄存器结果不一致的次数 (正常为 0)
 *   TxFallbackCnt / TxFallbackBytes  TX DMA 没搬完、改用查询补齐的次数与字节数
 *   TxAbortCnt  上一帧没发完(等 TC 超时), 本次发送被放弃的次数
 *   ReasmCnt    应用层续帧次数 (仅 UART4_RTO_REASSEMBLE=1 时可能非 0)
 *---------------------------------------------------------------------------*/
extern volatile uint16_t u16Uart4RxLen;             /* 最近一帧长度 */
extern volatile uint8_t  u8Uart4FrameOK;            /* 1 = 有一帧待取, 取走清 0 */
extern volatile uint16_t u16Uart4DropCnt;
extern volatile uint16_t u16Uart4OverflowCnt;
extern volatile uint16_t u16Uart4ErrCnt;
extern volatile uint16_t u16Uart4DmaStallCnt;
extern volatile uint16_t u16Uart4TxFallbackCnt;
extern volatile uint32_t u32Uart4TxFallbackBytes;
extern volatile uint16_t u16Uart4TxAbortCnt;
extern volatile uint16_t u16Uart4ReasmCnt;

#endif /* UART4_MODE == USART4_DMA_RTO */

/*------------------------------- 对外接口 ------------------------------------*/
void Hw_Uart4_Init(void);

/* 发送 (DMA_RTO 模式走 TX DMA; NORMAL 模式为逐字节查询发送) */
void UART4_SendData(const uint8_t *pBuf, uint16_t u16Len);
void UART4_SendString(const char *str);

/* 取一帧数据 (仅 DMA_RTO 模式; 返回 1 表示取到, 0 表示无新帧) */
uint8_t UART4_GetFrame(uint8_t *pBuf, uint16_t u16MaxLen, uint16_t *pu16Len);

/* 直接取到已冻结帧的首地址 (不做拷贝, 避免大帧搬两遍)。
 * 返回 NULL 表示无新帧; 非 NULL 时 *pu16Len 给出帧长。
 * 注意: 取到后要尽快调 UART4_ReleaseFrame() 释放, 否则新帧会被丢弃。 */
const uint8_t *UART4_GetFramePtr(uint16_t *pu16Len);

/* 释放 UART4_GetFramePtr() 取到的帧 (不清数据, 只是让中断可以写新帧) */
void UART4_ReleaseFrame(void);

/* 任务里轮询(建议 10ms 一次): DMA_RTO 模式取出整帧, 按 UART4_APP_MODE 处理,
 * 并顺带打印一行运行统计; NORMAL 模式空操作 */
void UART4_FrameProc(void);

#endif
