/*******************************************************************************
 * Hw_Uart4.c  --  USART4 数据口驱动 (PB14 = USART4_TX / PB15 = USART4_RX)
 *
 * 由 Hw_Uart4.h 的 UART4_MODE 宏切换两种收发方式:
 *
 *  [USART_NORMAL]   中断逐字节收发 (原方式)
 *
 *  [USART4_DMA_RTO] RX DMA + 硬件 RTO 超时判帧 + TX DMA   <<< 默认
 *                   收: RI 事件经 AOS 触发 DMA 逐字节搬入乒乓缓冲;
 *                       一帧最后一字节后静默超过 RTO -> SR.RTOF -> RTO 中断
 *                       算帧长、只切缓冲指针(不拷贝)、重装 DMA 等下一帧;
 *                   发: 整帧由 TX DMA 搬运 (TI 事件触发), 带查询兜底。
 *
 *------------------------------------------------------------------------------
 * 依据 HC32F460 参考手册 Rev1.5 做的加固 (逐条对应手册章节):
 *
 *  A. 25.4.1.6 步骤 1~10 + 22.6 注1/注2
 *     - CMPBR = RTB*2^CKDIV - 4 已在手册原文核实 (CKDIVB=0 时 K=4), 代码不变。
 *     - 初始化顺序改为: 管脚/时钟/波特率 -> Timer0 -> DMA -> 注册中断 ->
 *       清 RTOF/RTOE/RTOIE -> 武装 RX DMA -> 最后才 RE/RIE/TE。原顺序把
 *       RTOE/RTOIE 放在 Timer0 和 DMA 之前, 上电时线上若有数据, 会在
 *       "超时中断已使能但 DMA 还没准备好"的窗口里进中断算出垃圾帧长。
 *     - 手册只在异步计数下列了两条写注意事项(先 ASYNCLK 后 SYNS; 写 CNT/CMP/
 *       CST/STFLR 需 3 个异步计数时钟生效), 并没有"连续写会丢第二次"的说法;
 *       但 CNTBR=0 本来就被 HCLEB 硬件清零覆盖, 属于多余动作, 已删除。
 *
 *  B. 22.5.3 BCONR 通道划分
 *     BCONR 的 b0~b15 是 A 通道、b16~b31 是 B 通道。原 RTO 中断里用
 *     "初始化时缓存的 BCONR" 整字回写来停表, 会把 A 通道的位一起写成陈旧值;
 *     如果 UART3(TMR02 A) 也开了 RTO, CSTA 会被这个回写清掉。
 *     现在改成读**当次活值**、只清 CSTB 再写回, 不再碰 A 通道。
 *     注册的 TIMER0_BaseInit/HardTriggerInit 会各自只改自己那半边。
 *     (半字访问 b16 的写法没有采用: 手册没有承诺 TMR0 寄存器支持半字访问,
 *      只对 DMA 寄存器明确要求 32bit, 32bit 读改写更稳。)
 *
 *  C. 11.4.7 TMR0_HTSSR 是 TMR01/TMR02 共有的单个寄存器, 只能填一个事件号,
 *     所以同一时刻只允许一个 USART 用硬件 RTO。工程里的 Hw_Uart3_DMA.c 写了
 *     第二个 RTO, 但它不在 uvprojx 里, 不参与编译。
 *
 *  D. RX 乒乓双缓冲 (原来 ISR 里停 DMA -> 拷 256 字节 -> 重启, 拷贝期间
 *     到达的字节会丢): 现在中断只切"已发布缓冲"指针, 拷贝交给任务,
 *     DMA 不停留; 待取帧所在的那块缓冲不会被 DMA 覆盖。
 *
 *  E. 接收错误处理 (原实现无条件 RxDmaRestart, 一个 FE/PE 就把半帧作废):
 *     现在只清错误标志 + 计数, 让这一帧走到 RTO 由上层(长度域+FCS)校验。
 *     DMA 缓冲写满的场景由 RTO 收尾并重新武装, 另有 OverflowCnt 计数。
 *
 *  F. 14.5.11 / 14.6 注2: 停 DMA 后先等 CHSTAT.CHACT[x] 归零再读剩余计数,
 *     因为"传输过程中写通道配置寄存器无效", 动作中读到的计数也不可靠。
 *
 *  G. 中断优先级: RTO(Int019) 与接收错误(Int018) 现在都是 PRIORITY_02。
 *     两者都会操作同一个 DMA 通道, 原来 RTO 能打断错误处理, 存在竞态。
 *
 *  H. TX: 发送前等 TC(而不是只等 TXE) 再启动, 避免"TE 0->1 触发 TI"时把
 *     上一帧最后一位截断; 增加 TxFallbackCnt/Bytes 统计, 用来判断 TX DMA
 *     路径是否真的在工作(TX 侧仍然要忙等到 TC 才算发完, 并没有省 CPU)。
 ******************************************************************************/
#include "hc32_ddl.h"
#include <stdio.h>
#include "Hw_Uart4.h"
#include "Manager_Protocol.h"   /* 收到一帧后的 APDU 协议处理 */

/* hc32f46x_utility.c 里实现, 但头文件中没有声明, 这里显式声明 */
extern void DebugOutput(uint8_t u8Data);

/* TX 等待超时保护 (防止异常时死等; 168MHz 下 200万次循环约 0.1s, 远大于
 * 256 字节 @115200 的 22ms 发送时间) */
#define UART4_TX_WAIT_LOOP      (2000000ul)

/*******************************************************************************
 * 公共: USART 基础初始化 (两种模式共用)
 *
 * 注意: 这里**不**使能 TE/RE。收发使能放到各自的 Hw_Uart4_Init() 最后一步,
 *       避免在 Timer0 / DMA / 中断向量都还没就位时就开始收字节。
 ******************************************************************************/
static void UART4_UsartBaseInit(void)
{
    stc_usart_uart_init_t stcUsartConf;
    stc_port_init_t Port_CFG;

    MEM_ZERO_STRUCT(stcUsartConf);
    MEM_ZERO_STRUCT(Port_CFG);

    stcUsartConf.enDataLength = UsartDataBits8;         /* 8 bit data       */
    stcUsartConf.enDirection  = UsartDataLsbFirst;      /* LSB first        */
    stcUsartConf.enParity     = UsartParityNone;        /* no parity        */
    stcUsartConf.enStopBit    = UsartOneStopBit;        /* 1 stop bit       */
    stcUsartConf.enSampleMode = UsartSamleBit8;         /* OVER8 = 1        */
    stcUsartConf.enDetectMode = UsartStartBitFallEdge;  /* RX 起始位下降沿  */

    /* 预分频 PSC 必须为 0 (PCLK1/1):
     * PCLK1 = 84MHz, OVER8=1 时 DIV = 84M/(115200*8) - 1 = 90.14 -> 整数90,
     * 配合 BRR 小数部分能得到接近精确的 115200。
     * 若用 UsartClkDiv_4 (PCLK1/4 = 21MHz), DIV 只能取 21, 实际波特率
     * 119318bps (误差 +3.6%), 收发都会出错。 */
    stcUsartConf.enClkDiv = UsartClkDiv_1;

#if (UART4_MODE == USART4_DMA_RTO)
    /* 手册 25.4.1.6 步骤3: 使用 RTO 时必须选内部时钟源并使能时钟输出,
     * 该时钟就是超时计数器(Timer0)的计数时钟 */
    stcUsartConf.enClkMode = UsartIntClkCkOutput;
#else
    stcUsartConf.enClkMode = UsartIntClkCkNoOutput;
#endif

    PWC_Fcg1PeriphClockCmd(USART4_CLK, Enable);

    /* PB14 -> USART4_TX */
    Port_CFG.enPinMode = Pin_Mode_Out;
    PORT_Init(USART4_TX_PORT, USART4_TX_PIN, &Port_CFG);
    PORT_SetBits(USART4_TX_PORT, USART4_TX_PIN);    /* 切到 USART 前先拉高, 避免误起始位 */
    /* PB15 -> USART4_RX */
    Port_CFG.enPinMode = Pin_Mode_In;
    PORT_Init(USART4_RX_PORT, USART4_RX_PIN, &Port_CFG);

    PORT_SetFunc(USART4_RX_PORT, USART4_RX_PIN, USART4_RX_FUNC, Disable);
    PORT_SetFunc(USART4_TX_PORT, USART4_TX_PIN, USART4_TX_FUNC, Disable);

    USART_UART_Init(USART4_UNIT, &stcUsartConf);
    USART_SetBaudrate(USART4_UNIT, USART4_BAUDRATE);
}

/* 注册指定中断源到指定向量
 * 注意: 中断源编号必须用 en_int_src_t(16 位宽)传递!
 *       USART4 的中断源编号是 293(EI)/294(RI)/297(RTO), 若形参用 uint8_t
 *       会被截断成 37/41, 注册到完全不相干的向量上去, 中断永远不响应。*/
static void UART4_IrqRegister(en_int_src_t enIntSrc, IRQn_Type enIrqn, func_ptr_t pfnCb, uint32_t u32Prio)
{
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    stcIrqRegiConf.enIntSrc    = enIntSrc;
    stcIrqRegiConf.enIRQn      = enIrqn;
    stcIrqRegiConf.pfnCallback = pfnCb;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_SetPriority(enIrqn, u32Prio);
    NVIC_ClearPendingIRQ(enIrqn);       /* 清掉初始化窗口里可能挂起的请求 */
    NVIC_EnableIRQ(enIrqn);
}

/*******************************************************************************
 * 模式一: USART_NORMAL  中断逐字节收发 (原方式)
 ******************************************************************************/
#if (UART4_MODE == USART_NORMAL)

static void USART4_RX_Callback(void)
{
    uint8_t data;
    data = (uint8_t)USART4_UNIT->DR_f.RDR;
    /* 数据口收到的字节直接转发到调试口(USART1) */
    DebugOutput(data);
}

static void USART4_RX_ERROR_Callback(void)
{
    if (USART4_UNIT->SR_f.ORE == 1u)        /* 上溢     */
    {
        USART4_UNIT->CR1_f.CORE = 1u;
    }
    if (USART4_UNIT->SR_f.FE == 1u)         /* 帧错误   */
    {
        USART4_UNIT->CR1_f.CFE = 1u;
    }
    if (USART4_UNIT->SR_f.PE == 1u)         /* 校验错误 */
    {
        USART4_UNIT->CR1_f.CPE = 1u;
    }
}

void Hw_Uart4_Init(void)
{
    UART4_UsartBaseInit();

    UART4_IrqRegister(USART4_RI_NUM, USART4_RX_IT_IRQn, USART4_RX_Callback, DDL_IRQ_PRIORITY_02);
    UART4_IrqRegister(USART4_EI_NUM, USART4_ER_IRQn,    USART4_RX_ERROR_Callback, DDL_IRQ_PRIORITY_03);

    USART_FuncCmd(USART4_UNIT, UsartRxInt, Enable);
    USART_FuncCmd(USART4_UNIT, UsartTx, Enable);
    USART_FuncCmd(USART4_UNIT, UsartRx, Enable);
}

/* NORMAL 模式无"帧"概念, 保持接口一致 */
void UART4_FrameProc(void)
{
}

uint8_t UART4_GetFrame(uint8_t *pBuf, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    (void)pBuf; (void)u16MaxLen; (void)pu16Len;
    return 0u;                              /* 仅 DMA_RTO 模式支持 */
}

const uint8_t *UART4_GetFramePtr(uint16_t *pu16Len)
{
    (void)pu16Len;
    return NULL;                            /* 仅 DMA_RTO 模式支持 */
}

void UART4_ReleaseFrame(void)
{
}

/* 逐字节查询发送 (带超时保护, 不会因为 TXE 异常而死等) */
void UART4_SendData(const uint8_t *pBuf, uint16_t u16Len)
{
    uint16_t i;
    uint32_t u32Guard;

    if ((NULL == pBuf) || (0u == u16Len))
    {
        return;
    }

    for (i = 0u; i < u16Len; i++)
    {
        u32Guard = 0u;
        while ((0u == USART4_UNIT->SR_f.TXE) && (u32Guard < UART4_TX_WAIT_LOOP))
        {
            u32Guard++;
        }
        USART_SendData(USART4_UNIT, pBuf[i]);
    }

    u32Guard = 0u;
    while ((0u == USART4_UNIT->SR_f.TC) && (u32Guard < UART4_TX_WAIT_LOOP))
    {
        u32Guard++;
    }
}

/*******************************************************************************
 * 模式二: USART4_DMA_RTO   RX DMA(乒乓) + RTO 不定长帧 + TX DMA
 ******************************************************************************/
#elif (UART4_MODE == USART4_DMA_RTO)

/*------------------------------ 运行统计 -------------------------------------*/
volatile uint16_t u16Uart4RxLen      = 0u;
volatile uint8_t  u8Uart4FrameOK     = 0u;
volatile uint16_t u16Uart4DropCnt     = 0u;
volatile uint16_t u16Uart4OverflowCnt = 0u;
volatile uint16_t u16Uart4ErrCnt      = 0u;
volatile uint16_t u16Uart4DmaStallCnt = 0u;
volatile uint16_t u16Uart4TxFallbackCnt = 0u;
volatile uint32_t u32Uart4TxFallbackBytes = 0ul;
volatile uint16_t u16Uart4TxAbortCnt  = 0u;
volatile uint16_t u16Uart4ReasmCnt    = 0u;

/* 统计有变化时置 1, 由任务打印一行 (中断里不做 printf) */
static volatile uint8_t u8Uart4WarnPending = 0u;

/*------------------------------ RX 乒乓双缓冲 --------------------------------*
 * gs_au8RxBuf[0/1]: DMA 交替写入的两块缓冲。
 *   u8Uart4DmaIdx = DMA 正在写的那一块
 *   u8Uart4PubIdx = 已经"发布"给任务的那一块 (u8Uart4FrameOK=1 时有效)
 * 发布时把 DmaIdx 取反, 所以待取帧所在的那一块永远不会被 DMA 覆盖。
 * 上一帧还没取走时来的新帧直接丢弃(计数), 并且**不**切换缓冲,
 * 让新数据原地覆盖被丢弃的那一帧, 这样也不用第三块缓冲。
 *---------------------------------------------------------------------------*/
static uint8_t gs_au8RxBuf[2][UART4_RX_BUF_SIZE];
/* 监视寄存器交叉校验不一致只提示一次, 避免每帧刷屏 */
static uint8_t  u8Uart4MonMismatchOnce = 0u;
static volatile uint8_t u8Uart4DmaIdx = 0u;
static volatile uint8_t u8Uart4PubIdx = 0u;

static uint8_t  UART4_TXbuff[UART4_TX_BUF_SIZE];

/* 用"地址监视寄存器"算 DMA 已经搬走的字节数
 * 手册 14.5.23: 监视寄存器在每完成一个数据块后更新为下一次传输的地址, 所以
 *     当前监视地址 - 起始地址 = 已搬字节数
 * 地址每搬一个字节前进一格, 语义最直接, 也不依赖"计数监视寄存器何时被装载"。
 * 返回 0xFFFF 表示监视地址不在 [u32Base, u32Base + u16Span] 内, 不可信。*/
static uint16_t UART4_DmaMovedByAddr(uint32_t u32MonAddr, uint32_t u32Base, uint16_t u16Span)
{
    if ((u32MonAddr < u32Base) || (u32MonAddr > (u32Base + (uint32_t)u16Span)))
    {
        return 0xFFFFu;
    }
    return (uint16_t)(u32MonAddr - u32Base);
}

/* 读 DMA 通道剩余传输计数, 取 DTCTLx.CNT 与 MONDTCTLx.CNT 的较小值。
 * 注意: 手册只写了"配置寄存器在传输前后保持不变"、监视寄存器"每完成一个数据块
 * 减 1", 并没有明写监视寄存器在通道启动时会不会从配置寄存器装载。本文件按
 * "会装载"使用, 但真正决定帧长/发送量的判据都优先走地址监视寄存器, 并且把两种
 * 算法算出来的结果交叉比对(不一致计一次 DmaStallCnt) —— 上板跑一跑就知道
 * 哪种解释与实际相符。 */
static uint16_t UART4_DmaRemain(M4_DMA_TypeDef *pstcDma, uint8_t u8Ch)
{
    uint16_t u16Cfg;
    uint16_t u16Mon;

    switch (u8Ch)
    {
        case DmaCh0:
            u16Cfg = (uint16_t)pstcDma->DTCTL0_f.CNT;
            u16Mon = (uint16_t)pstcDma->MONDTCTL0_f.CNT;
            break;
        case DmaCh1:
            u16Cfg = (uint16_t)pstcDma->DTCTL1_f.CNT;
            u16Mon = (uint16_t)pstcDma->MONDTCTL1_f.CNT;
            break;
        case DmaCh2:
            u16Cfg = (uint16_t)pstcDma->DTCTL2_f.CNT;
            u16Mon = (uint16_t)pstcDma->MONDTCTL2_f.CNT;
            break;
        default:
            u16Cfg = (uint16_t)pstcDma->DTCTL3_f.CNT;
            u16Mon = (uint16_t)pstcDma->MONDTCTL3_f.CNT;
            break;
    }

    return (u16Mon < u16Cfg) ? u16Mon : u16Cfg;
}

/* 等 DMA 通道真正空闲 (手册 14.5.11 CHSTAT.CHACT[x]: 1 = 该通道处于动作中)。
 * 手册 14.6 注2 明确"传输过程中写通道配置寄存器无效", 所以重编程前必须等它停。*/
static void UART4_DmaWaitIdle(M4_DMA_TypeDef *pstcDma, uint8_t u8Ch)
{
    uint32_t u32Guard = 0u;

    while ((0u != ((((uint32_t)pstcDma->CHSTAT_f.CHACT) >> u8Ch) & 1u)) &&
           (u32Guard < UART4_DMA_IDLE_WAIT))
    {
        u32Guard++;
    }
    if (0u != ((((uint32_t)pstcDma->CHSTAT_f.CHACT) >> u8Ch) & 1u))
    {
        u16Uart4DmaStallCnt++;              /* 理论为 0, 非 0 说明 DMA 卡住 */
        u8Uart4WarnPending = 1u;
    }
}

/* 重装并启动 RX DMA: 从 gs_au8RxBuf[u8Idx] 的 u16Offset 处起, 收到缓冲末尾为止。
 * u16Offset 非 0 只在应用层续帧(UART4_RTO_REASSEMBLE)时出现。*/
static void UART4_RxDmaArm(uint8_t u8Idx, uint16_t u16Offset)
{
    if (u16Offset >= UART4_RX_BUF_SIZE)
    {
        u16Offset = 0u;
    }

    DMA_ChannelCmd(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, Disable);
    UART4_DmaWaitIdle(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH);
    DMA_ClearIrqFlag(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, TrnCpltIrq);
    /* CNT 必须 >= 1: 设为 0 表示无限次传输(手册 14.5.14) */
    DMA_SetTransferCnt(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH,
                       (uint16_t)(UART4_RX_BUF_SIZE - u16Offset));
    DMA_SetDesAddress(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH,
                      (uint32_t)(&gs_au8RxBuf[u8Idx][u16Offset]));
    DMA_ChannelCmd(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, Enable);
}

/* 停 RX DMA 并返回"还剩几个字节没搬" (调用后通道处于停止态)
 *
 * 本函数决定"这一帧多少字节", 一旦算错上层就会拿到长度错误的报文, 所以:
 *   1) 先等通道真正停稳(手册 14.6 注2: 传输过程中监视寄存器没有意义), 再读;
 *   2) 主判据用目标地址监视寄存器 MONDAR —— RX 的目标地址递增, 每搬一字节前进
 *      一格, 已搬字节数 = MONDAR - 缓冲首地址, 不依赖计数监视寄存器;
 *   3) 计数监视寄存器只做交叉校验, 不一致时计一次异常但并不改变帧长。 */
static uint16_t UART4_RxDmaStopAndRemain(void)
{
    uint16_t u16RemainCnt;
    uint16_t u16MovedFromBase;
    uint16_t u16MovedByCnt;

    DMA_ChannelCmd(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, Disable);
    UART4_DmaWaitIdle(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH);

    u16MovedFromBase = UART4_DmaMovedByAddr((uint32_t)UART4_RX_DMA_UNIT->MONDAR2,
                                            (uint32_t)(&gs_au8RxBuf[u8Uart4DmaIdx][0]),
                                            UART4_RX_BUF_SIZE);
    u16RemainCnt = UART4_DmaRemain(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH);

    if (0xFFFFu == u16MovedFromBase)
    {
        /* 地址监视不在缓冲范围内: 说明 MONDAR 也没按预期更新, 这时只能退回计数法,
         * 但一定要计数 —— 帧长可能已经不可信, 上板必须先看到这个数。 */
        u16Uart4DmaStallCnt++;
        u8Uart4WarnPending = 1u;
        return u16RemainCnt;
    }

    u16MovedByCnt = (uint16_t)(UART4_RX_BUF_SIZE - u16RemainCnt);
    if (u16MovedByCnt != u16MovedFromBase)
    {
        /* 两种监视寄存器对"已搬字节数"的结论不一致: 说明本文件对"监视寄存器何时从
         * 配置寄存器装载"的假设与实际不符。帧长仍按地址法算(MONDAR 的语义最直接),
         * 但计数会一直累加, 任何一次统计打印都能看到 dmaStall != 0。
         * 打印只做一次, 免得每帧都刷一行。 */
        u16Uart4DmaStallCnt++;
        if (0u == u8Uart4MonMismatchOnce)
        {
            u8Uart4MonMismatchOnce = 1u;
            u8Uart4WarnPending     = 1u;
        }
    }

    return (uint16_t)(UART4_RX_BUF_SIZE - u16MovedFromBase);
}

#if (UART4_RTO_REASSEMBLE)
/*---------------------------- 应用层续帧判定 ---------------------------------*
 * 只在"头看起来就是本协议的帧、但长度域说明还没收全"时返回 1。
 * 判据取 68 | L | 00 三个字节 (工程里 Manager / DL-T 698.45 帧都是这个头)。
 * 返回 1 = 还需要继续收; 0 = 这一帧就当完整帧处理。
 *---------------------------------------------------------------------------*/
static uint8_t UART4_FrameNeedMore(const uint8_t *pBuf, uint16_t u16Len)
{
    uint16_t u16Need;

    if (u16Len < 3u)
    {
        return 0u;                          /* 太短, 无从判断 */
    }
    if ((0x68u != pBuf[0]) || (0x00u != pBuf[2]))
    {
        return 0u;                          /* 帧头不对, 不是本协议 */
    }

    u16Need = (uint16_t)((uint16_t)pBuf[1] + 8u);   /* L = 数据域字节数, 总长 = L + 8 */
    if ((u16Need > u16Len) && (u16Need <= UART4_RX_BUF_SIZE))
    {
        return 1u;
    }
    return 0u;
}
#endif /* UART4_RTO_REASSEMBLE */

/*******************************************************************************
 * RTO 超时中断: 一帧接收结束 (手册 25.4.1.6)
 *
 * 手册步骤10 的顺序: 检测到 TIMEOUT 后 CSTB=0 停表, 再写 CR1.CRTOF 清 SR.RTOF。
 * 必须先停表: 若先清 RTOF 而计数器仍在比较值之上, 标志会立刻又被置起来。
 * (手册 22.6 注2: 异步计数下写 CSTB 要经过 3 个异步计数时钟才生效, 约 26us,
 *  这段时间里标志可能被再置一次, 所以计时器停稳前不要依赖 RTOF 的状态。
 *  我们不在这里轮询等待 —— 3 个计数时钟的忙等放进中断太长。)
 ******************************************************************************/
static void USART4_RTO_Callback(void)
{
    uint16_t u16Len;
    uint16_t u16Remain;
    uint32_t u32Bconr;

    /* 1. 停超时计数器。
     *    BCONR 的 b0~b15 是 TMR02 的 A 通道(UART3 用), b16 才是 UART4 用的 CSTB。
     *    这里读**当次活值**、只清 CSTB 再写回, 绝不回写缓存值——
     *    否则会把 A 通道的 CSTA 一起写成陈旧值, 把 UART3 的 RTO 表停掉。 */
    u32Bconr  = M4_TMR02->BCONR;
    u32Bconr &= ~UART4_RTO_BCONR_CSTB;
    M4_TMR02->BCONR = u32Bconr;

    /* 清 Timer0 自己的比较匹配标志 (手册 22.5.4 STFLR.CMBF, 写 0 清) */
    UART4_RTO_TMR_UNIT->STFLR_f.CMBF = 0u;

    /* 2. 清 USART 的 TIMEOUT 标志 (写 CR1.CRTOF 清 SR.RTOF) */
    USART4_UNIT->CR1_f.CRTOF = 1u;

    /* 3. 停 RX DMA, 等通道空闲后再由剩余计数算出本帧长度 */
    u16Remain = UART4_RxDmaStopAndRemain();
    u16Len    = (uint16_t)(UART4_RX_BUF_SIZE - u16Remain);
    if (u16Len > UART4_RX_BUF_SIZE)         /* 防御: 计数异常时当空帧 */
    {
        u16Len = 0u;
    }

    if (0u == u16Len)
    {
        /* 线上噪声引起的空超时: 不提交, 直接重新武装继续等 */
        UART4_RxDmaArm(u8Uart4DmaIdx, 0u);
        return;
    }

    if (u16Len >= UART4_RX_BUF_SIZE)
    {
        u16Uart4OverflowCnt++;              /* 一帧超过缓冲上限, 被截断 */
        u8Uart4WarnPending = 1u;
    }

#if (UART4_RTO_REASSEMBLE)
    /* 应用层续帧: 明显还没收全就接着往后收, 先不发布 */
    if ((0u == u8Uart4FrameOK) &&
        (0u != UART4_FrameNeedMore(gs_au8RxBuf[u8Uart4DmaIdx], u16Len)) &&
        (u16Uart4ReasmCnt < UART4_RTO_REASSEMBLE_MAX))
    {
        u16Uart4ReasmCnt++;
        UART4_RxDmaArm(u8Uart4DmaIdx, u16Len);
        return;
    }
#endif

    if (0u == u8Uart4FrameOK)
    {
        /* 发布: 任务去读 u8Uart4PubIdx 那一块, 中断不再碰它 */
        u8Uart4PubIdx  = u8Uart4DmaIdx;
        u16Uart4RxLen  = u16Len;
        u8Uart4FrameOK = 1u;

        u8Uart4DmaIdx ^= 1u;                /* 切到另一块, 待取帧不会被覆盖 */
        UART4_RxDmaArm(u8Uart4DmaIdx, 0u);
    }
    else
    {
        /* 上一帧还没被任务取走: 丢掉这一帧, 原地重收(覆盖被丢的数据),
         * 缓冲指针不动, 所以正在等任务取走的那一块始终安全。 */
        u16Uart4DropCnt++;
        u8Uart4WarnPending = 1u;
        UART4_RxDmaArm(u8Uart4DmaIdx, 0u);
    }
}

/*******************************************************************************
 * 接收错误中断 (ORE / FE / PE)
 *
 * 手册 25.4.1.3: 发生任何一种接收错误都不能再进行数据接收, 清掉对应标志即可恢复。
 * 原实现清完标志后无条件 RxDmaRestart(), 相当于一个线上毛刺就把已收到的半帧
 * 整块作废、缓冲清零。现在只清标志 + 计数, 让这一帧走到 RTO 交给上层
 * (本工程的 Manager 协议有长度域和 FCS)去判断, 由上层决定丢不丢。
 *
 * 唯一的例外是 RX DMA 缓冲写满: 此时 DMA_CHEN 已被硬件自动清零(手册 14.5.8),
 * 后面的字节会一直撞 ORE, RI 也不再产生 —— 不用管, RTO 计数器到点后会超时,
 * RTO 中断里会把满帧发布出来并重新武装, 自己就能恢复。
 ******************************************************************************/
static void USART4_RX_ERROR_Callback(void)
{
    if (1u == USART4_UNIT->SR_f.ORE)        /* 上溢: RDR 没被读走又来新数据 */
    {
        USART4_UNIT->CR1_f.CORE = 1u;
        u16Uart4ErrCnt++;
        u8Uart4WarnPending = 1u;
    }
    if (1u == USART4_UNIT->SR_f.FE)         /* 帧错误: 停止位为低 */
    {
        USART4_UNIT->CR1_f.CFE = 1u;
        u16Uart4ErrCnt++;
        u8Uart4WarnPending = 1u;
    }
    if (1u == USART4_UNIT->SR_f.PE)         /* 校验错误 */
    {
        USART4_UNIT->CR1_f.CPE = 1u;
        u16Uart4ErrCnt++;
        u8Uart4WarnPending = 1u;
    }
}

/*------------------------------------------------------------------------------
 * RTO 超时计数器初始化 (Timer0 Unit2 B 通道 + USART4 波特率时钟)
 * 手册 25.4.1.6 步骤 7/8
 *----------------------------------------------------------------------------*/
static void UART4_RtoTimerInit(void)
{
    stc_tim0_base_init_t    stcTim0Cfg;
    stc_tim0_trigger_init_t stcTim0Trig;

    MEM_ZERO_STRUCT(stcTim0Cfg);
    MEM_ZERO_STRUCT(stcTim0Trig);

    PWC_Fcg2PeriphClockCmd(UART4_RTO_TMR_CLK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);   /* TMR0 硬件触发需 AOS 时钟 */

    /* 步骤7: 先关通道 (CSTB=0) */
    TIMER0_Cmd(UART4_RTO_TMR_UNIT, UART4_RTO_TMR_CH, Disable);

    /* 步骤8: 计数方式 + 异步时钟源 + 分频 + 比较值
     * 手册 22.6 注1: 异步计数要先设 ASYNCLK 再设 SYNS, 然后才启动 ——
     * TIMER0_BaseInit() 内部就是这个顺序(先 ASYNCLKB/CKDIVB, 后 SYNSB)。
     * 手册 22.5.3 ASYNCLKB 注: "当使能 UART 对应通道的超时功能时, 时钟源不再是
     * XTAL32, 而是 UART 波特率生成器产生的时钟 tick", 正是我们要的,
     * 这样 CMPBR 就能直接按"接收位数"计算, 与波特率无关。*/
    stcTim0Cfg.Tim0_CounterMode      = Tim0_Async;               /* SYNSB = 1 */
    stcTim0Cfg.Tim0_AsyncClockSource = Tim0_XTAL32;              /* ASYNCLKB = 1 */
    stcTim0Cfg.Tim0_ClockDivision    = UART4_RTO_CKDIV;          /* CKDIVB = 0 */
    stcTim0Cfg.Tim0_CmpValue         = UART4_RTO_CMP_VALUE;      /* = RTB - 4 */
    TIMER0_BaseInit(UART4_RTO_TMR_UNIT, UART4_RTO_TMR_CH, &stcTim0Cfg);

    /* 硬件触发: CAPMDB=0 比较输出, HCLEB=1 硬件清零, HSTAB=1 硬件启动
     * (手册步骤8: HCLEA<B>=1, HSTAA<B>=1, ASYNCLKA<B>=1, SYNSA<B>=1) */
    stcTim0Trig.Tim0_OCMode       = Tim0_OutputCapare;   /* CAPMDB = 0 */
    stcTim0Trig.Tim0_SelTrigSrc   = UART4_RTO_TMR_TRIG_EVT;  /* AOS: 收到字节(RI)时清零并重启 */
    stcTim0Trig.Tim0_InTrigEnable = Disable;
    stcTim0Trig.Tim0_InTrigClear  = Enable;              /* HCLEB = 1 */
    stcTim0Trig.Tim0_InTrigStop   = Disable;
    stcTim0Trig.Tim0_InTrigStart  = Enable;              /* HSTAB = 1 */
    TIMER0_HardTriggerInit(UART4_RTO_TMR_UNIT, UART4_RTO_TMR_CH, &stcTim0Trig);

    /* 关键(手册 22.3.3): Timer0 的硬件触发源由 TMR0_HTSSR.TRGSEL 指定事件编号。
     * 这里整字写入: TRGSEL = EVT_USART4_RI, COMEN = 0 (不用公共触发源)。
     * 旧版把它写成 0x1FF("无事件")是错的, 会让 RTO 计数器永远不被清零重启,
     * 结果 SR.RTOF 永不置位 -> 收到数据后一帧都取不出来。
     * 注意(手册 11.4.7): 这个寄存器为 2 个 Timer0 单元共有, 只能有一个 USART 用 RTO。*/
    M4_AOS->TMR0_HTSSR = (uint32_t)UART4_RTO_TMR_TRIG_EVT;

    TIMER0_WriteCntReg(UART4_RTO_TMR_UNIT, UART4_RTO_TMR_CH, 0u);

    /* 说明: BCONR 的 B 通道各控制位(ASYNCLKB/SYNSB/CKDIVB/HSTAB/HCLEB/CAPMDB)
     * 已由上面两次 DDL 调用按"只动自己那半边"的方式写好, 这里不再整字重建。 */
}

/*------------------------------------------------------------------------------
 * RX DMA: USART4_RI 事件(AOS)触发, 逐字节搬入乒乓缓冲
 *----------------------------------------------------------------------------*/
static void UART4_RxDmaInit(void)
{
    stc_dma_config_t stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    stcDmaCfg.u16BlockSize    = 1u;                     /* 1 次请求搬 1 字节 */
    stcDmaCfg.u16TransferCnt  = UART4_RX_BUF_SIZE;

    stcDmaCfg.u32SrcAddr      = ((uint32_t)(&USART4_UNIT->DR) + 2u);    /* RDR (DR 高半字) */
    stcDmaCfg.u32DesAddr      = (uint32_t)gs_au8RxBuf[0];

    stcDmaCfg.u16SrcRptSize   = 0u;
    stcDmaCfg.u16DesRptSize   = 0u;

    stcDmaCfg.stcDmaChCfg.enLlpEn    = Disable;
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enSrcInc   = AddressFix;      /* RDR 地址固定 */
    stcDmaCfg.stcDmaChCfg.enDesInc   = AddressIncrease; /* 缓冲地址递增 */
    stcDmaCfg.stcDmaChCfg.enIntEn    = Disable;         /* 用 RTO 判帧, 不用 DMA 中断 */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

    PWC_Fcg0PeriphClockCmd(UART4_DMA_CLK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);    /* AOS 触发必须开 */

    DMA_Cmd(UART4_RX_DMA_UNIT, Enable);
    DMA_InitChannel(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, &stcDmaCfg);
    DMA_SetTriggerSrc(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, EVT_USART4_RI);
    DMA_ClearIrqFlag(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, TrnCpltIrq);
    DMA_ChannelCmd(UART4_RX_DMA_UNIT, UART4_RX_DMA_CH, Disable);
}

/*------------------------------------------------------------------------------
 * TX DMA: USART4_TI(TXE) 事件触发, 由 UART4_TXbuff 搬到 TDR
 *----------------------------------------------------------------------------*/
static void UART4_TxDmaInit(void)
{
    stc_dma_config_t stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    stcDmaCfg.u16BlockSize    = 1u;
    stcDmaCfg.u16TransferCnt  = 1u;                     /* 每次发送时重设 */

    stcDmaCfg.u32SrcAddr      = (uint32_t)UART4_TXbuff;
    stcDmaCfg.u32DesAddr      = (uint32_t)(&USART4_UNIT->DR);       /* TDR */

    stcDmaCfg.u16SrcRptSize   = 0u;
    stcDmaCfg.u16DesRptSize   = 0u;

    stcDmaCfg.stcDmaChCfg.enLlpEn    = Disable;
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Disable;
    stcDmaCfg.stcDmaChCfg.enSrcInc   = AddressIncrease; /* 缓冲地址递增 */
    stcDmaCfg.stcDmaChCfg.enDesInc   = AddressFix;      /* TDR 地址固定 */
    /* 打开"传输完成"中断使能位, 是为了让 INTSTAT 的 TC 标志可靠置位,
     * 作为"整帧已搬完"的判据。这里不把 DMA 通道中断注册到 NVIC,
     * 所以不会真的产生中断。 */
    stcDmaCfg.stcDmaChCfg.enIntEn    = Enable;
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

    PWC_Fcg0PeriphClockCmd(UART4_DMA_CLK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    DMA_Cmd(UART4_TX_DMA_UNIT, Enable);
    DMA_InitChannel(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, &stcDmaCfg);
    DMA_SetTriggerSrc(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, EVT_USART4_TI);
    DMA_ClearIrqFlag(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, TrnCpltIrq);
    DMA_ChannelCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, Disable);
}

/*******************************************************************************
 * 初始化总入口
 *
 * 顺序按手册 25.4.1.6 的步骤 1~9, 但把"使能 RTOE/RTOIE"(步骤6)推迟到
 * Timer0 / DMA / 中断向量都就位之后 —— 否则上电瞬间线上有数据时, 会进到
 * "超时中断已使能、但 DMA 还没武装"的窗口, 算出垃圾帧长并发布出去。
 * 最后一步才使能收发器(RE/TE)与 RIE(步骤9), 这时整条链路已经是完整的。
 ******************************************************************************/
void Hw_Uart4_Init(void)
{
    UART4_UsartBaseInit();                  /* 步骤 1~5: 管脚 / 时钟 / CR / 波特率 */

    UART4_RtoTimerInit();                   /* 步骤 7~8: Timer0 超时计数器 */
    UART4_RxDmaInit();                      /* RX DMA 通道 */
    UART4_TxDmaInit();                      /* TX DMA 通道 */

    /* 中断向量先就位: RTO 与接收错误同一优先级 (都会动同一个 DMA 通道) */
    UART4_IrqRegister(USART4_EI_NUM,  USART4_ER_IRQn,     USART4_RX_ERROR_Callback, DDL_IRQ_PRIORITY_02);
    UART4_IrqRegister(USART4_RTO_NUM, USART4_RX_IT_IRQn,  USART4_RTO_Callback,      DDL_IRQ_PRIORITY_02);

    /* 步骤6: 先清 RTOF, 再使能超时 + 超时中断 */
    USART4_UNIT->CR1_f.CRTOF = 1u;
    USART_FuncCmd(USART4_UNIT, UsartTimeOut,    Enable);    /* CR1.RTOE  = 1 */
    USART_FuncCmd(USART4_UNIT, UsartTimeOutInt, Enable);    /* CR1.RTOIE = 1 */

    UART4_RxDmaArm(0u, 0u);                                 /* 武装 RX DMA 等第一帧 */

    /* 步骤9: 使能接收器与接收中断。
     * RIE 同时是 RI(字节收满) 和 REI(接收错误) 的使能位(手册表25-4)。打开它的
     * 目的是保证 RI 事件持续产生、供 AOS 触发 RX DMA 搬字节, 并让接收错误能走
     * Int018 恢复。RI 事件本身没有注册到任何向量(Int019 分给了 RTO), 所以
     * 每收一个字节不会进中断, 没有额外 CPU 负担。 */
    USART_FuncCmd(USART4_UNIT, UsartRxInt, Enable);
    USART_FuncCmd(USART4_UNIT, UsartRx,    Enable);
    USART_FuncCmd(USART4_UNIT, UsartTx,    Enable);
}

/*******************************************************************************
 * DMA 发送
 *
 * 关键点: 无论 DMA 是否被成功触发, 都必须保证整帧字节全部从 PB14 发出去。
 * 所以流程是 "DMA 搬大部分 + 查询补齐没搬完的部分", 而不是二选一。
 *
 * 说明: 这里最后要忙等到 TC 才算"发完", 所以 TX 用 DMA 并没有省 CPU,
 *       只是把逐字节的搬运交给硬件。真正省 CPU 的做法是把"等 TC"也移到中断里
 *       (TCIE + 发送完成中断), 但那样调用方就不能在返回后立刻复用缓冲,
 *       接口语义会变, 暂时保留阻塞语义。
 ******************************************************************************/

/* 等 TDR 空 (带超时保护, 避免 TXE 异常时死等) */
static void UART4_TxWaitReady(void)
{
    uint32_t u32Guard = 0u;

    while ((0u == USART4_UNIT->SR_f.TXE) && (u32Guard < UART4_TX_WAIT_LOOP))
    {
        u32Guard++;
    }
}

/* 等整帧真正移位发完 (SR.TC)。返回 1 = 已完成。
 * 必须用 TC 而不是 TXE: TXE 只表示 TDR 空了, 最后一个字节可能还在移位寄存器里。*/
static uint8_t UART4_TxWaitComplete(void)
{
    uint32_t u32Guard = 0u;

    while ((0u == USART4_UNIT->SR_f.TC) && (u32Guard < UART4_TX_WAIT_LOOP))
    {
        u32Guard++;
    }
    return (0u != USART4_UNIT->SR_f.TC) ? 1u : 0u;
}

/* TX DMA 已经搬进 TDR 的字节数 (0 ~ u16Programmed)
 * 判据一: DMA 的"传输完成"标志。它置位就一定是整帧都搬完了 —— 手册 14.5.8:
 *         传输次数用完后 DMA_CHEN 被硬件自动清零, 同时置"传输完成"标志,
 *         这个判据最可靠。
 * 判据二: 源地址监视寄存器 MONSAR。TX 的源地址递增, 每搬一字节前进一格,
 *         已搬字节数 = MONSAR3 - UART4_TXbuff。
 * 两个都拿不到可信值时返回 0, 交给调用方用查询补齐 —— 宁可多发也不能少发。*/
static uint16_t UART4_TxDmaDone(uint16_t u16Programmed)
{
    uint16_t u16Moved;

    if (0u == u16Programmed)
    {
        return 0u;
    }

    if (0u != DMA_GetIrqFlag(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, TrnCpltIrq))
    {
        return u16Programmed;
    }

    u16Moved = UART4_DmaMovedByAddr((uint32_t)UART4_TX_DMA_UNIT->MONSAR3,
                                    (uint32_t)UART4_TXbuff,
                                    u16Programmed);
    if (0xFFFFu == u16Moved)
    {
        return 0u;
    }

    return u16Moved;
}

void UART4_SendData(const uint8_t *pBuf, uint16_t u16Len)
{
    uint16_t i;
    uint16_t u16Done;
    uint32_t u32Guard;

    if ((NULL == pBuf) || (0u == u16Len) || (u16Len > UART4_TX_BUF_SIZE))
    {
        return;
    }

    /*---- 0) 上一帧必须真正发完 ----
     * 下面要用 "TE 由 0 变 1" 产生一次 TI(TXE) 事件来触发 DMA 起步。
     * 如果上一帧最后一位还在发送移位寄存器里, 这一步会把它截断成半位。
     * 所以这里等 TC(整帧发完), 而不是像原来那样只等 TXE。 */
    if (0u == UART4_TxWaitComplete())
    {
        u16Uart4TxAbortCnt++;               /* 上一帧异常未完成, 放弃本帧 */
        u8Uart4WarnPending = 1u;
        return;
    }

    for (i = 0u; i < u16Len; i++)
    {
        UART4_TXbuff[i] = pBuf[i];
    }

    /*---- 1) 整帧交给 TX DMA ----*/
    DMA_ChannelCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, Disable);
    UART4_DmaWaitIdle(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH);
    DMA_ClearIrqFlag(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, TrnCpltIrq);
    DMA_SetSrcAddress(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, (uint32_t)UART4_TXbuff);
    DMA_SetTransferCnt(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, u16Len);
    DMA_ChannelCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, Enable);

    /* TE 0->1 产生一次 TI(TXE) 事件, 经 AOS 触发 DMA 开始搬运
     * (与工程里 Hw_DMA_TX_UART3.c 的可用实现完全一致) */
    USART_FuncCmd(USART4_UNIT, UsartTx, Disable);
    USART_FuncCmd(USART4_UNIT, UsartTx, Enable);

    /*---- 2) 等 DMA 把整帧搬进 TDR ----*/
    u32Guard = 0u;
    while ((u16Len != UART4_TxDmaDone(u16Len)) && (u32Guard < UART4_TX_WAIT_LOOP))
    {
        u32Guard++;
    }

    /* 没等完说明 DMA 要么压根没被触发、要么半途停了。此时先把通道停稳再取监视
     * 值(手册 14.6 注2: 传输过程中监视寄存器没有意义), 才能算准还差多少字节。*/
    if (u16Len != UART4_TxDmaDone(u16Len))
    {
        DMA_ChannelCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, Disable);
        UART4_DmaWaitIdle(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH);
    }

    u16Done = UART4_TxDmaDone(u16Len);
    DMA_ChannelCmd(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH, Disable);
    UART4_DmaWaitIdle(UART4_TX_DMA_UNIT, UART4_TX_DMA_CH);

    /*---- 3) 兜底: DMA 没搬完的字节 (含完全没搬的情况) 用查询补齐 ----
     * 兜底本身会掩盖 "TX DMA 触发失效" 这种故障, 所以统计走了兜底的字节数:
     * 这个数一直为 0 才说明 TX DMA 路径真的在工作。 */
    if (u16Done < u16Len)
    {
        u16Uart4TxFallbackCnt++;
        u32Uart4TxFallbackBytes += (uint32_t)(u16Len - u16Done);
        u8Uart4WarnPending = 1u;

        for (i = u16Done; i < u16Len; i++)
        {
            UART4_TxWaitReady();
            USART_SendData(USART4_UNIT, UART4_TXbuff[i]);
        }
    }

    /*---- 4) 等最后一位真正移位发完 ----
     * 手册 25.5.1 SR.TC: 置位条件是"送出一帧数据的最后一位且发送数据寄存器没有被
     * 更新"; 清零条件是"TE=1 时向发送数据寄存器写入发送数据"。所以只要上面往 TDR
     * 写过至少一个字节(不管走 DMA 还是走兜底查询), TC 就已经被硬件清 0, 这里等它
     * 重新置 1 才真的是在等帧尾 —— 这正是本步骤存在的意义。
     * 另: "TE 由 0 变化为 1 时 TC 保持为 1", 所以不能靠切 TE 去复位 TC。*/
    (void)UART4_TxWaitComplete();
}

/*******************************************************************************
 * 取一帧 (任务里调用)
 ******************************************************************************/
const uint8_t *UART4_GetFramePtr(uint16_t *pu16Len)
{
    uint16_t u16Len;

    if (0u == u8Uart4FrameOK)
    {
        if (NULL != pu16Len)
        {
            *pu16Len = 0u;
        }
        return NULL;
    }

    u16Len = u16Uart4RxLen;
    if (u16Len > UART4_RX_BUF_SIZE)
    {
        u16Len = UART4_RX_BUF_SIZE;
    }
    if (NULL != pu16Len)
    {
        *pu16Len = u16Len;
    }

    return gs_au8RxBuf[u8Uart4PubIdx];
}

void UART4_ReleaseFrame(void)
{
    u8Uart4FrameOK = 0u;        /* 释放后中断才能发布新帧 */
}

uint8_t UART4_GetFrame(uint8_t *pBuf, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    const uint8_t *pFrame;
    uint16_t       u16Len = 0u;
    uint16_t       i;

    pFrame = UART4_GetFramePtr(&u16Len);
    if (NULL == pFrame)
    {
        if (NULL != pu16Len)
        {
            *pu16Len = 0u;
        }
        return 0u;
    }

    if (u16Len > u16MaxLen)
    {
        u16Len = u16MaxLen;
    }
    for (i = 0u; i < u16Len; i++)
    {
        pBuf[i] = pFrame[i];
    }
    if (NULL != pu16Len)
    {
        *pu16Len = u16Len;
    }

    UART4_ReleaseFrame();

    return 1u;
}

#if (UART4_APP_MODE == UART4_APP_ECHO)
/*******************************************************************************
 * 收到一帧后从 UART4(PB14) 回发应答: "RCV[长度]:数据\r\n"
 *   例: 收到 80 个 "0123...89"  ->  回发 "RCV[80]:0123...89\r\n"
 *
 * 长度最长 256, 加上头尾会超过 UART4_TX_BUF_SIZE, 所以按
 *   头  ->  数据(按 TX 缓冲上限分块)  ->  尾
 * 分几次顺序发送。UART4_SendData 是阻塞发送(等 TC 才返回), 段与段之间只有
 * 几条指令的间隔, 远小于一个字节时间(115200 下约 87us), 接收端看到的仍是
 * 连续的一帧, 不会被拆开。
 ******************************************************************************/
static void UART4_ReplyFrame(const uint8_t *pBuf, uint16_t u16Len)
{
    char     szHead[16];
    uint8_t  u8Head = 0u;
    uint16_t u16Off;
    uint16_t u16Chunk;

    if (u16Len > UART4_RX_BUF_SIZE)
    {
        u16Len = UART4_RX_BUF_SIZE;
    }

    /* ---- 头: "RCV[" ---- */
    szHead[u8Head++] = 'R';
    szHead[u8Head++] = 'C';
    szHead[u8Head++] = 'V';
    szHead[u8Head++] = '[';

    /* ---- 十进制长度 (0~256, 最多 3 位, 不依赖 sprintf) ---- */
    if (u16Len >= 100u)
    {
        szHead[u8Head++] = (char)('0' + ((u16Len / 100u) % 10u));
    }
    if (u16Len >= 10u)
    {
        szHead[u8Head++] = (char)('0' + ((u16Len / 10u) % 10u));
    }
    szHead[u8Head++] = (char)('0' + (u16Len % 10u));

    /* ---- 头: "]:" ---- */
    szHead[u8Head++] = ']';
    szHead[u8Head++] = ':';

    UART4_SendData((const uint8_t *)szHead, (uint16_t)u8Head);

    /* ---- 数据本体 (超长时分块) ---- */
    u16Off = 0u;
    while (u16Off < u16Len)
    {
        u16Chunk = (uint16_t)(u16Len - u16Off);
        if (u16Chunk > UART4_TX_BUF_SIZE)
        {
            u16Chunk = UART4_TX_BUF_SIZE;
        }
        UART4_SendData(&pBuf[u16Off], u16Chunk);
        u16Off = (uint16_t)(u16Off + u16Chunk);
    }

    /* ---- 尾: CRLF ---- */
    UART4_SendData((const uint8_t *)"\r\n", 2u);
}
#endif /* UART4_APP_MODE == UART4_APP_ECHO */

/* 运行统计有变化时打一行 (在任务上下文调用, 不在中断里 printf) */
static void UART4_LogStats(void)
{
    u8Uart4WarnPending = 0u;
    printf("[UART4] err=%u ovf=%u drop=%u dmaStall=%u txFallback=%u/%u txAbort=%u reasm=%u\r\n",
           (unsigned int)u16Uart4ErrCnt,
           (unsigned int)u16Uart4OverflowCnt,
           (unsigned int)u16Uart4DropCnt,
           (unsigned int)u16Uart4DmaStallCnt,
           (unsigned int)u16Uart4TxFallbackCnt,
           (unsigned int)u32Uart4TxFallbackBytes,
           (unsigned int)u16Uart4TxAbortCnt,
           (unsigned int)u16Uart4ReasmCnt);
}

/* 任务里轮询: 取出整帧, 交给应用层处理 (见 UART4_APP_MODE) */
void UART4_FrameProc(void)
{
    uint16_t       u16Len;
    const uint8_t *pFrame;

    if (0u != u8Uart4WarnPending)
    {
        UART4_LogStats();
    }

    if (0u == u8Uart4FrameOK)
    {
        return;
    }

    /* 关键: u8Uart4FrameOK 保持为 1 期间, RTO 中断既不会发布新帧,
     * 也不会让 DMA 写到这一块缓冲上。所以这里可以直接拿着指针用, 不必拷贝。*/
    u16Len = u16Uart4RxLen;
    if (u16Len > UART4_RX_BUF_SIZE)
    {
        u16Len = UART4_RX_BUF_SIZE;
    }
    pFrame = gs_au8RxBuf[u8Uart4PubIdx];

#if (UART4_APP_MODE == UART4_APP_ECHO)
    /* 通路自测: 把同一帧从 UART4(PB14) 回发 "RCV[长度]:数据\r\n" */
    UART4_ReplyFrame(pFrame, u16Len);
    UART4_ReleaseFrame();       /* 最后释放, 释放完中断才能写新帧 */
#else
    /* F460 Manager 协议应答器: 先解析出应答帧(不碰硬件), 再释放帧,
     * 最后才做阻塞式发送 —— 发送期间到达的新帧会被存到另一块缓冲, 不计丢帧。
     * (协议详见 App/Manager_Protocol.c) */
    {
        const uint8_t *pRsp;
        uint16_t       u16RspLen = 0u;

        pRsp = Manager_ProtocolResolve(pFrame, u16Len, &u16RspLen);

        UART4_ReleaseFrame();

        if (NULL != pRsp)
        {
            UART4_SendData(pRsp, u16RspLen);
        }
    }
#endif
}

#endif /* UART4_MODE */

/*******************************************************************************
 * 公共接口
 ******************************************************************************/

/* 发送以 0 结尾的字符串 */
void UART4_SendString(const char *str)
{
    uint16_t u16Len = 0u;

    if (NULL == str)
    {
        return;
    }

    while ((str[u16Len] != 0) && (u16Len < UART4_TX_BUF_SIZE))
    {
        u16Len++;
    }

    if (0u == u16Len)
    {
        return;
    }

    UART4_SendData((const uint8_t *)str, u16Len);
}
