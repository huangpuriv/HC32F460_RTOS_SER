/*******************************************************************************
 * Manager_Protocol.c  --  "Energy Controller <-> F460 Manager" APDU 应答器
 *
 * 从 DUAL_CDC_HC32F460_v2.1.7 1108 的 usb_dev_dual_cdc 工程 COM#1 通道移植,
 * 收发载体由 USB CDC 改为 USART4 数据口 (PB14 = TX / PB15 = RX)。
 * 协议行为与原实现逐字节一致, 详见 Manager_Protocol.h 的说明。

 *

 * 标准依据: T/SMI 1017.4-2021 <<能源控制器软件及接口技术规范 第4部分: 功能模组接口协议>>。

 *           MANAGER_2021 = 1 时按该标准补齐一致性校验与应答(见下方变更清单), 默认 0 关闭。
 *
 * 与原实现的有意差异:
 *   1) 不做分片重组。原工程因为 USB 单包最大 64 字节, 用 com_buffer/first_len
 *      把长帧拼起来; UART4 侧有硬件 RTO 判帧, 一帧是完整到达的, 不需要重组。
 *   2) 增加帧头/帧尾/L 域自检。L 不匹配只告警不丢弃, 保持原工程的宽松行为。
 *   3) 原工程收到请求后把整帧 HEX dump 到调试口, 这里默认只打一行摘要
 *      (MGR_DEBUG_PRINT 置 2 可恢复 dump)。
 ******************************************************************************/
#include "hc32_ddl.h"
#include <stdio.h>
#include "Manager_Protocol.h"
#include "Hw_Uart4.h"

#if (UART4_MODE == USART4_DMA_RTO)

/*-------------------------------- 常量 --------------------------------------*/
#define MGR_HDR                 0x68u       /* 帧起始符 */
#define MGR_TAIL                0x16u       /* 帧结束符 */

#define MGR_APDU_LINK_REQ       0x01u       /* 链路请求 -> 应答服务码 0x81 */
#define MGR_APDU_GET_REQ        0x02u       /* Get 请求 -> 应答服务码 0x82 */
#define MGR_APDU_SET_REQ        0x03u       /* Set 请求 -> 应答服务码 0x83 */

#define MGR_FRAME_MIN_LEN       8u          /* 最短: 68 L 00 C T OAD(2B) 加尾 */
#define MGR_LEN_OVERHEAD        8u          /* 长度域 L = 帧总长 - 8 */

/*------------------------------- 调试宏 -------------------------------------*/
#if (MGR_DEBUG_PRINT >= 1)
#define MGR_LOG(...)            printf(__VA_ARGS__)
#else
#define MGR_LOG(...)            do { } while (0)
#endif

#if (MGR_DEBUG_PRINT >= 2)
#define MGR_DUMP(tag, buf, n)   Mgr_DumpFrame((tag), (buf), (n))
#else
#define MGR_DUMP(tag, buf, n)   do { } while (0)
#endif

/*================ T/SMI 1017.4-2021 一致性增强 (MANAGER_2021 = 1) ============*
 * 默认 0 = 关闭; 关闭时本段完全不参与编译, 行为与源 USB CDC 工程逐字节一致。
 *
 * 开关打开后逐项生效的变更 (每条都能在标准 PDF 里找到出处):
 *   1) 6.2.2   长度域 L 按 2 字节解析 (bit0..bit11 数据域字节数, bit12..bit15 保留),
 *              校验保留位为 0 且 L == 帧总长 - 8; 不符直接丢弃。
 *              原实现: 只读 [1] 一个字节, 不符仅打印告警。
 *   2) 6.2.3   控制域 C 按 2 字节位域解析: [3] = DIR|PRM|控制码, [4] = 帧序号 FID。
 *              只接受 DIR=0(终端发出) + PRM=1(终端发起) + 控制码=02H(信息帧);
 *              确认帧(00H)/否认帧(01H) 无数据域, 一律忽略。
 *              原实现: [3] 完全不校验, [4] 被当成"标志"且不参与分派。
 *   3) 6.2.5   帧校验 FCS 按附录A 的 PPP FCS-16 计算并校验收到的帧, 不符丢弃。
 *              原实现: 从不校验 (源工程直接抄抓包常量)。
 *   4) 6.2.3.3 响应帧序号应与请求一致: 应答的 [4] 回填请求的 FID, 并按附录A 重算 FCS。
 *              原实现: 用 ROM 常量表里硬编码的 FID (实测 01/02/00/00/00/03 各不相同)。
 *   5) 8.2     表23 补齐 DTA1 = 0H 通用信息类标识; 对本模组未实现、或本不该读的信息类,
 *              按 7.4.7 Get-Result CHOICE / 7.4.8 SetResponse 回 DAR 否认应答
 *              (7.3.8 DAR: 1 拒绝读写 / 3 信息类未定义 / 4 暂时失效)。
 *              原实现: 一律静默不回帧。
 *   6) 7.4.1 / 7.4.9  增加上报通知 [4] -> 上报通知响应 [132] 的应答。
 *              原实现: 落入 default 分支被当作非 Manager 服务码忽略。
 *===========================================================================*/
#if (MANAGER_2021)

/* ---- 6.2.2 长度域 L ---- */
#define MGR_L_LEN_MASK          0x0FFFu   /* bit0..bit11 数据域字节数 */
#define MGR_L_RSV_MASK          0xF000u   /* bit12..bit15 保留, 必须为 0 */

/* ---- 6.2.3 控制域 C ---- */
#define MGR_C_DIR_MASK          0x80u     /* bit7 传输方向 DIR */
#define MGR_C_PRM_MASK          0x40u     /* bit6 启动标志 PRM */
#define MGR_C_CODE_MASK         0x3Fu     /* bit5..bit0 控制码 */
#define MGR_C_DIR_ECU           0x00u     /* DIR=0 本帧由终端发出 */
#define MGR_C_DIR_MDL           0x80u     /* DIR=1 本帧由模组发出 */
#define MGR_C_PRM_ECU           0x40u     /* PRM=1 本帧由终端发起 */
#define MGR_C_PRM_MDL           0x00u     /* PRM=0 本帧由模组发起 */
#define MGR_C_CODE_ACK          0x00u     /* 00H 确认帧: 无数据域 */
#define MGR_C_CODE_NAK          0x01u     /* 01H 否认帧: 数据域 = 错误信息码 2 字节 */
#define MGR_C_CODE_INFO         0x02u     /* 02H 信息帧: 数据域 = APDU */
#define MGR_C_REQ_ECU           (MGR_C_DIR_ECU | MGR_C_PRM_ECU | MGR_C_CODE_INFO)   /* 0x42 */
#define MGR_C_RSP_MDL           (MGR_C_DIR_MDL | MGR_C_PRM_MDL | MGR_C_CODE_INFO)   /* 0xC2 */

/* ---- 7.4.1 APDU CHOICE 标记 (请求 01/02/03 已在上面定义) ---- */
#define MGR_APDU_REPORT_REQ     0x04u     /* [4]   上报通知 */
#define MGR_APDU_LINK_RSP       0x81u     /* [129] 链路协商响应 */
#define MGR_APDU_GET_RSP        0x82u     /* [130] 数据读取响应 */
#define MGR_APDU_SET_RSP        0x83u     /* [131] 设置响应 */
#define MGR_APDU_REPORT_RSP     0x84u     /* [132] 上报通知响应 */

/* ---- 7.4.7 Get-Result CHOICE / 7.3 A-XDR 标记 ---- */
#define MGR_GETRES_ERR          0x00u     /* [0] 错误信息 DAR */
#define MGR_GETRES_DATA         0x01u     /* [1] 数据 Data */
#define MGR_TAG_ENUM            0x16u     /* 22: ENUMERATED (7.3.8 的 DAR 用它) */

/* ---- 7.3.8 DAR 数据访问结果 ---- */
#define MGR_DAR_SUCCESS         0x00u     /* 0 成功 */
#define MGR_DAR_REFUSE          0x01u     /* 1 拒绝读写 */
#define MGR_DAR_HW_FAIL         0x02u     /* 2 硬件失效 */
#define MGR_DAR_UNDEF_CLASS     0x03u     /* 3 信息类未定义 */
#define MGR_DAR_TEMP_FAIL       0x04u     /* 4 暂时失效 */
#define MGR_DAR_UNDEF_OBJ       0x05u     /* 5 对象未定义 */

/* ---- 8.1 / 8.2 表23 DTA1 = 0H 通用信息类标识 ---- */
#define MGR_DTA1_GENERAL        0x00u     /* DTA1=0H 通用信息类 */
#define MGR_DTA2_READONLY       0x00u     /* DTA2=0H 只读 (支持查询或通知) */
#define MGR_DTA2_RW             0x01u     /* DTA2=1H 可读/写 (支持查询和设置) */
#define MGR_DTA2_WRITEONLY      0x02u     /* DTA2=2H 只写 (不支持查询) */
#define MGR_DT_MODULE_INFO      0x0000u   /* 00.00H 模组信息        只读 */
#define MGR_DT_LINK_INFO        0x0001u   /* 00.01H 链路协商信息    只读 */
#define MGR_DT_PROTO_VER        0x0002u   /* 00.02H 协议版本        只读 */
#define MGR_DT_POWER_STS        0x0003u   /* 00.03H 电源状态        只读 */
#define MGR_DT_FILE_STS         0x0004u   /* 00.04H 文件传输状态    只读 */
#define MGR_DT_BOOT_TIME        0x0006u   /* 00.06H 启动时间        只读 */
#define MGR_DT_CLOCK            0x0100u   /* 01.00H 时钟            读/写 */
#define MGR_DT_HW_INIT          0x0200u   /* 02.00H 硬件初始化      只写 */
#define MGR_DT_PARA_INIT        0x0201u   /* 02.01H 参数初始化      只写 */
#define MGR_DT_DATA_INIT        0x0202u   /* 02.02H 数据初始化      只写 */
#define MGR_DT_FILE_XFER        0x0203u   /* 02.03H 文件传输        只写 */

/* ---- 应答 RAM 缓冲: FID 回填与 FCS 重算需要可写副本 ---- */
#define MGR_RSP_BUF_SIZE        160u      /* 最长应答 122 字节, 留足余量 */
#define MGR_FCS_INIT            0xFFFFu   /* 附录A PPPINITFCS16 */
#define MGR_FCS_GOOD            0xF0B8u   /* 附录A PPPGOODFCS16 (整帧含FCS自校验判据) */

/* ---- 附录A FCS-16 查表: P = 0x8408 = x^0 + x^5 + x^12 + x^16 ---- */
static const uint16_t gs_au16FcsTab[256] = {
    0x0000u, 0x1189u, 0x2312u, 0x329Bu, 0x4624u, 0x57ADu, 0x6536u, 0x74BFu,
    0x8C48u, 0x9DC1u, 0xAF5Au, 0xBED3u, 0xCA6Cu, 0xDBE5u, 0xE97Eu, 0xF8F7u,
    0x1081u, 0x0108u, 0x3393u, 0x221Au, 0x56A5u, 0x472Cu, 0x75B7u, 0x643Eu,
    0x9CC9u, 0x8D40u, 0xBFDBu, 0xAE52u, 0xDAEDu, 0xCB64u, 0xF9FFu, 0xE876u,
    0x2102u, 0x308Bu, 0x0210u, 0x1399u, 0x6726u, 0x76AFu, 0x4434u, 0x55BDu,
    0xAD4Au, 0xBCC3u, 0x8E58u, 0x9FD1u, 0xEB6Eu, 0xFAE7u, 0xC87Cu, 0xD9F5u,
    0x3183u, 0x200Au, 0x1291u, 0x0318u, 0x77A7u, 0x662Eu, 0x54B5u, 0x453Cu,
    0xBDCBu, 0xAC42u, 0x9ED9u, 0x8F50u, 0xFBEFu, 0xEA66u, 0xD8FDu, 0xC974u,
    0x4204u, 0x538Du, 0x6116u, 0x709Fu, 0x0420u, 0x15A9u, 0x2732u, 0x36BBu,
    0xCE4Cu, 0xDFC5u, 0xED5Eu, 0xFCD7u, 0x8868u, 0x99E1u, 0xAB7Au, 0xBAF3u,
    0x5285u, 0x430Cu, 0x7197u, 0x601Eu, 0x14A1u, 0x0528u, 0x37B3u, 0x263Au,
    0xDECDu, 0xCF44u, 0xFDDFu, 0xEC56u, 0x98E9u, 0x8960u, 0xBBFBu, 0xAA72u,
    0x6306u, 0x728Fu, 0x4014u, 0x519Du, 0x2522u, 0x34ABu, 0x0630u, 0x17B9u,
    0xEF4Eu, 0xFEC7u, 0xCC5Cu, 0xDDD5u, 0xA96Au, 0xB8E3u, 0x8A78u, 0x9BF1u,
    0x7387u, 0x620Eu, 0x5095u, 0x411Cu, 0x35A3u, 0x242Au, 0x16B1u, 0x0738u,
    0xFFCFu, 0xEE46u, 0xDCDDu, 0xCD54u, 0xB9EBu, 0xA862u, 0x9AF9u, 0x8B70u,
    0x8408u, 0x9581u, 0xA71Au, 0xB693u, 0xC22Cu, 0xD3A5u, 0xE13Eu, 0xF0B7u,
    0x0840u, 0x19C9u, 0x2B52u, 0x3ADBu, 0x4E64u, 0x5FEDu, 0x6D76u, 0x7CFFu,
    0x9489u, 0x8500u, 0xB79Bu, 0xA612u, 0xD2ADu, 0xC324u, 0xF1BFu, 0xE036u,
    0x18C1u, 0x0948u, 0x3BD3u, 0x2A5Au, 0x5EE5u, 0x4F6Cu, 0x7DF7u, 0x6C7Eu,
    0xA50Au, 0xB483u, 0x8618u, 0x9791u, 0xE32Eu, 0xF2A7u, 0xC03Cu, 0xD1B5u,
    0x2942u, 0x38CBu, 0x0A50u, 0x1BD9u, 0x6F66u, 0x7EEFu, 0x4C74u, 0x5DFDu,
    0xB58Bu, 0xA402u, 0x9699u, 0x8710u, 0xF3AFu, 0xE226u, 0xD0BDu, 0xC134u,
    0x39C3u, 0x284Au, 0x1AD1u, 0x0B58u, 0x7FE7u, 0x6E6Eu, 0x5CF5u, 0x4D7Cu,
    0xC60Cu, 0xD785u, 0xE51Eu, 0xF497u, 0x8028u, 0x91A1u, 0xA33Au, 0xB2B3u,
    0x4A44u, 0x5BCDu, 0x6956u, 0x78DFu, 0x0C60u, 0x1DE9u, 0x2F72u, 0x3EFBu,
    0xD68Du, 0xC704u, 0xF59Fu, 0xE416u, 0x90A9u, 0x8120u, 0xB3BBu, 0xA232u,
    0x5AC5u, 0x4B4Cu, 0x79D7u, 0x685Eu, 0x1CE1u, 0x0D68u, 0x3FF3u, 0x2E7Au,
    0xE70Eu, 0xF687u, 0xC41Cu, 0xD595u, 0xA12Au, 0xB0A3u, 0x8238u, 0x93B1u,
    0x6B46u, 0x7ACFu, 0x4854u, 0x59DDu, 0x2D62u, 0x3CEBu, 0x0E70u, 0x1FF9u,
    0xF78Fu, 0xE606u, 0xD49Du, 0xC514u, 0xB1ABu, 0xA022u, 0x92B9u, 0x8330u,
    0x7BC7u, 0x6A4Eu, 0x58D5u, 0x495Cu, 0x3DE3u, 0x2C6Au, 0x1EF1u, 0x0F78u,
};

static uint8_t gs_au8RspBuf[MGR_RSP_BUF_SIZE];

#endif /* MANAGER_2021 */

/*------------------------------ 应答报文 -------------------------------------*
 * 全部逐字节抄自原工程常量数组, 未做任何修改 (含 FCS 与结束符)。
 *---------------------------------------------------------------------------*/

/* 0x81 Link 链路应答, 83 字节 (L=0x4B) */
static const uint8_t gs_au8LinkResp[] = {
    0x68, 0x4B, 0x00, 0xC2, 0x01, 0x81, 0x00, 0x01, 0x0C, 0x47, 0x42, 0x48,
    0x31, 0x33, 0x2D, 0x58, 0x4A, 0x33, 0x34, 0x35, 0x32, 0x30, 0x30, 0x31,
    0x30, 0x32, 0x39, 0x43, 0x30, 0x31, 0x43, 0x31, 0x46, 0x42, 0x34, 0x30,
    0x35, 0x38, 0x34, 0x41, 0x34, 0x32, 0x35, 0x33, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x06, 0x40,
    0x06, 0x40, 0x01, 0x02, 0x01, 0x00, 0x01, 0x02, 0x4B, 0xC2, 0x16
};

/* 0x82 Get 模块信息应答 (OAD 00.00.00), 122 字节 (L=0x72) */
static const uint8_t gs_au8GetModule[] = {
    0x68, 0x72, 0x00, 0xC2, 0x02, 0x82, 0x00, 0x00, 0x01, 0x02, 0x08, 0x0A,
    0x0C, 0x47, 0x42, 0x48, 0x31, 0x33, 0x2D, 0x58, 0x4A, 0x33, 0x34, 0x35,
    0x32, 0x0A, 0x30, 0x30, 0x31, 0x30, 0x32, 0x39, 0x43, 0x30, 0x31, 0x43,
    0x31, 0x46, 0x42, 0x34, 0x30, 0x35, 0x38, 0x34, 0x41, 0x34, 0x32, 0x35,
    0x33, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x0A, 0x04, 0x58, 0x4A, 0x20, 0x20, 0x0A, 0x04, 0x56,
    0x31, 0x2E, 0x30, 0x0A, 0x06, 0x32, 0x32, 0x30, 0x38, 0x32, 0x36, 0x0A,
    0x04, 0x56, 0x31, 0x2E, 0x30, 0x0A, 0x06, 0x32, 0x32, 0x30, 0x36, 0x32,
    0x32, 0x0A, 0x08, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0xE3,
    0xA8, 0x16
};

/* 0x82 Get 链路信息应答 (OAD 00.00.01), 103 字节 (L=0x5F) */
static const uint8_t gs_au8GetLinkInfo[] = {
    0x68, 0x5F, 0x00, 0xC2, 0x00, 0x82, 0x00, 0x01, 0x01, 0x02, 0x07, 0x12,
    0x00, 0x03, 0x0A, 0x0C, 0x47, 0x4B, 0x58, 0x34, 0x33, 0x2D, 0x58, 0x4A,
    0x32, 0x32, 0x30, 0x31, 0x0A, 0x30, 0x30, 0x31, 0x30, 0x32, 0x39, 0x43,
    0x30, 0x31, 0x43, 0x31, 0x46, 0x42, 0x34, 0x30, 0x35, 0x33, 0x34, 0x33,
    0x34, 0x32, 0x35, 0x33, 0x30, 0x30, 0x30, 0x30, 0x30, 0x46, 0x34, 0x32,
    0x34, 0x45, 0x45, 0x34, 0x36, 0x41, 0x33, 0x36, 0x34, 0x30, 0x43, 0x32,
    0x42, 0x43, 0x46, 0x34, 0x45, 0x41, 0x12, 0x08, 0x00, 0x12, 0x08, 0x00,
    0x11, 0x01, 0x01, 0x02, 0x02, 0x02, 0x16, 0x01, 0x11, 0x00, 0x02, 0x02,
    0x16, 0x01, 0x11, 0x02, 0x81, 0xDC, 0x16
};

/* 0x82 Get 协议版本应答 (OAD 00.00.02), 15 字节 (L=0x07) */
static const uint8_t gs_au8GetProtoVer[] = {
    0x68, 0x07, 0x00, 0xC2, 0x00, 0x82, 0x00, 0x02, 0x01, 0x12, 0x00, 0x03,
    0x5F, 0xAF, 0x16
};

/* 0x82 Get 电源状态应答 (OAD 00.00.03), 14 字节 (L=0x06) */
static const uint8_t gs_au8GetPowerSts[] = {
    0x68, 0x06, 0x00, 0xC2, 0x00, 0x82, 0x00, 0x03, 0x01, 0x16, 0x00, 0x72,
    0x9D, 0x16
};

/* 0x83 Set 日期时间应答 (OAD 00.01.00), 12 字节 (L=0x04) */
static const uint8_t gs_au8SetResp[] = {
    0x68, 0x04, 0x00, 0xC2, 0x03, 0x83, 0x01, 0x00, 0x00, 0x3B, 0xD5, 0x16
};

/*------------------------------ 内部函数 ------------------------------------*/
#if (MGR_DEBUG_PRINT >= 2)
static void Mgr_DumpFrame(const char *pTag, const uint8_t *pBuf, uint16_t u16Len)
{
    uint16_t i;

    MGR_LOG("%s[%u]:", pTag, (unsigned int)u16Len);
    for (i = 0u; i < u16Len; i++)
    {
        MGR_LOG("%02X ", (unsigned int)pBuf[i]);
    }
    MGR_LOG("\r\n");
}
#endif

/* 打一行应答摘要日志 */
static void Mgr_LogResp(const uint8_t *pBuf, uint16_t u16Len)
{
    MGR_LOG("[MGR] -> EC: apdu=0x%02X len=%u\r\n",
            (unsigned int)pBuf[5], (unsigned int)u16Len);
}


#if (MANAGER_2021)
/*-------------------------- 附录A 帧校验 FCS-16 ------------------------------*
 * 覆盖范围: L(2B) + C(2B) + 数据域(L 字节), 即 &frame[1] 起的 (4 + L) 字节。
 * 初值 0xFFFF, 末尾取反 ^0xFFFF, 低字节在前。
 * 整帧(含FCS)自校验时结果应为 MGR_FCS_GOOD (0xF0B8), 与附录A 一致。
 *---------------------------------------------------------------------------*/
static uint16_t Mgr_Fcs16(const uint8_t *pBuf, uint16_t u16Len)
{
    uint16_t u16Fcs = MGR_FCS_INIT;

    while (0u != u16Len)
    {
        u16Fcs = (uint16_t)((u16Fcs >> 8) ^
                            gs_au16FcsTab[(u16Fcs ^ (uint16_t)(*pBuf)) & 0x00FFu]);
        pBuf++;
        u16Len--;
    }
    return u16Fcs;
}

/*------------------------ 应答整帧构造 (ROM -> RAM) --------------------------*
 * 把常量表里的应答拷到 RAM 缓冲, 回填请求的帧序号 FID (6.2.3.3),
 * 再按附录A 重算 FCS。返回 RAM 缓冲首地址; 超界返回 NULL。
 *---------------------------------------------------------------------------*/
static const uint8_t *Mgr_BuildResp(const uint8_t *pSrc, uint16_t u16Len, uint8_t u8Fid)
{
    uint16_t u16L;
    uint16_t u16Fcs;
    uint16_t i;

    if ((NULL == pSrc) || (u16Len > MGR_RSP_BUF_SIZE) || (u16Len < MGR_FRAME_MIN_LEN))
    {
        return NULL;
    }
    if (pSrc != gs_au8RspBuf)
    {
        for (i = 0u; i < u16Len; i++)
        {
            gs_au8RspBuf[i] = pSrc[i];
        }
    }

    u16L   = (uint16_t)((((uint16_t)gs_au8RspBuf[2] << 8) |
                          (uint16_t)gs_au8RspBuf[1]) & MGR_L_LEN_MASK);
    gs_au8RspBuf[4] = u8Fid;                                    /* 6.2.3.3 响应帧序号与请求一致 */
    u16Fcs = (uint16_t)(Mgr_Fcs16(&gs_au8RspBuf[1], (uint16_t)(4u + u16L)) ^ 0xFFFFu);
    gs_au8RspBuf[5u + u16L]      = (uint8_t)(u16Fcs & 0x00FFu); /* 附录A: 低字节在前 */
    gs_au8RspBuf[5u + u16L + 1u] = (uint8_t)((u16Fcs >> 8) & 0x00FFu);
    return gs_au8RspBuf;
}

/*------------------- DAR 否认应答 (7.4.7 / 7.4.8 / 7.3.8) --------------------*
 * Get  类: 数据域 = 82 | DTA | DTB | 00(Get-Result 错误信息[0]) | 16(enum) | DAR
 * Set  类: 数据域 = 83 | DTA | DTB | DAR
 * 返回: 写入 gs_au8RspBuf 的整帧长度; 0 = 失败。
 *---------------------------------------------------------------------------*/
static uint16_t Mgr_BuildDarResp(const uint8_t *pReq, uint8_t u8Fid, uint8_t u8Dar)
{
    uint16_t u16L;
    uint16_t u16Fcs;
    uint8_t *p = gs_au8RspBuf;

    if (MGR_APDU_SET_REQ == pReq[5])
    {
        u16L = 4u;                                   /* APDU + DT(2B) + DAR */
        p[5] = MGR_APDU_SET_RSP;
        p[6] = pReq[6];
        p[7] = pReq[7];
        p[8] = u8Dar;
    }
    else
    {
        u16L = 6u;                                   /* APDU + DT(2B) + [0] + enum + DAR */
        p[5]  = MGR_APDU_GET_RSP;
        p[6]  = pReq[6];
        p[7]  = pReq[7];
        p[8]  = MGR_GETRES_ERR;
        p[9]  = MGR_TAG_ENUM;
        p[10] = u8Dar;
    }
    if ((uint16_t)(u16L + MGR_LEN_OVERHEAD) > MGR_RSP_BUF_SIZE)
    {
        return 0u;
    }

    p[0] = MGR_HDR;
    p[1] = (uint8_t)(u16L & 0x00FFu);
    p[2] = (uint8_t)((u16L >> 8) & 0x00FFu);
    p[3] = (uint8_t)MGR_C_RSP_MDL;                   /* 0xC2 */
    p[4] = u8Fid;
    u16Fcs = (uint16_t)(Mgr_Fcs16(&p[1], (uint16_t)(4u + u16L)) ^ 0xFFFFu);
    p[5u + u16L]      = (uint8_t)(u16Fcs & 0x00FFu);
    p[5u + u16L + 1u] = (uint8_t)((u16Fcs >> 8) & 0x00FFu);
    p[5u + u16L + 2u] = MGR_TAIL;
    return (uint16_t)(u16L + MGR_LEN_OVERHEAD);
}

/*----------- 上报通知响应 (7.4.9, APDU [132] = 0x84) 数据域 = DT ------------*/
static uint16_t Mgr_BuildReportResp(uint8_t u8Dta, uint8_t u8Dtb, uint8_t u8Fid)
{
    uint16_t u16Fcs;
    uint8_t *p = gs_au8RspBuf;

    p[0] = MGR_HDR;
    p[1] = 3u;                                       /* L = 3 (数据域 3 字节) */
    p[2] = 0u;
    p[3] = (uint8_t)MGR_C_RSP_MDL;                   /* 0xC2 */
    p[4] = u8Fid;
    p[5] = MGR_APDU_REPORT_RSP;                      /* 0x84 */
    p[6] = u8Dta;
    p[7] = u8Dtb;
    u16Fcs = (uint16_t)(Mgr_Fcs16(&p[1], 7u) ^ 0xFFFFu);
    p[8]  = (uint8_t)(u16Fcs & 0x00FFu);
    p[9]  = (uint8_t)((u16Fcs >> 8) & 0x00FFu);
    p[10] = MGR_TAIL;
    return 11u;
}

/*------------ 8.2 表23: 读请求落空时的 DAR 判定 (7.3.8) ---------------------*/
static uint8_t Mgr_DarForGet(uint8_t u8Dta1, uint8_t u8Dta2, uint8_t u8Dtb)
{
    if (MGR_DTA1_GENERAL != u8Dta1)
    {
        return MGR_DAR_UNDEF_CLASS;      /* 非通用信息类, 本模组未定义 */
    }
    if (MGR_DTA2_READONLY == u8Dta2)
    {
        if (((uint8_t)MGR_DT_FILE_STS == u8Dtb) || ((uint8_t)MGR_DT_BOOT_TIME == u8Dtb))
        {
            return MGR_DAR_TEMP_FAIL;    /* 表23 已定义, 但本模组未实现 */
        }
        return MGR_DAR_UNDEF_OBJ;
    }
    if (MGR_DTA2_RW == u8Dta2)
    {
        return MGR_DAR_TEMP_FAIL;        /* 01.00H 时钟 "读": 本模组未实现 */
    }
    if (MGR_DTA2_WRITEONLY == u8Dta2)
    {
        return MGR_DAR_REFUSE;           /* 8.1 c) 只写类不支持查询操作 */
    }
    return MGR_DAR_UNDEF_OBJ;
}

/*------------ 8.2 表23: 写/设置请求落空时的 DAR 判定 (7.3.8) -----------------*/
static uint8_t Mgr_DarForSet(uint8_t u8Dta1, uint8_t u8Dta2, uint8_t u8Dtb)
{
    (void)u8Dtb;

    if (MGR_DTA1_GENERAL != u8Dta1)
    {
        return MGR_DAR_UNDEF_CLASS;      /* 非通用信息类, 本模组未定义 */
    }
    if (MGR_DTA2_READONLY == u8Dta2)
    {
        return MGR_DAR_REFUSE;           /* 8.1 a) 只读类不支持设置 */
    }
    return MGR_DAR_TEMP_FAIL;            /* 只写类已定义, 但本模组未实现 */
}
#endif /* MANAGER_2021 */
/*------------------------------ 对外接口 ------------------------------------*/
const uint8_t *Manager_ProtocolResolve(const uint8_t *pFrame, uint16_t u16Len,
                                       uint16_t *pu16RspLen)
{
    const uint8_t *pRsp   = NULL;
    uint16_t       u16Rsp = 0u;
    uint8_t        u8Apdu;
    uint8_t        u8Dta1;
    uint8_t        u8Dta2;
    uint8_t        u8Dtb;
#if (MANAGER_2021)
    uint16_t       u16L    = 0u;
    uint16_t       u16RspRam = 0u;       /* 由 RAM 构造器写好的应答帧长 */
    uint8_t        u8Fid   = 0u;
    uint8_t        u8Dar   = 0u;
#endif

    if (NULL != pu16RspLen)
    {
        *pu16RspLen = 0u;
    }

    /* ---- 长度 / 帧头 / 帧尾 自检 ---- */
    if ((NULL == pFrame) || (NULL == pu16RspLen) || (u16Len < MGR_FRAME_MIN_LEN))
    {
        MGR_LOG("[MGR] drop: too short (%u bytes)\r\n", (unsigned int)u16Len);
        return NULL;
    }
    if ((MGR_HDR != pFrame[0]) || (MGR_TAIL != pFrame[u16Len - 1u]))
    {
        MGR_LOG("[MGR] drop: bad head/tail (0x%02X..0x%02X)\r\n",
                (unsigned int)pFrame[0], (unsigned int)pFrame[u16Len - 1u]);
        return NULL;
    }

#if (MANAGER_2021)
    /* ---- 6.2.2 长度域 L: 2 字节 BIN, bit0..bit11 数据域字节数, bit12..bit15 保留 ---- */
    u16L = (uint16_t)((((uint16_t)pFrame[2] << 8) | (uint16_t)pFrame[1]) & MGR_L_LEN_MASK);
    if ((0u != ((((uint16_t)pFrame[2] << 8) | (uint16_t)pFrame[1]) & MGR_L_RSV_MASK)) ||
        (u16L != (uint16_t)(u16Len - MGR_LEN_OVERHEAD)))
    {
        MGR_LOG("[MGR] drop: bad L (raw=%04X), len-8=%u\r\n",
                (unsigned int)(((uint16_t)pFrame[2] << 8) | (uint16_t)pFrame[1]),
                (unsigned int)(u16Len - MGR_LEN_OVERHEAD));
        return NULL;
    }

    /* ---- 6.2.3 控制域 C: [3] = DIR|PRM|控制码, [4] = 帧序号 FID ---- */
    if ((MGR_C_CODE_INFO != (pFrame[3] & MGR_C_CODE_MASK)) ||
        (MGR_C_DIR_ECU   != (pFrame[3] & MGR_C_DIR_MASK)) ||
        (MGR_C_PRM_ECU   != (pFrame[3] & MGR_C_PRM_MASK)))
    {
        /* 确认帧(00H)/否认帧(01H) 无数据域; 模组自己发出的帧也不该在此处理 */
        MGR_LOG("[MGR] ignore: C=0x%02X not ECU info frame\r\n", (unsigned int)pFrame[3]);
        return NULL;
    }
    u8Fid = pFrame[4];

    /* ---- 6.2.5 + 附录A 帧校验 FCS: 覆盖 L + C + 数据域 ---- */
    {
        uint16_t u16Got = (uint16_t)((uint16_t)pFrame[5u + u16L] |
                                     ((uint16_t)pFrame[6u + u16L] << 8));
        uint16_t u16Cal = (uint16_t)(Mgr_Fcs16(&pFrame[1], (uint16_t)(4u + u16L)) ^ 0xFFFFu);
        if (u16Got != u16Cal)
        {
            MGR_LOG("[MGR] drop: bad FCS got=%04X calc=%04X\r\n",
                    (unsigned int)u16Got, (unsigned int)u16Cal);
            return NULL;
        }
    }
#else
    /* 非严格模式: L 不匹配只告警, 不一致也照收 (与源 USB CDC 工程逐字节一致) */
    if ((uint16_t)pFrame[1] != (uint16_t)(u16Len - MGR_LEN_OVERHEAD))
    {
        MGR_LOG("[MGR] warn: L=%u, len-8=%u\r\n",
                (unsigned int)pFrame[1],
                (unsigned int)(u16Len - MGR_LEN_OVERHEAD));
    }
#endif

    u8Apdu = pFrame[5];
    u8Dta1 = (uint8_t)((pFrame[6] & 0xF0u) >> 4);
    u8Dta2 = (uint8_t)(pFrame[6] & 0x0Fu);
    u8Dtb  = pFrame[7];

#if (MANAGER_2021)
    MGR_LOG("[MGR] <- EC: apdu=0x%02X len=%u DT=%u.%u.%u FID=%u\r\n",
            (unsigned int)u8Apdu, (unsigned int)u16Len,
            (unsigned int)u8Dta1, (unsigned int)u8Dta2, (unsigned int)u8Dtb,
            (unsigned int)u8Fid);
#else
    MGR_LOG("[MGR] <- EC: apdu=0x%02X len=%u OAD=%u.%u.%u\r\n",
            (unsigned int)u8Apdu, (unsigned int)u16Len,
            (unsigned int)u8Dta1, (unsigned int)u8Dta2, (unsigned int)u8Dtb);
#endif
    MGR_DUMP("  req", pFrame, u16Len);

    switch (u8Apdu)
    {
    /*------------------------------------------------ Link 链路协商 --------*/
    case MGR_APDU_LINK_REQ:
        pRsp   = gs_au8LinkResp;
        u16Rsp = (uint16_t)sizeof(gs_au8LinkResp);
        break;

    /*------------------------------------------------- Get 读请求 ----------*/
    case MGR_APDU_GET_REQ:
        if ((0u == u8Dta1) && (0u == u8Dta2) && (0u == u8Dtb))       /* 00.00.00 模组信息 */
        {
            pRsp   = gs_au8GetModule;
            u16Rsp = (uint16_t)sizeof(gs_au8GetModule);
        }
        else if ((0u == u8Dta1) && (0u == u8Dta2) && (1u == u8Dtb))  /* 00.00.01 链路协商信息 */
        {
            pRsp   = gs_au8GetLinkInfo;
            u16Rsp = (uint16_t)sizeof(gs_au8GetLinkInfo);
        }
        else if ((0u == u8Dta1) && (0u == u8Dta2) && (2u == u8Dtb))  /* 00.00.02 协议版本 */
        {
            pRsp   = gs_au8GetProtoVer;
            u16Rsp = (uint16_t)sizeof(gs_au8GetProtoVer);
        }
        else if ((0u == u8Dta1) && (0u == u8Dta2) && (3u == u8Dtb))  /* 00.00.03 电源状态 */
        {
            pRsp   = gs_au8GetPowerSts;
            u16Rsp = (uint16_t)sizeof(gs_au8GetPowerSts);
        }
        else
        {
#if (MANAGER_2021)
            /* 8.2 表23: 已定义但本模组未实现 / 只写不可读 / 表外未定义 -> DAR 否认应答 */
            u8Dar     = Mgr_DarForGet(u8Dta1, u8Dta2, u8Dtb);
            u16RspRam = Mgr_BuildDarResp(pFrame, u8Fid, u8Dar);
            MGR_LOG("[MGR] no data: DT=%u.%u.%u -> DAR=%u\r\n",
                    (unsigned int)u8Dta1, (unsigned int)u8Dta2,
                    (unsigned int)u8Dtb, (unsigned int)u8Dar);
#else
            MGR_LOG("[MGR] no reply: unknown OAD %u.%u.%u\r\n",
                    (unsigned int)u8Dta1, (unsigned int)u8Dta2, (unsigned int)u8Dtb);
#endif
        }
        break;

    /*------------------------------------------------- Set 写请求 ----------*/
    case MGR_APDU_SET_REQ:
        if ((0u == u8Dta1) && (1u == u8Dta2) && (0u == u8Dtb))       /* 01.00.00 设置时钟 */
        {
            pRsp   = gs_au8SetResp;
            u16Rsp = (uint16_t)sizeof(gs_au8SetResp);
        }
        else
        {
#if (MANAGER_2021)
            u8Dar     = Mgr_DarForSet(u8Dta1, u8Dta2, u8Dtb);
            u16RspRam = Mgr_BuildDarResp(pFrame, u8Fid, u8Dar);
            MGR_LOG("[MGR] no exec: DT=%u.%u.%u -> DAR=%u\r\n",
                    (unsigned int)u8Dta1, (unsigned int)u8Dta2,
                    (unsigned int)u8Dtb, (unsigned int)u8Dar);
#else
            MGR_LOG("[MGR] no reply: unknown OAD %u.%u.%u\r\n",
                    (unsigned int)u8Dta1, (unsigned int)u8Dta2, (unsigned int)u8Dtb);
#endif
        }
        break;

#if (MANAGER_2021)
    /*------------------------ 上报通知 [4] -> 上报通知响应 [132] (7.4.9) ----*/
    case MGR_APDU_REPORT_REQ:
        u16RspRam = Mgr_BuildReportResp(pFrame[6], pFrame[7], u8Fid);
        MGR_LOG("[MGR] report ack: DT=%u.%u\r\n",
                (unsigned int)((pFrame[6] & 0xF0u) >> 4),
                (unsigned int)(pFrame[6] & 0x0Fu));
        break;
#endif

    /*-------------- 非 F460 Manager 的服务码 (属 CCO 通道), 本模块不处理 ---*/
    default:
        MGR_LOG("[MGR] not-manager apdu 0x%02X, ignored\r\n", (unsigned int)u8Apdu);
        break;
    }

#if (MANAGER_2021)
    if (NULL != pRsp)
    {
        pRsp = Mgr_BuildResp(pRsp, u16Rsp, u8Fid);   /* 拷贝到 RAM, 回填 FID, 重算 FCS */
        if (NULL != pRsp)
        {
            Mgr_LogResp(pRsp, u16Rsp);
            *pu16RspLen = u16Rsp;
        }
    }
    else if (0u != u16RspRam)
    {
        pRsp        = gs_au8RspBuf;                  /* DAR / 上报响应已在缓冲内算好 FCS */
        u16Rsp      = u16RspRam;
        Mgr_LogResp(pRsp, u16Rsp);
        *pu16RspLen = u16Rsp;
    }
    else
    {
        /* 不需要应答 */
    }
#else
    if (NULL != pRsp)
    {
        Mgr_LogResp(pRsp, u16Rsp);
        *pu16RspLen = u16Rsp;
    }
#endif

    return pRsp;
}

#endif /* UART4_MODE == USART4_DMA_RTO */
