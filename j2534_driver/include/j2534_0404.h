// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "j2534_types.h"

#define STATUS_NOERROR 0x00000000UL
#define ERR_NOT_SUPPORTED 0x00000001UL
#define ERR_INVALID_CHANNEL_ID 0x00000002UL
#define ERR_INVALID_PROTOCOL_ID 0x00000003UL
#define ERR_NULL_PARAMETER 0x00000004UL
#define ERR_INVALID_IOCTL_VALUE 0x00000005UL
#define ERR_INVALID_FLAGS 0x00000006UL
#define ERR_FAILED 0x00000007UL
#define ERR_DEVICE_NOT_CONNECTED 0x00000008UL
#define ERR_TIMEOUT 0x00000009UL
#define ERR_INVALID_MSG 0x0000000AUL
#define ERR_INVALID_TIME_INTERVAL 0x0000000BUL
#define ERR_EXCEEDED_LIMIT 0x0000000CUL
#define ERR_INVALID_MSG_ID 0x0000000DUL
#define ERR_DEVICE_IN_USE 0x0000000EUL
#define ERR_INVALID_IOCTL_ID 0x0000000FUL
#define ERR_BUFFER_EMPTY 0x00000010UL
#define ERR_BUFFER_FULL 0x00000011UL
#define ERR_BUFFER_OVERFLOW 0x00000012UL
#define ERR_PIN_INVALID 0x00000013UL
#define ERR_CHANNEL_IN_USE 0x00000014UL
#define ERR_MSG_PROTOCOL_ID 0x00000015UL
#define ERR_INVALID_FILTER_ID 0x00000016UL
#define ERR_NO_FLOW_CONTROL 0x00000017UL
#define ERR_NOT_UNIQUE 0x00000018UL
#define ERR_INVALID_BAUDRATE 0x00000019UL
#define ERR_INVALID_DEVICE_ID 0x0000001AUL

#define J1850VPW 0x00000001UL
#define J1850PWM 0x00000002UL
#define ISO9141 0x00000003UL
#define ISO14230 0x00000004UL
#define CAN 0x00000005UL
#define ISO15765 0x00000006UL
#define SCI_A_ENGINE 0x00000007UL
#define SCI_A_TRANS 0x00000008UL
#define SCI_B_ENGINE 0x00000009UL
#define SCI_B_TRANS 0x0000000AUL

#define GET_CONFIG 0x00000001UL
#define SET_CONFIG 0x00000002UL
#define READ_VBATT 0x00000003UL
#define FIVE_BAUD_INIT 0x00000004UL
#define FAST_INIT 0x00000005UL
#define CLEAR_TX_BUFFER 0x00000007UL
#define CLEAR_RX_BUFFER 0x00000008UL
#define CLEAR_PERIODIC_MSGS 0x00000009UL
#define CLEAR_MSG_FILTERS 0x0000000AUL
#define CLEAR_FUNCT_MSG_LOOKUP_TABLE 0x0000000BUL
#define ADD_TO_FUNCT_MSG_LOOKUP_TABLE 0x0000000CUL
#define DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE 0x0000000DUL
#define READ_PROG_VOLTAGE 0x0000000EUL

#define DATA_RATE 0x00000001UL
#define LOOPBACK 0x00000003UL
#define NODE_ADDRESS 0x00000004UL
#define NETWORK_LINE 0x00000005UL
#define P1_MIN 0x00000006UL
#define P1_MAX 0x00000007UL
#define P2_MIN 0x00000008UL
#define P2_MAX 0x00000009UL
#define P3_MIN 0x0000000AUL
#define P3_MAX 0x0000000BUL
#define P4_MIN 0x0000000CUL
#define P4_MAX 0x0000000DUL
#define W1_MAX 0x0000000EUL
#define W2_MAX 0x0000000FUL
#define W3_MAX 0x00000010UL
#define W4_MIN 0x00000011UL
#define W5_MIN 0x00000012UL
#define TIDLE 0x00000013UL
#define TINIL 0x00000014UL
#define TWUP 0x00000015UL
#define PARITY 0x00000016UL
#define W0_MIN 0x00000019UL
#define T1_MAX 0x0000001AUL
#define T2_MIN 0x0000001BUL
#define T4_MAX 0x0000001CUL
#define T5_MIN 0x0000001DUL
#define ISO15765_BS 0x0000001EUL
#define ISO15765_STMIN 0x0000001FUL
#define DATA_BITS 0x00000020UL
#define FIVE_BAUD_MOD 0x00000021UL
#define BS_TX 0x00000022UL
#define STMIN_TX 0x00000023UL
#define ISO15765_WFT_MAX 0x00000025UL
#define J1962_PINS 0x00008001UL

#define PASS_FILTER 0x00000001UL
#define BLOCK_FILTER 0x00000002UL
#define FLOW_CONTROL_FILTER 0x00000003UL

#define TX_MSG_TYPE 0x00000001UL
#define START_OF_MESSAGE 0x00000002UL
#define RX_BREAK 0x00000004UL
#define TX_INDICATION 0x00000008UL
#define ISO15765_PADDING_ERROR 0x00000010UL
#define ISO15765_FRAME_PAD 0x00000040UL
#define ISO15765_ADDR_TYPE 0x00000080UL
#define CAN_29BIT_ID 0x00000100UL
#define WAIT_P3_MIN_ONLY 0x00000200UL
#define ISO9141_NO_CHECKSUM 0x00000200UL
#define CAN_ID_BOTH 0x00000800UL
#define ISO9141_K_LINE_ONLY 0x00001000UL
#define SCI_MODE 0x00400000UL
#define SCI_TX_VOLTAGE 0x00800000UL

#define SHORT_TO_GROUND 0xFFFFFFFEUL
#define VOLTAGE_OFF 0xFFFFFFFFUL

#pragma pack(push, 1)
typedef struct {
    J2534_ULONG ProtocolID;
    J2534_ULONG RxStatus;
    J2534_ULONG TxFlags;
    J2534_ULONG Timestamp;
    J2534_ULONG DataSize;
    J2534_ULONG ExtraDataIndex;
    unsigned char Data[4128];
} PASSTHRU_MSG;

typedef struct {
    J2534_ULONG Parameter;
    J2534_ULONG Value;
} SCONFIG;

typedef struct {
    J2534_ULONG NumOfParams;
    SCONFIG *ConfigPtr;
} SCONFIG_LIST;

typedef struct {
    J2534_ULONG NumOfBytes;
    unsigned char *BytePtr;
} SBYTE_ARRAY;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif
J2534_EXPORT J2534_LONG J2534_CALL PassThruOpen(void *name, J2534_ULONG *deviceId);
J2534_EXPORT J2534_LONG J2534_CALL PassThruClose(J2534_ULONG deviceId);
J2534_EXPORT J2534_LONG J2534_CALL PassThruConnect(J2534_ULONG deviceId,
                                                   J2534_ULONG protocol,
                                                   J2534_ULONG flags,
                                                   J2534_ULONG baudrate,
                                                   J2534_ULONG *channelId);
J2534_EXPORT J2534_LONG J2534_CALL PassThruDisconnect(J2534_ULONG channelId);
J2534_EXPORT J2534_LONG J2534_CALL PassThruReadMsgs(J2534_ULONG channelId,
                                                    PASSTHRU_MSG *messages,
                                                    J2534_ULONG *count,
                                                    J2534_ULONG timeout);
J2534_EXPORT J2534_LONG J2534_CALL PassThruWriteMsgs(J2534_ULONG channelId,
                                                     PASSTHRU_MSG *messages,
                                                     J2534_ULONG *count,
                                                     J2534_ULONG timeout);
J2534_EXPORT J2534_LONG J2534_CALL PassThruStartPeriodicMsg(J2534_ULONG channelId,
                                                            PASSTHRU_MSG *message,
                                                            J2534_ULONG *messageId,
                                                            J2534_ULONG interval);
J2534_EXPORT J2534_LONG J2534_CALL PassThruStopPeriodicMsg(J2534_ULONG channelId,
                                                           J2534_ULONG messageId);
J2534_EXPORT J2534_LONG J2534_CALL PassThruStartMsgFilter(J2534_ULONG channelId,
                                                          J2534_ULONG type,
                                                          PASSTHRU_MSG *mask,
                                                          PASSTHRU_MSG *pattern,
                                                          PASSTHRU_MSG *flow,
                                                          J2534_ULONG *filterId);
J2534_EXPORT J2534_LONG J2534_CALL PassThruStopMsgFilter(J2534_ULONG channelId,
                                                         J2534_ULONG filterId);
J2534_EXPORT J2534_LONG J2534_CALL PassThruSetProgrammingVoltage(J2534_ULONG deviceId,
                                                                 J2534_ULONG pin,
                                                                 J2534_ULONG voltage);
J2534_EXPORT J2534_LONG J2534_CALL PassThruReadVersion(J2534_ULONG deviceId,
                                                       char *firmware,
                                                       char *dll,
                                                       char *api);
J2534_EXPORT J2534_LONG J2534_CALL PassThruGetLastError(char *description);
J2534_EXPORT J2534_LONG J2534_CALL PassThruIoctl(J2534_ULONG target,
                                                 J2534_ULONG id,
                                                 void *input,
                                                 void *output);
#ifdef __cplusplus
}
#endif
