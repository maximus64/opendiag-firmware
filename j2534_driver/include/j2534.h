// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "j2534_types.h"

#define STATUS_NOERROR 0x00000000UL
#define ERR_NOT_SUPPORTED 0x00000001UL
#define ERR_INVALID_CHANNEL_ID 0x00000002UL
#define ERR_PROTOCOL_ID_NOT_SUPPORTED 0x00000003UL
#define ERR_NULL_PARAMETER 0x00000004UL
#define ERR_IOCTL_VALUE_NOT_SUPPORTED 0x00000005UL
#define ERR_FLAG_NOT_SUPPORTED 0x00000006UL
#define ERR_FAILED 0x00000007UL
#define ERR_DEVICE_NOT_CONNECTED 0x00000008UL
#define ERR_TIMEOUT 0x00000009UL
#define ERR_INVALID_MSG 0x0000000AUL
#define ERR_TIME_INTERVAL_NOT_SUPPORTED 0x0000000BUL
#define ERR_EXCEEDED_LIMIT 0x0000000CUL
#define ERR_INVALID_MSG_ID 0x0000000DUL
#define ERR_DEVICE_IN_USE 0x0000000EUL
#define ERR_IOCTL_ID_NOT_SUPPORTED 0x0000000FUL
#define ERR_BUFFER_EMPTY 0x00000010UL
#define ERR_BUFFER_FULL 0x00000011UL
#define ERR_BUFFER_OVERFLOW 0x00000012UL
#define ERR_PIN_NOT_SUPPORTED 0x00000013UL
#define ERR_RESOURCE_CONFLICT 0x00000014UL
#define ERR_MSG_PROTOCOL_ID 0x00000015UL
#define ERR_INVALID_FILTER_ID 0x00000016UL
#define ERR_MSG_NOT_ALLOWED 0x00000017UL
#define ERR_NOT_UNIQUE 0x00000018UL
#define ERR_BAUDRATE_NOT_SUPPORTED 0x00000019UL
#define ERR_INVALID_DEVICE_ID 0x0000001AUL
#define ERR_DEVICE_NOT_OPEN 0x0000001BUL
#define ERR_NULL_REQUIRED 0x0000001CUL
#define ERR_FILTER_TYPE_NOT_SUPPORTED 0x0000001DUL
#define ERR_IOCTL_PARAM_ID_NOT_SUPPORTED 0x0000001EUL
#define ERR_VOLTAGE_IN_USE 0x0000001FUL
#define ERR_PIN_IN_USE 0x00000020UL
#define ERR_INIT_FAILED 0x00000021UL
#define ERR_OPEN_FAILED 0x00000022UL
#define ERR_BUFFER_TOO_SMALL 0x00000023UL
#define ERR_LOG_CHAN_NOT_ALLOWED 0x00000024UL
#define ERR_SELECT_TYPE_NOT_SUPPORTED 0x00000025UL
#define ERR_CONCURRENT_API_CALL 0x00000026UL
#define ERR_INVALID_CHANNEL_DESCRIPTOR 0x00000027UL
#define DEVICE_STATE_UNKNOWN 0x00000000UL
#define DEVICE_AVAILABLE 0x00000001UL
#define DEVICE_IN_USE 0x00000002UL
#define DEVICE_DLL_FW_COMPATIBILTY_UNKNOWN 0x00000000UL
#define DEVICE_DLL_FW_COMPATIBLE 0x00000001UL
#define DEVICE_DLL_OR_FW_NOT_COMPATIBLE 0x00000002UL
#define DEVICE_DLL_NOT_COMPATIBLE 0x00000003UL
#define DEVICE_FW_NOT_COMPATIBLE 0x00000004UL
#define DEVICE_CONN_UNKNOWN 0x00000000UL
#define DEVICE_CONN_WIRELESS 0x00000001UL
#define DEVICE_CONN_WIRED 0x00000002UL
#define J1850VPW 0x00000001UL
#define J1850PWM 0x00000002UL
#define ISO9141 0x00000003UL
#define ISO14230 0x00000004UL
#define CAN 0x00000005UL
#define J2610 0x00000007UL
#define ISO15765_LOGICAL 0x00000200UL
#define J1962_CONNECTOR 0x00000001UL
#define LINK_DOWN 0x00000001UL
#define READABLE_TYPE 0x00000001UL
#define PASS_FILTER 0x00000001UL
#define BLOCK_FILTER 0x00000002UL
#define SHORT_TO_GROUND 0xFFFFFFFEUL
#define PIN_OFF 0xFFFFFFFFUL
#define GET_CONFIG 0x00000001UL
#define SET_CONFIG 0x00000002UL
#define READ_PIN_VOLTAGE 0x00000003UL
#define FIVE_BAUD_INIT 0x00000004UL
#define FAST_INIT 0x00000005UL
#define CLEAR_TX_QUEUE 0x00000007UL
#define CLEAR_RX_QUEUE 0x00000008UL
#define CLEAR_PERIODIC_MSGS 0x00000009UL
#define CLEAR_MSG_FILTERS 0x0000000AUL
#define CLEAR_FUNCT_MSG_LOOKUP_TABLE 0x0000000BUL
#define ADD_TO_FUNCT_MSG_LOOKUP_TABLE 0x0000000CUL
#define READ_PROG_VOLTAGE 0x0000000EUL
#define BUS_ON 0x0000000FUL
#define DATA_RATE 0x00000001UL
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
#define T3_MAX 0x00000024UL
#define ISO15765_WAIT_LIMIT 0x00000025UL
#define W1_MIN 0x00000026UL
#define W2_MIN 0x00000027UL
#define W3_MIN 0x00000028UL
#define W4_MAX 0x00000029UL
#define N_BR_MIN 0x0000002AUL
#define ISO15765_PAD_VALUE 0x0000002BUL
#define N_AS_MAX 0x0000002CUL
#define N_AR_MAX 0x0000002DUL
#define N_BS_MAX 0x0000002EUL
#define N_CR_MAX 0x0000002FUL
#define N_CS_MIN 0x00000030UL
#define ECHO_PHYSICAL_CHANNEL_TX 0x00000031UL
#define BUS_NORMAL 0x00000000UL
#define BUS_PLUS 0x00000001UL
#define BUS_MINUS 0x00000002UL
#define NO_PARITY 0x00000000UL
#define ODD_PARITY 0x00000001UL
#define EVEN_PARITY 0x00000002UL
#define DATA_BITS_8 0x00000000UL
#define DATA_BITS_7 0x00000001UL
#define ISO_STD_INIT 0x00000000UL
#define ISO_INV_KB2 0x00000001UL
#define ISO_INV_ADD 0x00000002UL
#define ISO_9141_STD 0x00000003UL
#define DISABLE_ECHO 0x00000000UL
#define ENABLE_ECHO 0x00000001UL
#define TX_MSG_TYPE 0x00000001UL
#define START_OF_MESSAGE 0x00000002UL
#define RX_BREAK 0x00000004UL
#define TX_INDICATION_SUCCESS 0x00000008UL
#define ISO15765_PADDING_ERROR 0x00000010UL
#define RX_ERROR 0x00000020UL
#define BUFFER_OVERFLOW 0x00000040UL
#define ISO15765_ADDR_TYPE 0x00000080UL
#define CAN_29BIT_ID 0x00000100UL
#define TX_FAILED 0x00000200UL
#define ISO15765_FRAME_PAD 0x00000040UL
#define WAIT_P3_MIN_ONLY 0x00000200UL
#define SCI_TX_VOLTAGE 0x00800000UL
#define SCI_MODE 0x00400000UL
#define K_LINE_ONLY 0x00001000UL
#define CAN_ID_BOTH 0x00000800UL
#define CHECKSUM_DISABLED 0x00000200UL
#define FULL_DUPLEX 0x00000001UL
#define DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE 0x0000000DUL

#pragma pack(push, 1)
typedef struct {
    J2534_ULONG ProtocolID;
    J2534_ULONG MsgHandle;
    J2534_ULONG RxStatus;
    J2534_ULONG TxFlags;
    J2534_ULONG Timestamp;
    J2534_ULONG DataLength;
    J2534_ULONG ExtraDataIndex;
    unsigned char *DataBuffer;
    J2534_ULONG DataBufferSize;
} PASSTHRU_MSG;

typedef struct {
    char DeviceName[80];
    J2534_ULONG DeviceAvailable;
    J2534_ULONG DeviceDLLFWStatus;
    J2534_ULONG DeviceConnectMedia;
    J2534_ULONG DeviceConnectSpeed;
    J2534_ULONG DeviceSignalQuality;
    J2534_ULONG DeviceSignalStrength;
} SDEVICE;

typedef struct {
    J2534_ULONG Connector;
    J2534_ULONG NumOfResources;
    J2534_ULONG *ResourceListPtr;
} RESOURCE_STRUCT;

typedef struct {
    J2534_ULONG LocalTxFlags;
    J2534_ULONG RemoteTxFlags;
    unsigned char LocalAddress[5];
    unsigned char RemoteAddress[5];
} ISO15765_CHANNEL_DESCRIPTOR;

typedef struct {
    J2534_ULONG ChannelCount;
    J2534_ULONG ChannelThreshold;
    J2534_ULONG *ChannelList;
} SCHANNELSET;

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

#ifndef OPENDIAG_TYPES_ONLY
#ifdef __cplusplus
extern "C" {
#endif

J2534_EXPORT J2534_LONG J2534_CALL PassThruScanForDevices(J2534_ULONG *pDeviceCount);
J2534_EXPORT J2534_LONG J2534_CALL PassThruGetNextDevice(SDEVICE *psDevice);
J2534_EXPORT J2534_LONG J2534_CALL PassThruOpen(const char *pName, J2534_ULONG *pDeviceID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruClose(J2534_ULONG DeviceID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruConnect(J2534_ULONG DeviceID,
                                                   J2534_ULONG ProtocolID,
                                                   J2534_ULONG Flags,
                                                   J2534_ULONG BaudRate,
                                                   RESOURCE_STRUCT ResourceStruct,
                                                   J2534_ULONG *pChannelID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruDisconnect(J2534_ULONG ChannelID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruLogicalConnect(J2534_ULONG PhysicalChannelID,
                                                          J2534_ULONG ProtocolID,
                                                          J2534_ULONG Flags,
                                                          void *pChannelDescriptor,
                                                          J2534_ULONG *pChannelID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruLogicalDisconnect(J2534_ULONG ChannelID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruSelect(SCHANNELSET *ChannelSetPtr,
                                                  J2534_ULONG SelectType,
                                                  J2534_ULONG Timeout);
J2534_EXPORT J2534_LONG J2534_CALL PassThruReadMsgs(J2534_ULONG ChannelID,
                                                    PASSTHRU_MSG *pMsg,
                                                    J2534_ULONG *pNumMsgs,
                                                    J2534_ULONG Timeout);
J2534_EXPORT J2534_LONG J2534_CALL PassThruQueueMsgs(J2534_ULONG ChannelID,
                                                     PASSTHRU_MSG *pMsg,
                                                     J2534_ULONG *pNumMsgs);
J2534_EXPORT J2534_LONG J2534_CALL PassThruStartPeriodicMsg(J2534_ULONG ChannelID,
                                                            PASSTHRU_MSG *pMsg,
                                                            J2534_ULONG *pMsgID,
                                                            J2534_ULONG TimeInterval);
J2534_EXPORT J2534_LONG J2534_CALL PassThruStopPeriodicMsg(J2534_ULONG ChannelID,
                                                           J2534_ULONG MsgID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruStartMsgFilter(J2534_ULONG ChannelID,
                                                          J2534_ULONG FilterType,
                                                          PASSTHRU_MSG *pMaskMsg,
                                                          PASSTHRU_MSG *pPatternMsg,
                                                          J2534_ULONG *pFilterID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruStopMsgFilter(J2534_ULONG ChannelID,
                                                         J2534_ULONG FilterID);
J2534_EXPORT J2534_LONG J2534_CALL PassThruSetProgrammingVoltage(J2534_ULONG DeviceID,
                                                                 RESOURCE_STRUCT ResourceStruct,
                                                                 J2534_ULONG Voltage);
J2534_EXPORT J2534_LONG J2534_CALL PassThruReadVersion(J2534_ULONG DeviceID,
                                                       char *pFirmwareVersion,
                                                       char *pDllVersion,
                                                       char *pApiVersion);
J2534_EXPORT J2534_LONG J2534_CALL PassThruGetLastError(char *pErrorDescription);
J2534_EXPORT J2534_LONG J2534_CALL PassThruIoctl(J2534_ULONG ControlTarget,
                                                 J2534_ULONG IoctlID,
                                                 void *InputPtr,
                                                 void *OutputPtr);

#ifdef __cplusplus
}
#endif

#endif // OPENDIAG_TYPES_ONLY
