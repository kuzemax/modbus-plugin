#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <time.h>
#include <glib.h>  // 包含 glib.h 头文件
#include <dlfcn.h> // 包含 dlfcn.h 头文件
#include "arkime.h"

extern ArkimeConfig_t config;

#define ARKIME_API_VERSION 542
#define ARKIME_SESSIONID_LEN 40

// Modbus 功能码定义
#define MODBUS_READ_COILS          0x01
#define MODBUS_READ_DISCRETE_INPUTS 0x02
#define MODBUS_READ_HOLDING_REGISTERS 0x03
#define MODBUS_READ_INPUT_REGISTERS 0x04
#define MODBUS_WRITE_SINGLE_COIL    0x05
#define MODBUS_WRITE_SINGLE_REGISTER 0x06
#define MODBUS_WRITE_MULTIPLE_COILS  0x0F
#define MODBUS_WRITE_MULTIPLE_REGISTERS 0x10

// Modbus 异常码定义
#define MODBUS_EXC_ILLEGAL_FUNCTION      0x01
#define MODBUS_EXC_ILLEGAL_DATA_ADDRESS  0x02
#define MODBUS_EXC_ILLEGAL_DATA_VALUE     0x03
#define MODBUS_EXC_SLAVE_DEVICE_FAILURE  0x04
#define MODBUS_EXC_ACKNOWLEDGE           0x05
#define MODBUS_EXC_SLAVE_DEVICE_BUSY     0x06
#define MODBUS_EXC_NEGATIVE_ACKNOWLEDGE  0x07
#define MODBUS_EXC_MEMORY_PARITY_ERROR   0x08

// 定义 Modbus 数据结构
typedef struct {
    uint16_t transactionId;
    uint16_t protocolId;
    uint16_t length;
    uint8_t  unitId;
    uint8_t  functionCode;
    uint16_t address;       // 数据地址
    uint16_t quantity;      // 读取或者写入的数量
    uint16_t value;         // 单个寄存器或线圈的值
    uint8_t *data;          // 写入或读取的数据
    uint16_t dataLength;    // 数据长度
    uint8_t  exceptionCode; // 异常码
    char   *description;    // 描述信息
} ModbusData;

// 全局变量，存储字段位置
static int modbus_transaction_id_field;
static int modbus_function_code_field;
static int modbus_description_field;
static int modbus_unit_id_field;
static int modbus_address_field;
static int modbus_quantity_field;
static int modbus_data_field;
static int modbus_src_port_field;
static int modbus_dest_port_field;
static int modbus_exc_code_field;

// 函数声明 (保持不变)
void modbus_free_data(ModbusData *data);
void modbus_process_tcp_payload(ArkimeSession_t *session, const uint8_t *payload, int len);
int modbus_parse_pdu(const uint8_t *payload, int len, ModbusData *modbusData, ArkimeSession_t *session);

// 自定义错误打印函数，带时间戳
void modbus_log(const char *format, ...) {
    va_list args;
    va_start(args, format);

    time_t timer;
    char buffer[26];
    struct tm tm_info; // 声明 struct tm 变量
    time(&timer);

    localtime_r(&timer, &tm_info); // 使用 localtime_r 函数

    strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", &tm_info); // 传递指针

    fprintf(stderr, "%s [modbus_plugin] ", buffer);
    vfprintf(stderr, format, args);
    fprintf(stderr, "\n");
    va_end(args);
    fflush(stderr);
}

/******************************************************************************/
// 插件初始化函数
void arkime_plugin_init() {
    modbus_log("arkime_plugin_init() called");

    // 注册插件
    int rc = arkime_plugins_register_internal("modbus", TRUE, sizeof(ArkimeSession_t), ARKIME_API_VERSION);
    if (rc == -1) {
        fprintf(stderr, "ERROR: Couldn't register plugin\n");
        exit(1);
    }

    // 定义 Arkime 字段
    modbus_transaction_id_field = arkime_field_define("modbus", "integer",
                                                     "modbus.transactionId", "Modbus Transaction ID", "modbus.transactionId",
                                                     "Modbus Transaction ID",
                                                     ARKIME_FIELD_TYPE_INT_HASH, 0,
                                                     NULL);

    modbus_function_code_field = arkime_field_define("modbus", "integer",
                                                   "modbus.functionCode", "Modbus Function Code", "modbus.functionCode",
                                                   "Modbus Function Code",
                                                   ARKIME_FIELD_TYPE_INT_HASH, 0,
                                                   NULL);

    modbus_description_field = arkime_field_define("modbus", "string",
                                                  "modbus.description", "Modbus Description", "modbus.description",
                                                  "Modbus Description",
                                                  ARKIME_FIELD_TYPE_STR_HASH, 0,
                                                  NULL);

    modbus_unit_id_field = arkime_field_define("modbus", "integer",
                                                      "modbus.unitId", "Modbus Unit ID", "modbus.unitId",
                                                      "Modbus Unit ID",
                                                      ARKIME_FIELD_TYPE_INT_HASH, 0,
                                                      NULL);

    modbus_address_field = arkime_field_define("modbus", "integer",
                                                       "modbus.address", "Modbus Address", "modbus.address",
                                                       "Modbus Address",
                                                       ARKIME_FIELD_TYPE_INT_HASH, 0,
                                                       NULL);

    modbus_quantity_field = arkime_field_define("modbus", "integer",
                                                        "modbus.quantity", "Modbus Quantity", "modbus.quantity",
                                                        "Modbus Quantity",
                                                        ARKIME_FIELD_TYPE_INT_HASH, 0,
                                                        NULL);

    modbus_data_field = arkime_field_define("modbus", "string",
                                                    "modbus.data", "Modbus Data", "modbus.data",
                                                    "Modbus Data",
                                                    ARKIME_FIELD_TYPE_STR_HASH, 0,
                                                    NULL);
    modbus_src_port_field = arkime_field_define("modbus", "integer",
                                                        "modbus.srcPort", "Modbus Source Port", "modbus.srcPort",
                                                        "Modbus Source Port",
                                                        ARKIME_FIELD_TYPE_INT_HASH, 0,
                                                        NULL);

    modbus_dest_port_field = arkime_field_define("modbus", "integer",
                                                     "modbus.destPort", "Modbus Destination Port", "modbus.destPort",
                                                     "Modbus Destination Port",
                                                     ARKIME_FIELD_TYPE_INT_HASH, 0,
                                                     NULL);
    modbus_exc_code_field = arkime_field_define("modbus", "integer",
                                                     "modbus.exceptionCode", "Modbus Exception Code", "modbus.exceptionCode",
                                                     "Modbus Exception Code",
                                                     ARKIME_FIELD_TYPE_INT_HASH, 0,
                                                     NULL);

    // 注册 TCP 端口 502 的流量处理回调函数
    arkime_parsers_classifier_register_port("modbus", NULL, 502, ARKIME_PARSERS_PORT_TCP, (ArkimeClassifyFunc)modbus_process_tcp_payload);

    modbus_log("modbus plugin initialized!");
}

/******************************************************************************/
// 判断是否是 Modbus 数据 (更严格的检查) (保持不变)
int modbus_is_modbus(const uint8_t *payload, int len) {
    if (len < 7) {
        modbus_log("modbus_is_modbus: len < 7, returning FALSE");
        return FALSE;
    }

    // 检查协议标识符, Modbus TCP 固定为 0
    uint16_t protocolId = ntohs(*(uint16_t*)(payload + 2));
    if (protocolId != 0) {
        modbus_log("modbus_is_modbus: protocolId != 0, returning FALSE");
        return FALSE;
    }

    // 检查长度字段，确保其值与实际长度匹配 (减去前 6 个字节)
    uint16_t expectedLength = ntohs(*(uint16_t*)(payload + 4));
    if (expectedLength != len - 6) {
        modbus_log("modbus_is_modbus: expectedLength != len - 6, returning FALSE");
        return FALSE;
    }

    modbus_log("modbus_is_modbus: is Modbus TCP");
    return TRUE;
}

/******************************************************************************/
// 处理 TCP 数据包
void modbus_process_tcp_payload(ArkimeSession_t *session, const uint8_t *payload, int len) {
    char sessionString[ARKIME_SESSIONID_LEN];
    arkime_session_id_string(session->sessionId, sessionString);
    modbus_log("modbus_process_tcp_payload called, len = %d, session id = %s", len, sessionString);

    if (len <= 0) {
        modbus_log("modbus_process_tcp_payload: len <= 0, returning");
        return;
    }

    // 判断是否是 Modbus 数据
    if (!modbus_is_modbus(payload, len)) {
        modbus_log("modbus_process_tcp_payload: not Modbus traffic, returning");
        return;
    }

    ModbusData *modbusData = ARKIME_TYPE_ALLOC0(ModbusData); // 使用 Arkime 的内存分配函数
    if (!modbusData) {
        modbus_log("ERROR - 内存分配失败");
        return;
    }

    modbus_log("modbus_process_tcp_payload: Modbus data detected, parsing PDU");

    // 解析 Modbus PDU
    if (modbus_parse_pdu(payload, len, modbusData, session) == 0) { // 传递 session 指针
        modbus_log("modbus_process_tcp_payload: modbus_parse_pdu success, adding fields");
        // 添加 Modbus 数据到 Arkime 字段
        arkime_field_int_add(modbus_transaction_id_field, session, modbusData->transactionId);
        modbus_log("modbus_process_tcp_payload: added transactionId = %u", modbusData->transactionId);

        arkime_field_int_add(modbus_function_code_field, session, modbusData->functionCode);
        modbus_log("modbus_process_tcp_payload: added functionCode = %u", modbusData->functionCode);

        arkime_field_int_add(modbus_unit_id_field, session, modbusData->unitId);
        modbus_log("modbus_process_tcp_payload: added unitId = %u", modbusData->unitId);

        arkime_field_int_add(modbus_src_port_field, session, session->port1);
        modbus_log("modbus_process_tcp_payload: added srcPort = %u", session->port1);

        arkime_field_int_add(modbus_dest_port_field, session, session->port2);
        modbus_log("modbus_process_tcp_payload: added destPort = %u", session->port2);

        if (modbusData->exceptionCode != 0){
            arkime_field_int_add(modbus_exc_code_field, session,  modbusData->exceptionCode);
            modbus_log("modbus_process_tcp_payload: added exceptionCode = %u", modbusData->exceptionCode);
        }

        if (modbusData->address != 0xFFFF){
            arkime_field_int_add(modbus_address_field, session,  modbusData->address);
             modbus_log("modbus_process_tcp_payload: added address = %u",  modbusData->address);
        }

         if (modbusData->quantity != 0xFFFF){
            arkime_field_int_add(modbus_quantity_field, session,  modbusData->quantity);
             modbus_log("modbus_process_tcp_payload: added quantity = %u",   modbusData->quantity);
        }

        // 避免 NULL 指针解引用
        if (modbusData->description != NULL) {
            arkime_field_string_add(modbus_description_field, session, modbusData->description, -1, TRUE);
            modbus_log("modbus_process_tcp_payload: added description = %s",modbusData->description);
        } else {
            modbus_log("modbus_process_tcp_payload: description is NULL");
        }

        if (modbusData->dataLength > 0 && modbusData->data != NULL) {
            char hex_string[1024];
            arkime_sprint_hex_string(hex_string, modbusData->data, modbusData->dataLength);
            arkime_field_string_add(modbus_data_field, session, hex_string, -1, TRUE);
            modbus_log("modbus_process_tcp_payload: added data: %s", hex_string);  // 打印实际数据
             ARKIME_SIZE_FREE(modbus_data, modbusData->data);
             modbusData->data = NULL;
             modbusData->dataLength=0;
        }

        modbus_free_data(modbusData);
    } else {
        modbus_log("modbus_process_tcp_payload: modbus_parse_pdu failed");
        modbus_free_data(modbusData);
    }
}

/******************************************************************************/
// 解析 Modbus PDU (保持不变)
int modbus_parse_pdu(const uint8_t *payload, int len, ModbusData *modbusData, ArkimeSession_t *session) {
    modbus_log("modbus_parse_pdu called, len = %d,functionCode=%d", len,payload[7]);
    if (len < 8) {
        modbus_log("modbus_parse_pdu: len < 8, returning -1");
        return -1; // 至少需要 8 个字节
    }

    modbusData->transactionId = ntohs(*(uint16_t*)(payload + 0));
    modbus_log("modbus_parse_pdu: transactionId = %u",modbusData->transactionId);
    modbusData->protocolId    = ntohs(*(uint16_t*)(payload + 2));
    modbus_log("modbus_parse_pdu: protocolId = %u",modbusData->protocolId);
    modbusData->length        = ntohs(*(uint16_t*)(payload + 4));
    modbus_log("modbus_parse_pdu: length = %u",modbusData->length);
    modbusData->unitId        = payload[6];
    modbus_log("modbus_parse_pdu: unitId = %u",modbusData->unitId);
    modbusData->functionCode  = payload[7];
    modbus_log("modbus_parse_pdu: functionCode = %u",modbusData->functionCode);

    const uint8_t *pdu = payload + 8;
    int pdu_len = len - 8;

    modbusData->description = "Unknown"; // 设置默认值
    //设置默认值，避免显示异常
    modbusData->address = 0xFFFF;
    modbusData->quantity = 0xFFFF;
    modbusData->exceptionCode = 0x00;

    switch (modbusData->functionCode) {
        case MODBUS_READ_COILS:         // 0x01
        case MODBUS_READ_DISCRETE_INPUTS:  // 0x02
        case MODBUS_READ_HOLDING_REGISTERS: // 0x03
        case MODBUS_READ_INPUT_REGISTERS:   // 0x04
        {
            if (pdu_len < 4) {
                modbus_log("modbus_parse_pdu: read function, pdu_len < 4, returning -1");
                return -1;
            }
            modbusData->address  = ntohs(*(uint16_t*)(pdu + 0));
            modbus_log("modbus_parse_pdu: address = %u", modbusData->address);
            modbusData->quantity = ntohs(*(uint16_t*)(pdu + 2));
            modbus_log("modbus_parse_pdu: quantity = %u",modbusData->quantity);

            switch (modbusData->functionCode) {
                case MODBUS_READ_COILS:
                    modbusData->description = "Read Coils";
                    modbus_log("modbus_parse_pdu: description = Read Coils");
                    break;
                case MODBUS_READ_DISCRETE_INPUTS:
                    modbusData->description = "Read Discrete Inputs";
                    modbus_log("modbus_parse_pdu: description = Read Discrete Inputs");
                    break;
                case MODBUS_READ_HOLDING_REGISTERS:
                    modbusData->description = "Read Holding Registers";
                    modbus_log("modbus_parse_pdu: description = Read Holding Registers");
                    break;
                case MODBUS_READ_INPUT_REGISTERS:
                    modbusData->description = "Read Input Registers";
                    modbus_log("modbus_parse_pdu: description = Read Input Registers");
                    break;
            }
            break;
        }

        case MODBUS_WRITE_SINGLE_COIL:    // 0x05
        case MODBUS_WRITE_SINGLE_REGISTER: // 0x06
        {
            if (pdu_len < 4) {
                modbus_log("modbus_parse_pdu: write single function, pdu_len < 4, returning -1");
                return -1;
            }

            modbusData->address = ntohs(*(uint16_t*)(pdu + 0));
            modbus_log("modbus_parse_pdu: address = %u", modbusData->address);
            modbusData->value   = ntohs(*(uint16_t*)(pdu + 2));
            modbus_log("modbus_parse_pdu: value = %u",modbusData->value);

            switch (modbusData->functionCode) {
                case MODBUS_WRITE_SINGLE_COIL:
                    modbusData->description = "Write Single Coil";
                    modbus_log("modbus_parse_pdu: description = Write Single Coil");
                    break;
                case MODBUS_WRITE_SINGLE_REGISTER:
                    modbusData->description = "Write Single Register";
                    modbus_log("modbus_parse_pdu: description = Write Single Register");
                    break;
            }
            break;
        }

        case MODBUS_WRITE_MULTIPLE_COILS:    // 0x0F
        case MODBUS_WRITE_MULTIPLE_REGISTERS: // 0x10
        {
            if (pdu_len < 5) {
                modbus_log("modbus_parse_pdu: write multiple function, pdu_len < 5, returning -1");
                return -1;
            }

            modbusData->address  = ntohs(*(uint16_t*)(pdu + 0));
            modbus_log("modbus_parse_pdu: address = %u",modbusData->address);
            modbusData->quantity = ntohs(*(uint16_t*)(pdu + 2));
            modbus_log("modbus_parse_pdu: quantity = %u",modbusData->quantity);
            modbusData->dataLength = pdu[4];
            modbus_log("modbus_parse_pdu: dataLength = %u",modbusData->dataLength);

            if (pdu_len < 5 + modbusData->dataLength) {
                modbus_log("modbus_parse_pdu: write multiple function, pdu_len < 5 + dataLength, returning -1");
                return -1;
            }

            modbusData->data = ARKIME_SIZE_ALLOC0(modbus_data, modbusData->dataLength); // 使用 Arkime 的内存分配函数
            if (!modbusData->data) {
                modbus_log("ERROR - 内存分配失败");
                return -1;
            }
            memcpy(modbusData->data, pdu + 5, modbusData->dataLength);

            switch (modbusData->functionCode) {
                case MODBUS_WRITE_MULTIPLE_COILS:
                    modbusData->description = "Write Multiple Coils";
                    modbus_log("modbus_parse_pdu: description = Write Multiple Coils");
                    break;
                case MODBUS_WRITE_MULTIPLE_REGISTERS:
                    modbusData->description = "Write Multiple Registers";
                    modbus_log("modbus_parse_pdu: description = Write Multiple Registers");
                    break;
            }
            break;
        }

        case 0x80 ... 0xFF: // Exception Response
        {
            if (pdu_len < 1) {
                modbus_log("modbus_parse_pdu: exception response, pdu_len < 1, returning -1");
                return -1;
            }
            modbusData->exceptionCode = pdu[0];
            modbus_log("modbus_parse_pdu: exceptionCode = %u",modbusData->exceptionCode);

            switch (modbusData->exceptionCode) {
                case MODBUS_EXC_ILLEGAL_FUNCTION:
                    modbusData->description = "Exception: Illegal Function";
                    modbus_log("modbus_parse_pdu: description = Exception: Illegal Function");
                    break;
                case MODBUS_EXC_ILLEGAL_DATA_ADDRESS:
                    modbusData->description = "Exception: Illegal Data Address";
                    modbus_log("modbus_parse_pdu: description = Exception: Illegal Data Address");
                    break;
                case MODBUS_EXC_ILLEGAL_DATA_VALUE:
                    modbusData->description = "Exception: Illegal Data Value";
                    modbus_log("modbus_parse_pdu: description = Exception: Illegal Data Value");
                    break;
                case MODBUS_EXC_SLAVE_DEVICE_FAILURE:
                    modbusData->description = "Exception: Slave Device Failure";
                    modbus_log("modbus_parse_pdu: description = Exception: Slave Device Failure");
                    break;
                case MODBUS_EXC_ACKNOWLEDGE:
                    modbusData->description = "Exception: Acknowledge";
                    modbus_log("modbus_parse_pdu: description = Exception: Acknowledge");
                    break;
                case MODBUS_EXC_SLAVE_DEVICE_BUSY:
                    modbusData->description = "Exception: Slave Device Busy";
                    modbus_log("modbus_parse_pdu: description = Exception: Slave Device Busy");
                    break;
                case MODBUS_EXC_NEGATIVE_ACKNOWLEDGE:
                    modbusData->description = "Exception: Negative Acknowledge";
                    modbus_log("modbus_parse_pdu: description = Exception: Negative Acknowledge");
                    break;
                case MODBUS_EXC_MEMORY_PARITY_ERROR:
                    modbusData->description = "Exception: Memory Parity Error";
                    modbus_log("modbus_parse_pdu: description = Exception: Memory Parity Error");
                    break;
                default:
                    modbusData->description = "Exception: Unknown Error";
                    modbus_log("modbus_parse_pdu: description = Exception: Unknown Error");
                    break;
            }
            break;
        }

        default:
            modbus_log("modbus_parse_pdu: unknown function code, returning -1");
            return -1;
    }

    return 0;
}

/******************************************************************************/
// 释放 ModbusData 结构体内存 (保持不变)
void modbus_free_data(ModbusData *data) {
    if (data) {
        if (data->data) {
            ARKIME_SIZE_FREE(modbus_data, data->data); // 使用 Arkime 的内存释放函数
        }
        ARKIME_TYPE_FREE(ModbusData, data); // 使用 Arkime 的内存释放函数
    }
}

/******************************************************************************/
// 供 Arkime 加载插件的函数
#ifndef UNIT_TEST
void arkime_plugin_load() {

    modbus_log("arkime_plugin_load() called");
    arkime_plugin_init();
}
#endif
