/* Configuration of Spektrum's reference SRXL2 slave for tests/srxl2_ref.c (differential testing). */
#ifndef _SRXL_CONFIG_H_
#define _SRXL_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define SRXL_NUM_OF_BUSES 1
#define SRXL_CRC_OPTIMIZE_MODE SRXL_CRC_OPTIMIZE_SPEED

void ref_uart_send(uint8_t uart, uint8_t* buffer, uint8_t length);
void ref_uart_baud(uint8_t uart, uint32_t baud);

static inline void srxlChangeBaudRate(uint8_t uart, uint32_t baudRate) { ref_uart_baud(uart, baudRate); }
static inline void srxlSendOnUart(uint8_t uart, uint8_t* pBuffer, uint8_t length) { ref_uart_send(uart, pBuffer, length); }
static inline void srxlFillTelemetry(SrxlTelemetryData* pTelemetryData) { (void)pTelemetryData; }
static inline void srxlReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
{
    (void)pChannelData;
    (void)isFailsafe;
}
static inline bool srxlOnBind(SrxlFullID device, SrxlBindData info)
{
    (void)device;
    (void)info;
    return true;
}
static inline void srxlOnVtx(SrxlVtxData* pVtxData) { (void)pVtxData; }
static inline void srxlEnterCriticalSection(void) {}
static inline void srxlExitCriticalSection(void) {}

#ifdef __cplusplus
}
#endif

#endif
