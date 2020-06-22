/*
 * emsuart.cpp
 *
 * The low level UART code for ESP8266 to read and write to the EMS bus via uart
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 */

#include "emsuart.h"
#include "ems.h"
#include <user_interface.h>

_EMSRxBuf * pEMSRxBuf;
_EMSRxBuf * paEMSRxBuf[EMS_MAXBUFFERS];
uint8_t     emsRxBufIdx  = 0;
// uint32_t    emsRxTime    = 0;
bool        drop_next_rx = true;

os_event_t recvTaskQueue[EMSUART_recvTaskQueueLen]; // our Rx queue

//
// Main interrupt handler
// Important: do not use ICACHE_FLASH_ATTR !
//
static void emsuart_rx_intr_handler(void * para) {
    static uint8_t length;
    static uint8_t uart_buffer[EMS_MAXBUFFERSIZE];

    if (USIS(EMSUART_UART) & ((1 << UIBD))) { // BREAK detection = End of EMS data block
        length = 0;
        while ((USS(EMSUART_UART) >> USRXC) & 0xFF) {
            uint8_t rx = USF(EMSUART_UART);
            if (length < EMS_MAXBUFFERSIZE) {
                uart_buffer[length++] = rx;
            } else {
                drop_next_rx = true;
            }
        }
        USIC(EMSUART_UART) = (1 << UIBD); // INT clear the BREAK detect interrupt
        USC0(EMSUART_UART) &= ~(1 << UCBRK); // reset tx-brk
        if(!drop_next_rx) {
            ETS_UART_INTR_DISABLE(); // disable all interrupts and clear them
            pEMSRxBuf->length = length;
            os_memcpy((void *)pEMSRxBuf->buffer, (void *)&uart_buffer, pEMSRxBuf->length); // copy data into transfer buffer, including the BRK 0x00 at the end
            ETS_UART_INTR_ENABLE();                          // re-enable UART interrupts
            // emsRxTime = millis();
            system_os_post(EMSUART_recvTaskPrio, 0, 0); // call emsuart_recvTask() at next opportunity
        }
        drop_next_rx = false;
        EMS_Sys_Status.emsRxStatus = EMS_RX_STATUS_IDLE;
    }
}

/*
 * system task triggered on BRK interrupt
 * incoming received messages are always asynchronous
 * The full buffer is sent to the ems_parseTelegram() function in ems.cpp.
 */
static void ICACHE_FLASH_ATTR emsuart_recvTask(os_event_t * events) {
    _EMSRxBuf * pCurrent = pEMSRxBuf;
    pEMSRxBuf            = paEMSRxBuf[++emsRxBufIdx % EMS_MAXBUFFERS]; // next free EMS Receive buffer
    uint8_t length       = pCurrent->length;                           // number of bytes including the BRK at the end
    pCurrent->length     = 0;
 
    if (length == 2 || length > 4) {
        ems_parseTelegram((uint8_t *)pCurrent->buffer, length - 1); // transmit EMS buffer, excluding the BRK
    }
}

/*
 * flush everything left over in buffer, this clears both rx and tx FIFOs
 */
static inline void ICACHE_FLASH_ATTR emsuart_flush_fifos() {
    uint32_t tmp = ((1 << UCRXRST) | (1 << UCTXRST)); // bit mask
    USC0(EMSUART_UART) |= (tmp);                      // set bits
    USC0(EMSUART_UART) &= ~(tmp);                     // clear bits
}

/*
 * init UART0 driver
 */
void ICACHE_FLASH_ATTR emsuart_init() {
    ETS_UART_INTR_DISABLE();
    ETS_UART_INTR_ATTACH(nullptr, nullptr);

    // allocate and preset EMS Receive buffers
    for (int i = 0; i < EMS_MAXBUFFERS; i++) {
        _EMSRxBuf * p = (_EMSRxBuf *)malloc(sizeof(_EMSRxBuf));
        paEMSRxBuf[i] = p;
    }
    pEMSRxBuf = paEMSRxBuf[0]; // preset EMS Rx Buffer

    // pin settings
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0RXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);

    // set 9600, 8 bits, no parity check, 1 stop bit
    USD(EMSUART_UART)  = (UART_CLK_FREQ / EMSUART_BAUD);
    USC0(EMSUART_UART) = EMSUART_CONFIG; // 8N1

    //emsuart_flush_fifos();

    // conf1 params
    // UCTOE = RX TimeOut enable (default is 1)
    // UCTOT = RX TimeOut Threshold (7 bit) = want this when no more data after 1 characters (default is 2)
    // UCFFT = RX FIFO Full Threshold (7 bit) = want this to be 31 for 32 bytes of buffer (default was 127)
    // see https://www.espressif.com/sites/default/files/documentation/esp8266-technical_reference_en.pdf
    //
    // change: we set UCFFT to 1 to get an immediate indicator about incoming traffic.
    //         Otherwise, we're only noticed by UCTOT or RxBRK!
//     USC1(EMSUART_UART) = 0;                   // reset config first
//    if (EMS_Sys_Status.emsTxMode == EMS_TXMODE_TEST) {
        //USC1(EMSUART_UART) = (0x7F << UCFFT); // set rxbuffer 127
        USIC(EMSUART_UART) = 0xFFFF;          // clear all interupt flags
        USIE(EMSUART_UART) = (1 << UIBD);     // enable break interrupt; 
//    } else {
//        USC1(EMSUART_UART) = (0x01 << UCFFT) | (0x01 << UCTOT) | (0 << UCTOE); // enable interupts
//        USIC(EMSUART_UART) = 0xFFFF; // clear all interupts
//        USIE(EMSUART_UART) = (1 << UIBD) | (1 << UIFF) | (0 << UITO); // enable break and fifo-full
//    } 
    // set interrupts for triggers
    // enable rx break, fifo full and timeout.
    // but not frame error UIFR (because they are too frequent) or overflow UIOF because our buffer is only max 32 bytes
    // change: we don't care about Rx Timeout - it may lead to wrong readouts
    // swap Rx and Tx pins to use GPIO13 (D7) and GPIO15 (D8) respectively
    system_uart_swap();

    // set up interrupt callbacks for Rx
    system_os_task(emsuart_recvTask, EMSUART_recvTaskPrio, recvTaskQueue, EMSUART_recvTaskQueueLen);

    // disable esp debug which will go to Tx and mess up the line - see https://github.com/espruino/Espruino/issues/655
    system_set_os_print(0);


    ETS_UART_INTR_ATTACH(emsuart_rx_intr_handler, nullptr);
    drop_next_rx = true;
    ETS_UART_INTR_ENABLE();
}

/*
 * stop UART0 driver
 * This is called prior to an OTA upload and also before a save to SPIFFS to prevent conflicts
 */
void ICACHE_FLASH_ATTR emsuart_stop() {
    ETS_UART_INTR_DISABLE();
}

/*
 * re-start UART0 driver
 */
void ICACHE_FLASH_ATTR emsuart_start() {
    if (USIR(EMSUART_UART) & ((1 << UIBD))) {
        USIC(EMSUART_UART) |= (1 << UIBD); // INT clear the BREAK detect interrupt
        drop_next_rx = true;  
    }
    ETS_UART_INTR_ENABLE();
}

/*
 * Send a BRK signal
 * Which is a 11-bit set of zero's (11 cycles)
 */
void ICACHE_FLASH_ATTR emsuart_tx_brk() {
    // must make sure Tx FIFO is empty
    while (((USS(EMSUART_UART) >> USTXC) & 0xFF))
        ;

    emsuart_flush_fifos();

    // To create a 11-bit <BRK> we set TXD_BRK bit so the break signal will
    // automatically be sent when the tx fifo is empty
    uint32_t tmp = (1 << UCBRK);
    USC0(EMSUART_UART) |= (tmp); // set bit

    if (EMS_Sys_Status.emsTxMode == EMS_TXMODE_EMSPLUS) { // EMS+ mode
        delayMicroseconds(EMSUART_TX_BRK_WAIT);
    } else if (EMS_Sys_Status.emsTxMode == EMS_TXMODE_HT3) {     // junkers mode
        delayMicroseconds(EMSUART_TX_WAIT_BRK - EMSUART_TX_LAG); // 1144 (11 Bits)
    }
    USC0(EMSUART_UART) &= ~(tmp); // clear bit
}

/*
 * Send to Tx, ending with a <BRK>
 */
_EMS_TX_STATUS ICACHE_FLASH_ATTR emsuart_tx_buffer(uint8_t * buf, uint8_t len) {
    _EMS_TX_STATUS result = EMS_TX_STATUS_OK;

    // if(millis() > (emsRxTime + EMS_RX_TO_TX_TIMEOUT)) { // reply 
    //     return EMS_TX_STATUS_TIMEOUT;
    // }
    if (len) {
        if (EMS_Sys_Status.emsTxMode == EMS_TXMODE_TEST) { 
            USC0(EMSUART_UART) &= ~(1 << UCBRK); // just to make sure brk-bit is clear
            for (uint8_t i = 0; i < len ; i++) {
                USF(EMSUART_UART) = buf[i];
            }
            USC0(EMSUART_UART) |= (1 << UCBRK);
        } else if (EMS_Sys_Status.emsTxMode == EMS_TXMODE_EMSPLUS) { // With extra tx delay for EMS+
            for (uint8_t i = 0; i < len; i++) {
                USF(EMSUART_UART) = buf[i];
                delayMicroseconds(EMSUART_TX_BRK_WAIT); // https://github.com/proddy/EMS-ESP/issues/23#
            }
            emsuart_tx_brk();                                    // send <BRK>
        } else if (EMS_Sys_Status.emsTxMode == EMS_TXMODE_HT3) { // Junkers logic by @philrich
            for (uint8_t i = 0; i < len; i++) {
                USF(EMSUART_UART) = buf[i];

                // just to be safe wait for tx fifo empty (needed?)
                while (((USS(EMSUART_UART) >> USTXC) & 0xff))
                    ;

                // wait until bits are sent on wire
                delayMicroseconds(EMSUART_TX_WAIT_BYTE - EMSUART_TX_LAG + EMSUART_TX_WAIT_GAP);
            }
            emsuart_tx_brk(); // send <BRK>
        } else if (EMS_Sys_Status.emsTxMode == EMS_TXMODE_DEFAULT) {
            /*
             * based on code from https://github.com/proddy/EMS-ESP/issues/103 by @susisstrolch
             * we emit the whole telegram, with Rx interrupt disabled, collecting busmaster response in FIFO.
             * after sending the last char we poll the Rx status until either
             * - size(Rx FIFO) == size(Tx-Telegram)
             * - <BRK> is detected
             * At end of receive we re-enable Rx-INT and send a Tx-BRK in loopback mode.
             * 
             * EMS-Bus error handling
             * 1. Busmaster stops echoing on Tx w/o permission
             * 2. Busmaster cancel telegram by sending a BRK
             * 
             * Case 1. is handled by a watchdog counter which is reset on each
             * Tx attempt. The timeout should be 20x EMSUART_BIT_TIME plus 
             * some smart guess for processing time on targeted EMS device.
             * We set EMS_Sys_Status.emsTxStatus to EMS_TX_WTD_TIMEOUT and return
             * 
             * Case 2. is handled via a BRK chk during transmission.
             * We set EMS_Sys_Status.emsTxStatus to EMS_TX_BRK_DETECT and return
             * 
             */
            ETS_UART_INTR_DISABLE(); // disable rx interrupt
            emsuart_flush_fifos();

            // throw out the telegram...
            for (uint8_t i = 0; i < len && result == EMS_TX_STATUS_OK; i++) {
                uint16_t wdc            = EMS_TX_TO_COUNT;
                volatile uint8_t _usrxc = (USS(EMSUART_UART) >> USRXC) & 0xFF;
                USF(EMSUART_UART)       = buf[i]; // send each Tx byte
                while ((((USS(EMSUART_UART) >> USRXC) & 0xFF) == _usrxc) && (result == EMS_TX_STATUS_OK)) {
                    delayMicroseconds(EMSUART_BUSY_WAIT); // burn CPU cycles...
                    if (--wdc == 0) {
                        EMS_Sys_Status.emsTxStatus = result = EMS_TX_WTD_TIMEOUT;
                    }
                    if (USIR(EMSUART_UART) & (1 << UIBD)) {
                        USIC(EMSUART_UART)         = (1 << UIBD); // clear BRK detect IRQ
                        EMS_Sys_Status.emsTxStatus = result = EMS_TX_BRK_DETECT;
                    }
                }
                // after each byte wait extra time
                // delayMicroseconds(EMSUART_BIT_TIME / 4); 
            }

            // we got the whole telegram in the Rx buffer
            // on Rx-BRK (bus collision), we simply enable Rx and leave it
            // otherwise we send the final Tx-BRK in the loopback and re=enable Rx-INT.
            // worst case, we'll see an additional Rx-BRK...
            if (result != EMS_TX_BRK_DETECT) {
                // no bus collision - send terminating BRK signal
                uint16_t wdc = EMS_TX_TO_COUNT;
//                uint32_t tmp = (1 << UCLBE) | (1 << UCBRK);
                USC0(EMSUART_UART) |= (1 << UCBRK);    //  set <BRK>
                while ((!(USIR(EMSUART_UART) & (1 << UIBD))) && (--wdc > 0)) // wait until BRK detected...
                    delayMicroseconds(EMSUART_BUSY_WAIT);
                USC0(EMSUART_UART) &= ~(1 << UCBRK); // disable flags
//                USIC(EMSUART_UART) = (1 << UIBD); // clear BRK detect IRQ
//                phantomBreak       = 1;
            }
            ETS_UART_INTR_ENABLE(); // receive anything from FIFO...
        }
    }
    if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_JABBER) {
        ems_dumpBuffer("tx_buffer: ", buf, len); 
    }
    if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_ALL) {
        if (result == EMS_TX_STATUS_OK) ems_dumpBuffer("tx: ", buf, len);
        else ems_dumpBuffer("tx:-", buf, len);
    }

    return result;
}
