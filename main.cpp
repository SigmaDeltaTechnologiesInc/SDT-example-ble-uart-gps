/* SDT-example-ble-uart-gps
 * 
 * Copyright (c) 2018 Sigma Delta Technologies Inc.
 * 
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "mbed.h"
#include "features/FEATURE_BLE/ble/BLE.h"
#include "features/FEATURE_BLE/ble/services/UARTService.h"

/* Serial */
#define BAUDRATE 9600
Serial g_Serial_pc(USBTX, USBRX, BAUDRATE);

/* DigitalOut */
#define LED_ON      0
#define LED_OFF     1
DigitalOut g_DO_LedRed(LED_RED, LED_OFF);
DigitalOut g_DO_LedGreen(LED_GREEN, LED_OFF);
DigitalOut g_DO_LedBlue(LED_BLUE, LED_OFF);
DigitalOut* g_pDO_Led = &g_DO_LedBlue;

/* Ticker */
Ticker g_Ticker;

/* EventQueue */
EventQueue g_EventQueue(/* event count */ 16 * /* event size */ 32);

/* BLE */
#define BLE_DEVICE_NAME "SDT Device"
BLE& g_pBle = BLE::Instance();                          // you can't use this name, 'ble', because this name is already declared in UARTService.h

/* UART service */
#define UARTSERVICE_WRITE_DATA_MAX  20
UARTService* g_pUartService;

/* Variable */
bool g_b_BleConnect = false;
bool g_b_UartStart = false;
unsigned char g_puc_UartRxBuf[50] = {'\0', };       // DataBuffer to write by BLE



void callbackTicker(void) {
    // g_Serial_pc.printf("LED Toggle\n");
    *g_pDO_Led = !(*g_pDO_Led);
}

void readWriteData(void) {
    if (g_b_UartStart) {
        char data = g_Serial_pc.getc();

        if (data == '$') {                              // GPS data is $GPxxxxxxxxxx
            unsigned char rxBufIndex = 0;

            while (true) {
                data = g_Serial_pc.getc();
                if (data != '$') {
                    g_puc_UartRxBuf[rxBufIndex++] = data;
                }
                else {
                    break;
                }
            }
            g_Serial_pc.printf("%s\n", g_puc_UartRxBuf);        // For debug

            if (g_pUartService) {
                g_pUartService->write(g_puc_UartRxBuf, UARTSERVICE_WRITE_DATA_MAX); // Elements of first parameter(const void *_buffer) are copied to 'sendBuffer' in write() just as much second parameter(size_t length).
                                                                                    // When the number of elements in sendBuffer is 20, the data of sendBuffer will be transmitted by BLE.
            }
        }
    }
}

void callbackPeriodicEvent(void) {
    if (g_pBle.gap().getState().connected) {
        g_EventQueue.call(readWriteData);
    }
}

void callbackEventsToProcess(BLE::OnEventsToProcessCallbackContext* context) {
    g_EventQueue.call(Callback<void()>(&g_pBle, &BLE::processEvents));
}

void callbackBleDataWritten(const GattWriteCallbackParams* params) {
    if ((g_pUartService != NULL) && (params->handle == g_pUartService->getTXCharacteristicHandle())) {
        // uint16_t bytesRead = params->len;
        const unsigned char* pBleRxBuf = params->data;
        // g_Serial_pc.printf("data from BLE : %s\r\n", params->data);  // For debug

        if (pBleRxBuf[0] == 's') {
            g_b_UartStart = true;
        }
        else if (pBleRxBuf[0] == 'p') {
            g_b_UartStart = false;
        }
    }
}

void callbackBleConnection(const Gap::ConnectionCallbackParams_t* params) {
    // g_Serial_pc.printf("Connected!\n");
    g_b_BleConnect = true;
    *g_pDO_Led = LED_OFF;
    g_pDO_Led = &g_DO_LedGreen;
}

void callbackBleDisconnection(const Gap::DisconnectionCallbackParams_t* params) {
    g_Serial_pc.printf("Disconnected!\n");
    g_Serial_pc.printf("Restarting the advertising process\n\r");
    g_b_BleConnect = false;
    *g_pDO_Led = LED_OFF;
    g_pDO_Led = &g_DO_LedBlue;
    g_pBle.gap().startAdvertising();
}

void callbackBleInitComplete(BLE::InitializationCompleteCallbackContext* params) {
    BLE& ble = params->ble;                     // 'ble' equals g_pBle declared in global
    ble_error_t error = params->error;          // 'error' has BLE_ERROR_NONE if the initialization procedure started successfully.

    if (error == BLE_ERROR_NONE) {
        // g_Serial_pc.printf("Initialization completed successfully\n");
    }
    else {
        /* In case of error, forward the error handling to onBleInitError */
        // g_Serial_pc.printf("Initialization failled\n");
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        // g_Serial_pc.printf("ID of BLE instance is not DEFAULT_INSTANCE\n");
        return;
    }

    /* Setup UARTService */
    g_pUartService = new UARTService(ble);

    /* Setup and start advertising */
    ble.gattServer().onDataWritten(callbackBleDataWritten);
    ble.gap().onConnection(callbackBleConnection);
    ble.gap().onDisconnection(callbackBleDisconnection);
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000);     // Advertising interval in units of milliseconds
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME, (const uint8_t *)BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));
    ble.gap().startAdvertising();
    // g_Serial_pc.printf("Start advertising\n");
}

int main(void) {
    // g_Serial_pc.printf("< Sigma Delta Technologies Inc. >\n\r");

    /* Init BLE */
    g_pBle.onEventsToProcess(callbackEventsToProcess);
    g_pBle.init(callbackBleInitComplete);

    /* Check whether IC is running or not */
    g_Ticker.attach(callbackTicker, 1);
    
    g_EventQueue.call_every(300, callbackPeriodicEvent);
    g_EventQueue.dispatch_forever();

    return 0;
}
