#include <Arduino.h>
#include <stdio.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

const int ledPin = 5;

const int UARTPin = 17;

const uart_port_t uart_num = UART_NUM_2;


void setup() {


    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

    //MARK: UART setup

    // Configure UART parameters
    pinMode (ledPin, OUTPUT);
    Serial2.begin(9600);
    Serial.begin(9600);

    // ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // // Set UART pins(TX: IO16 (UART2 default), RX: IO17 (UART2 default), RTS: IO18, CTS: IO19)
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));
}

void loop() {
    /*
    digitalWrite (ledPin, HIGH);
    delay(500);
    digitalWrite (ledPin, LOW);
    delay(500);
    */

    // char* test_str = 'This is a test string.\n';
    // uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));
    Serial.write("yeet");
    Serial2.write("yeet");
    Serial.println("sending");
    delay(500);

} 