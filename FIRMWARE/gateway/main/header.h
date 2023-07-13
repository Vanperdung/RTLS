#ifndef _HEADER_H_
#define _HEADER_H_
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sntp.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_https_ota.h"
#include "mqtt_client.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#define FIRMWARE_VER "1.0.0"
#define HARDWARE_VER "1.0.0"
#define IDSTR "%04x"
#define MQTT_BROKER "broker.hivemq.com"
#define TOPIC_LOCATION "tag/%d/location"
#define HEARTBEAT_TIME 30

#define UART1_TXD_PIN 17
#define UART1_RXD_PIN 16

typedef enum 
{
    DISCONNECTED,
    WIFI_CONNECTED,
    ETHERNET_CONNECTED,
} wireless_state_t;

#endif