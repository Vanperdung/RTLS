#include "header.h"

#include "uart_user.h"
#include "common_user.h"
#include "mqtt_user.h"
#include "wifi_user.h"
#include "smartcfg.h"
#include "logo.h"

static const char *TAG = "MAIN";
RTC_NOINIT_ATTR gateway_mode_t mode;

extern char mac_address[6];
extern char ip_address[16];
extern char ssid[32];

uint16_t ID = 0;
wireless_state_t wireless_state = DISCONNECTED;
extern esp_mqtt_client_handle_t client;
extern char topic_response[64];

void getIdDevice(void)
{
    esp_base_mac_addr_get((uint8_t *)mac_address);
    ID = mac_address[4] << 8 | mac_address[5];
}

void app_main(void)
{
    esp_err_t err;
    TickType_t hbTime = xTaskGetTickCount() / portTICK_RATE_MS;
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    printf("%s", RTLS_LOGO);
    wifi_init();
    getIdDevice();
    if (mode == SMARTCONFIG)
    {
        smartconfig_init();
    }
    else
    {
        wifi_config_t wifi_cfg = {
            .sta = {
                .pmf_cfg = {
                    .capable = true,
                    .required = false,
                },
            },
        };
        if (esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_cfg) == ESP_OK)
        {
            if (strlen((char *)wifi_cfg.sta.ssid) > 0)
            {
                ESP_LOGI(TAG, "Wifi configuration already stored in flash partition called NVS");
                ESP_LOGI(TAG, "%s", wifi_cfg.sta.ssid);
                ESP_LOGI(TAG, "%s", wifi_cfg.sta.password);
                memcpy(ssid, wifi_cfg.sta.ssid, strlen((char *)wifi_cfg.sta.ssid));
                wifi_sta_init(wifi_cfg, WIFI_MODE_STA);
                mqtt_client_init();
            }
            else
            {
                ESP_LOGE(TAG, "Cannot read WiFi info");
            }
        }
        xTaskCreate(&uart_user_task, "UART", 4096, NULL, 10, NULL);
    }
    while (1)
    {
        if (!gpio_get_level(GPIO_NUM_0))
        {
            mode = SMARTCONFIG;
            esp_restart();
        }
        if (hbTime + HEARTBEAT_TIME * 1000 < xTaskGetTickCount() / portTICK_RATE_MS)
        {
            char hbBuff[128] = {0};
            hbTime = xTaskGetTickCount() / portTICK_RATE_MS;
            if (wireless_state == WIFI_CONNECTED)
            {
                wifi_ap_record_t wifi_info;
                ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&wifi_info));
                sprintf(hbBuff, "{\"id\":%d,\"method\":\"heartbeat\",\"ssid\":%s,\"rssi\":%d}", ID, wifi_info.ssid, wifi_info.rssi);
                esp_mqtt_client_publish(client, topic_response, hbBuff, strlen(hbBuff), 0, 0);
            }
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}
