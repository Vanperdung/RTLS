#include "fota.h"

static const char *TAG = "FOTA";
extern const uint8_t github_cert_pem_start[] asm("_binary_git_ota_pem_start");
extern const uint8_t github_cert_pem_end[] asm("_binary_git_ota_pem_end");

esp_err_t fota_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key = %s, value = %s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len = %d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

void fota_task(void *param)
{
    char fota_url[128] = {0};
    char reset_cmd[] = "$reset*";
    strcpy(fota_url, (char *)param);
    ESP_LOGI(TAG, "OTA start, URL: %s", fota_url);
    esp_http_client_config_t ota_cfg = {
        .url = fota_url,
        .event_handler = fota_event_handler,
        .keep_alive_enable = true,
        .cert_pem = (char *)github_cert_pem_start,
    };
    esp_err_t ret = esp_https_ota(&ota_cfg);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "FOTA done, restarting...");
        uart_write_bytes(UART_NUM_1, reset_cmd, strlen(reset_cmd));
        vTaskDelay(1000 / portTICK_RATE_MS);
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG, "FOTA failed...");
    }
}