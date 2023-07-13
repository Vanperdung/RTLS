#include "mqtt_user.h"
#include "common_user.h"
#include "fota.h"

static const char *TAG = "MQTT";
RingbufHandle_t mqtt_ring_buf;
esp_mqtt_client_handle_t client;
char mac_address[6] = {0};
char ip_address[16] = {0};
char ssid[32] = {0};
char topic_response[64] = {0};
char topic_request[64] = {0};
char topic_request_all[64] = {0};
char topic_location[64] = {0};
extern wireless_state_t wireless_state;
extern uint16_t ID;

void mqtt_create_topic(void)
{
    sprintf(topic_response, "gateway/%d/response", ID);
    sprintf(topic_request, "gateway/%d/request", ID);
    sprintf(topic_request_all, "gateway/all/request");
}

void mqtt_send_status(void)
{
    char status[512] = {0};
    sprintf(status, "{\"id\":%d,\"method\":\"extended_status\",\"values\":{\"firmwareVersion\":\"%s\",\"hardwareVersion\":\"%s\",\"ip\":\"%s\",\"mac\":\"" MACSTR "\",\"ssid\":\"%s\",\"resetReason\":\"reset\"}}",
            ID, FIRMWARE_VER, HARDWARE_VER, ip_address, MAC2STR(mac_address), ssid);
    esp_mqtt_client_publish(client, topic_response, status, strlen(status), 0, 0);
}

static int format_command(char *command, char *header, char *data, int size)
{
    int k = strlen(header);
    for (int i = 0; i < k; i++)
    {
        command[i] = header[i];
    }
    for (int j = k; j < k + size; j++)
    {
        command[j] = data[j - k];
    }
    command[k + size] = '*';
    return k + size + 1;
}

void mqtt_client_init(void)
{
    uint8_t broker[50] = {0};
    sprintf((char *)broker, "mqtt://%s", MQTT_BROKER);
    mqtt_ring_buf = xRingbufferCreate(4096, RINGBUF_TYPE_NOSPLIT);
    if (mqtt_ring_buf == NULL)
        ESP_LOGE(TAG, "Failed to create ring buffer");
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = (char *)broker,
        .keepalive = 60,
    };
    mqtt_create_topic();
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    xTaskCreate(&mqtt_user_task, "MQTT", 4096, NULL, 9, NULL);
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        ESP_LOGI(TAG, "MQTT event connected");
        mqtt_send_status();
        esp_mqtt_client_subscribe(client, topic_request, 0);
        esp_mqtt_client_subscribe(client, topic_request_all, 0);
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event subcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event unsubcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT event published, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
    {
        UBaseType_t res = xRingbufferSend(mqtt_ring_buf, event->data, event->data_len, portMAX_DELAY);
        if (res != pdTRUE)
            ESP_LOGE(TAG, "Failed to send item\n");
        break;
    }
    case MQTT_EVENT_ERROR:
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_user_task(void *param)
{
    mqtt_obj_t mqtt_obj;
    rtls_obj_t rtls_obj;
    char *msg_recv = NULL;
    size_t msg_size = 0;
    esp_err_t ret;
    char data_send[128] = {0};
    char start_cmd[] = "$start*";
    char stop_cmd[] = "$stop*";
    char get_info_cmd[] = "$get_info*";
    char reset_cmd[] = "$reset*";
    while (1)
    {
        msg_recv = (char *)xRingbufferReceive(mqtt_ring_buf, &msg_size, portMAX_DELAY);
        if (msg_recv)
        {
            msg_recv[msg_size] = '\0';
            ESP_LOGI(TAG, "Payload: %s", msg_recv);
            memset(&mqtt_obj, 0, sizeof(mqtt_obj));
            ret = mqtt_parse_data(msg_recv, &mqtt_obj);
            if (ret == ESP_OK)
            {
                memset(data_send, 0, sizeof(data_send));
                memset(&rtls_obj, 0, sizeof(rtls_obj));
                if (!strcmp((char *)mqtt_obj.method, "cfg_anchor"))
                {
                    rtls_obj.cfg_anchor.address[0] = (uint8_t)(mqtt_obj.address >> 8);
                    rtls_obj.cfg_anchor.address[1] = (uint8_t)(mqtt_obj.address & 0x00FF);
                    rtls_obj.cfg_anchor.slot = mqtt_obj.slot;
                    rtls_obj.cfg_anchor.x.xVal = mqtt_obj.x;
                    rtls_obj.cfg_anchor.y.yVal = mqtt_obj.y;
                    int len = format_command(data_send, "$cfg_anchor", (char *)&rtls_obj.cfg_anchor, sizeof(rtls_obj.cfg_anchor));
                    uart_write_bytes(UART_NUM_1, data_send, len);
                }
                else if (!strcmp((char *)mqtt_obj.method, "del_anchor"))
                {
                    rtls_obj.del_anchor.address[0] = (uint8_t)(mqtt_obj.address >> 8);
                    rtls_obj.del_anchor.address[1] = (uint8_t)(mqtt_obj.address & 0x00FF);
                    rtls_obj.del_anchor.slot = mqtt_obj.slot;
                    int len = format_command(data_send, "$del_anchor", (char *)&rtls_obj.del_anchor, sizeof(rtls_obj.del_anchor));
                    uart_write_bytes(UART_NUM_1, data_send, len);
                }
                else if (!strcmp((char *)mqtt_obj.method, "cfg_tag"))
                {
                    rtls_obj.cfg_tag.address[0] = (uint8_t)(mqtt_obj.address >> 8);
                    rtls_obj.cfg_tag.address[1] = (uint8_t)(mqtt_obj.address & 0x00FF);
                    rtls_obj.cfg_tag.slot = mqtt_obj.slot;
                    int len = format_command(data_send, "$cfg_tag", (char *)&rtls_obj.cfg_tag, sizeof(rtls_obj.cfg_tag));
                    uart_write_bytes(UART_NUM_1, data_send, len);
                }
                else if (!strcmp((char *)mqtt_obj.method, "del_tag"))
                {
                    rtls_obj.del_tag.address[0] = (uint8_t)(mqtt_obj.address >> 8);
                    rtls_obj.del_tag.address[1] = (uint8_t)(mqtt_obj.address & 0x00FF);
                    rtls_obj.del_tag.slot = mqtt_obj.slot;
                    int len = format_command(data_send, "$del_tag", (char *)&rtls_obj.del_tag, sizeof(rtls_obj.del_tag));
                    uart_write_bytes(UART_NUM_1, data_send, len);
                }
                else if (!strcmp((char *)mqtt_obj.method, "cfg_superframe"))
                {
                    rtls_obj.cfg_sf.address[0] = (uint8_t)(mqtt_obj.address >> 8);
                    rtls_obj.cfg_sf.address[1] = (uint8_t)(mqtt_obj.address & 0x00FF);
                    rtls_obj.cfg_sf.sf_num = mqtt_obj.sf_num;
                    int len = format_command(data_send, "$cfg_superframe", (char *)&rtls_obj.cfg_sf, sizeof(rtls_obj.cfg_sf));
                    uart_write_bytes(UART_NUM_1, data_send, len);
                }
                else if (!strcmp((char *)mqtt_obj.method, "start"))
                {
                    uart_write_bytes(UART_NUM_1, start_cmd, strlen(start_cmd));
                }
                else if (!strcmp((char *)mqtt_obj.method, "stop"))
                {
                    uart_write_bytes(UART_NUM_1, stop_cmd, strlen(stop_cmd));
                }
                else if (!strcmp((char *)mqtt_obj.method, "get_info"))
                {
                    uart_write_bytes(UART_NUM_1, get_info_cmd, strlen(get_info_cmd));
                }
                else if (!strcmp((char *)mqtt_obj.method, "reset"))
                {
                    uart_write_bytes(UART_NUM_1, reset_cmd, strlen(reset_cmd));
                    esp_restart();
                }
                else if (!strcmp((char *)mqtt_obj.method, "fota"))
                {
                    uart_write_bytes(UART_NUM_1, reset_cmd, strlen(reset_cmd));
                    xTaskCreate(&fota_task, "OTA", 8192, mqtt_obj.url, 20, NULL);
                }
            }
            vRingbufferReturnItem(mqtt_ring_buf, (void *)msg_recv);
        }
    }
}