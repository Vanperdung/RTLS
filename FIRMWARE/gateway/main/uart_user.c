#include "uart_user.h"
#include "common_user.h"

static const char *TAG = "UART";
extern esp_mqtt_client_handle_t client;
extern char topic_response[64];
extern char topic_location[64];
extern wireless_state_t wireless_state;
extern uint16_t ID;

static int format_id_command(char *command, char *header)
{
    int k = strlen(header);
    for (int i = 0; i < k; i++)
    {
        command[i] = header[i];
    }
    command[k] = (uint8_t)(ID >> 8);
    command[k + 1] = (uint8_t)(ID & 0x00FF);
    command[k + 2] = '*';
    return k + 2 + 1;
}

void uart_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, 256 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_cfg);
    uart_set_pin(UART_NUM_1, UART1_TXD_PIN, UART1_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void uart_user_task(void *param)
{
    rtls_obj_t rtls_obj;
    int msg_size = 0;
    char msg_recv[256] = {0};
    char data_send[256] = {0};
    uint16_t address;
    uart_init();

    int len = format_id_command(data_send, "$id");
    uart_write_bytes(UART_NUM_1, data_send, len);

    while (1)
    {
        msg_size = uart_read_bytes(UART_NUM_1, msg_recv, sizeof(msg_recv), 10 / portTICK_RATE_MS);
        if (msg_size)
        {
            msg_recv[msg_size] = '\0';
            if (strstr(msg_recv, "cfg_anchor") != NULL)
            {
                memcpy((uint8_t *)&rtls_obj.cfg_anchor, &msg_recv[11], sizeof(rtls_obj.cfg_anchor));
                address = (uint16_t)(rtls_obj.cfg_anchor.address[0] << 8 | rtls_obj.cfg_anchor.address[1]);
                sprintf(data_send, "{\"id\":%d,\"method\":\"cfg_anchor\",\"address\":%d,\"slot\":%d,\"x\":%0.2f,\"y\":%0.2f}", ID, address, rtls_obj.cfg_anchor.slot, rtls_obj.cfg_anchor.x.xVal, rtls_obj.cfg_anchor.y.yVal);
                if (wireless_state == WIFI_CONNECTED)
                    esp_mqtt_client_publish(client, topic_response, data_send, strlen(data_send), 0, 0);
            }
            else if (strstr(msg_recv, "del_anchor") != NULL)
            {
                memcpy((uint8_t *)&rtls_obj.del_anchor, &msg_recv[11], sizeof(rtls_obj.del_anchor));
                address = (uint16_t)(rtls_obj.del_anchor.address[0] << 8 | rtls_obj.del_anchor.address[1]);
                sprintf(data_send, "{\"id\":%d,\"method\":\"del_anchor\",\"address\":%d,\"slot\":%d}", ID, address, rtls_obj.del_anchor.slot);
                if (wireless_state == WIFI_CONNECTED)
                    esp_mqtt_client_publish(client, topic_response, data_send, strlen(data_send), 0, 0);
            }
            else if (strstr(msg_recv, "cfg_superframe") != NULL)
            {
                memcpy((uint8_t *)&rtls_obj.cfg_sf, &msg_recv[15], sizeof(rtls_obj.cfg_sf));
                address = (uint16_t)(rtls_obj.cfg_sf.address[0] << 8 | rtls_obj.cfg_sf.address[1]);
                sprintf(data_send, "{\"id\":%d,\"method\":\"cfg_superframe\",\"address\":%d,\"sf_num\":%d}", ID, address, rtls_obj.cfg_sf.sf_num);
                if (wireless_state == WIFI_CONNECTED)
                    esp_mqtt_client_publish(client, topic_response, data_send, strlen(data_send), 0, 0);
            }
            else if (strstr(msg_recv, "start") != NULL)
            {
                sprintf(data_send, "{\"id\":%d,\"method\":\"start\",\"address\":%d}", ID, ID);
                if (wireless_state == WIFI_CONNECTED)
                    esp_mqtt_client_publish(client, topic_response, data_send, strlen(data_send), 0, 0);
            }
            else if (strstr(msg_recv, "get_info") != NULL)
            {
                memcpy((uint8_t *)&rtls_obj.info, &msg_recv[9], sizeof(rtls_obj.info));
                address = (uint16_t)(rtls_obj.info.anchorInfo.address[0] << 8 | rtls_obj.info.anchorInfo.address[1]);
                sprintf(data_send, "{\"id\":%d,\"method\":\"get_info\",\"bcn\":%d,\"twr\":%d,\"sf_num\":%d,\"address\":%d,\"slot\":%d,\"x\":%0.2f,\"y\":%0.2f", ID,
                        rtls_obj.info.bcnSlotTotal,
                        rtls_obj.info.twrSlotTotal,
                        rtls_obj.info.sfNumberTotal,
                        address,
                        rtls_obj.info.anchorInfo.slot,
                        rtls_obj.info.anchorInfo.x.xVal,
                        rtls_obj.info.anchorInfo.y.yVal);
                if (wireless_state == WIFI_CONNECTED)
                    esp_mqtt_client_publish(client, topic_response, data_send, strlen(data_send), 0, 0);
            }
            else if (strstr(msg_recv, "location") != NULL)
            {
                int j;
                for (j = msg_size; j >= 0; j--)
                {
                    if (msg_recv[j] == '*')
                        break;
                }
                int cnt = msg_recv[j - 1];
                if (cnt > 0 && cnt < 15)
                {
                    tagLoc_t *tagLoc = (tagLoc_t *)calloc(cnt, sizeof(tagLoc_t));
                    memcpy((uint8_t *)tagLoc, &msg_recv[9], sizeof(tagLoc_t) * cnt);
                    for (int k = 0; k < cnt; k++)
                    {
                        address = (uint16_t)(tagLoc[k].tagAddress[0] << 8 | tagLoc[k].tagAddress[1]);
                        sprintf(data_send, "{\"id\":%d,\"method\":\"location\",\"address\":%d,\"slot\":%d,\"x\":%0.2f,\"y\":%0.2f}", ID,
                                address,
                                tagLoc[k].twrSlotNumber,
                                tagLoc[k].x.xVal,
                                tagLoc[k].y.yVal);
                        sprintf(topic_location, TOPIC_LOCATION, address);
                        if (WIFI_CONNECTED)
                            esp_mqtt_client_publish(client, topic_location, data_send, strlen(data_send), 0, 0);
                    }
                    free(tagLoc);
                }
            }
            else if (strstr(msg_recv, "cfg_tag") != NULL)
            {
                memcpy((uint8_t *)&rtls_obj.cfg_tag, &msg_recv[8], sizeof(rtls_obj.cfg_tag));
                address = (uint16_t)(rtls_obj.cfg_tag.address[0] << 8 | rtls_obj.cfg_tag.address[1]);
                sprintf(data_send, "{\"id\":%d,\"method\":\"cfg_tag\",\"address\":%d,\"slot\":%d}", ID, address, rtls_obj.cfg_tag.slot);
                if (wireless_state == WIFI_CONNECTED)
                    esp_mqtt_client_publish(client, topic_response, data_send, strlen(data_send), 0, 0);
            }
            else if (strstr(msg_recv, "del_tag") != NULL)
            {
                memcpy((uint8_t *)&rtls_obj.del_tag, &msg_recv[8], sizeof(rtls_obj.del_tag));
                address = (uint16_t)(rtls_obj.del_tag.address[0] << 8 | rtls_obj.del_tag.address[1]);
                sprintf(data_send, "{\"id\":%d,\"method\":\"del_tag\",\"address\":%d,\"slot\":%d}", ID, address, rtls_obj.del_tag.slot);
                if (wireless_state == WIFI_CONNECTED)
                    esp_mqtt_client_publish(client, topic_response, data_send, strlen(data_send), 0, 0);
            }
            else if (strstr(msg_recv, "distance") != NULL)
            {
                ESP_LOGI(TAG, "%s", msg_recv);
                int txCnt, rxCnt;
                double distance, fpath, rssi, prNlos, mc, cl;
                char *ptr;
                int i = 0;
                char *p[9];
                char *token = strtok(msg_recv, ",");
                while (token != NULL)
                {
                    token = strtok(NULL, ",");
                    p[i++] = token;
                }
                txCnt = atoi(p[0]);
                rxCnt = atoi(p[1]);
                distance = strtod(p[2], &ptr);
                rssi = strtod(p[3], &ptr);
                fpath = strtod(p[4], &ptr);
                prNlos = strtod(p[5], &ptr);
                mc = strtod(p[6], &ptr);
                cl = strtod(p[7], &ptr);
                sprintf(data_send, "{\"id\":%d,\"method\":\"distance\",\"txCnt\":%d,\"rxCnt\":%d,\"value\":%.2f,\"rssi\":%.2f,\"fpath\":%.2f,\"prNlos\":%.2f,\"mc\":%.2f,\"cl\":%.2f}", ID, txCnt, rxCnt, distance, rssi, fpath, prNlos, mc, cl);
                if (wireless_state == WIFI_CONNECTED)
                    esp_mqtt_client_publish(client, topic_response, data_send, strlen(data_send), 0, 0);
            }
        }
    }
}