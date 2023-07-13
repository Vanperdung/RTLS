#include "common_user.h"
#include "cJSON.h"

extern uint16_t ID;

esp_err_t mqtt_parse_data(char *mqtt_data, mqtt_obj_t *mqtt_obj)
{
    cJSON *root = cJSON_Parse(mqtt_data);
    if (root == NULL)
        return ESP_FAIL;
    cJSON *elem = NULL;
    uint16_t id = cJSON_GetObjectItem(root, "id")->valueint;
    if (!(id == ID || id == 0xFFFF))
        return ESP_FAIL;
    cJSON_ArrayForEach(elem, root)
    {
        if (elem->string)
        {
            char *elem_str = elem->string;
            if (!strcmp(elem_str, "id"))
            {
                mqtt_obj->id = elem->valueint;
            }
            else if (!strcmp(elem_str, "method"))
            {
                strcpy((char *)mqtt_obj->method, elem->valuestring);
            }
            else if (!strcmp(elem_str, "address"))
            {
                mqtt_obj->address = elem->valueint;
            }
            else if (!strcmp(elem_str, "slot"))
            {
                mqtt_obj->slot = elem->valueint;
            }
            else if (!strcmp(elem_str, "x"))
            {
                mqtt_obj->x = elem->valuedouble;
            }
            else if (!strcmp(elem_str, "y"))
            {
                mqtt_obj->y = elem->valuedouble;
            }
            else if (!strcmp(elem_str, "sf_num"))
            {
                mqtt_obj->sf_num = elem->valueint;
            }
            else if (!strcmp(elem_str, "url"))
            {
                strcpy((char *)mqtt_obj->url, elem->valuestring);
            }
        }
    }
    return ESP_OK;
}