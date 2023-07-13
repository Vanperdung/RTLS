idf_component_register(SRCS "fota.c" "common_user.c" "wifi_user.c" "mqtt_user.c" "main.c" "smartcfg.c" "uart_user.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES ${project_dir}/cert/git_ota.pem)
