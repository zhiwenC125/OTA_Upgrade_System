idf_component_register(SRCS "hello_world_main.c"
                            "wifi_station.c"
                            "mqtt_client_task.c"
                            "ca_cert.c"
                       PRIV_REQUIRES driver esp_wifi esp_netif esp_event nvs_flash mqtt
                       INCLUDE_DIRS "")
