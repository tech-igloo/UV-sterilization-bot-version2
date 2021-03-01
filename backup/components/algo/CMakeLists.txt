set(component_srcs "src/algo.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES "nvs_flash" "esp_http_server" "spiffs" "mdns"
                       REQUIRES server)