ESP-IDF WEB app
====================

This is a web application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Please check [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for getting started instructions.

Build Instructions:

* Run idf.py menuconfig. Go to Component Config -> Wi-Fi -> Wifi Task Core ID to set to core 0 on which the Wifi will run.
* Make sure that the max_uri and max_header fields are set to 1024 in the http_server option in menuconfig.
* Go to Partition table set custom partition CSV file as - partitions.csv (Included in the present directory).
* Set the offset of partition table as - 0x8000.
* Navigate to /ESP32-main-code, Run idf.py build.
* In case it does not build, run idf.py fullclean and build again.
