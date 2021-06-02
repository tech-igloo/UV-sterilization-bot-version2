ESP-IDF WEB app
====================

This is a web application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Please check [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for getting started instructions.

Build Instructions:
Navigate to /idf/backup.
Run idf.py build.
In case it does not build, run idf.py fullclean
Make sure that the max_uri and max_header fields are set to 1024 in the http_server option in menuconfig.
Run idf.py menuconfig. Go to Component Config -> Wi-Fi -> Wifi Task Core ID to set the core on which the Wifi will run.