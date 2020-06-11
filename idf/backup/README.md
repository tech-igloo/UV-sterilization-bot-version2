ESP-IDF WEB app
====================

This is a web application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Please check [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for getting started instructions.

Build Instructions:
Navigate to /idf/backup, and execute idf.py fullclean
Execute idf.py menuconfig
Navigate to http_server and change maximum length of both fields to 1024
Run idf.py build