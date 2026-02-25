@echo off
set IDF_PATH=E:\esp\Espressif\frameworks\esp-idf-v5.5.2
set IDF_TOOLS_PATH=E:\esp\Espressif
call "%IDF_PATH%\export.bat"
cd /d E:\IoT2\ESP_Firmware
idf.py set-target esp32s3
