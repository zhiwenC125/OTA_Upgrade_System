@echo off
set IDF_PATH=E:\esp\v5.5.2\esp-idf
set PATH=E:\esp\Espressif\python_env\idf5.5_py3.11_env\Scripts;E:\esp\Espressif\tools\idf-git\2.44.0\cmd;%PATH%
cd /d E:\IoT2\ESP_Firmware
python E:\esp\v5.5.2\esp-idf\tools\idf.py build
