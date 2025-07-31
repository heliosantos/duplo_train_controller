An alternative to the Duplo mobile app for controlling Duplo trains.
The controller is powered by a ESP32 C3 microcontroller.

<img width="2978" height="2071" alt="controller" src="https://github.com/user-attachments/assets/c10b37eb-4a15-4608-b4ec-3ee505c8f1ad" />

```shell
# Requirements:
# - esp-idf

# enable on shell
. /opt/esp-idf/export.sh

# set configurations
idf.py menuconfig

# create project
idf.py create-project blink
cd blink

# set target chip
idf.py set-target esp32c3

# build
idf.py build

# flash
idf.py -p /dev/ttyACM0 flash monitor
```

