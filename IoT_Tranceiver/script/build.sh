#!/bin/bash

#enable the venv
source ~/zephyrproject/.venv/bin/activate

#enable the zephyr env
source ~/zephyrproject/zephyr/zephyr-env.sh

# west build -p always -b esp_wrover_kit/esp32/procpu

west build -p always -b esp_wrover_kit/esp32/procpu -- -DEXTRA_DTC_OVERLAY_FILE=modem_overlay.overlay