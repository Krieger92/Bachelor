#!/bin/bash

#enable the venv
source ~/zephyrproject/.venv/bin/activate

#enable the zephyr env
source ~/zephyrproject/zephyr/zephyr-env.sh

west build -p always -b esp_wrover_kit
