#!/bin/bash

BOARD="arduino:avr:uno"
PORT="$(ls /dev/ttyUSB* | head -n1)"
ACTION="--upload"
ARDUINO="arduino"
INO_LOCATION="./transmitter.ino"

JAVA_TOOL_OPTIONS='-Djava.awt.headless=true' "${ARDUINO}" "${ACTION}" "${INO_LOCATION}" --board "${BOARD}" --port "${PORT}"
