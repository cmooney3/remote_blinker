#!/bin/bash

BOARD="arduino:avr:leonardo"
PORT="$(ls /dev/ttyACM* | head -n1)"
ACTION="--upload"
ARDUINO="arduino"
INO_LOCATION="./receiver.ino"

JAVA_TOOL_OPTIONS='-Djava.awt.headless=true' "${ARDUINO}" "${ACTION}" "${INO_LOCATION}" --board "${BOARD}" --port "${PORT}"
