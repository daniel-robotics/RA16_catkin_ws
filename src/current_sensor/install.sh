#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PACKAGE_NAME="$(basename "$SCRIPT_DIR")"
cd "$SCRIPT_DIR"

# Install Adafruit Blinka
sudo pip install --upgrade adafruit-python-shell
wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
sudo python raspi-blinka.py
rm raspi-blinka.py

# Enable Raspberry Pi I2C Clock Stretching

# Install Pip dependencies
cd "$SCRIPT_DIR"
if [ -f "./requirements.txt" ]; then
    pip install -r requirements.txt
fi
