#!/bin/bash
SCRIPT_DIR=$(dirname "$0")
sudo cp "$SCRIPT_DIR/10-tas.rules" /etc/udev/rules.d &&
sudo udevadm control --reload &&
sudo service udev reload &&
sudo service udev restart

## if not work, reboot nuc
