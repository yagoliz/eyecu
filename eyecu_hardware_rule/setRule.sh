#!/bin/bash

sudo cp 20-eye-devices.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger
