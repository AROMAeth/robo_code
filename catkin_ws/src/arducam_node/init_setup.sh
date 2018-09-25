#!/bin/bash

sudo cp `rospack find arducam_node`/scripts/arducam.rules  /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
chmod +x src/arducam.py
chmod +x src/arducam_long.py
chmod +x src/arducam_short.py