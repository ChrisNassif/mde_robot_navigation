#!/bin/bash

sudo tee "/etc/udev/rules.d/97-robot-navigation.rules" > /dev/null << 'EOF'
ACTION=="add", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", SYMLINK+="pico", MODE="0666"
EOF

# Reload udev
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger
