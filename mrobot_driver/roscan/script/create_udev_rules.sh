#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo ""

sudo cp can_transceiver.rules /etc/udev/rules.d

echo ""
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart
