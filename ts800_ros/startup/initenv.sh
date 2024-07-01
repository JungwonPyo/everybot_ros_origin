#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", ENV{ID_USB_INTERFACE_NUM}=="00", MODE:="0666", GROUP:="dialout",  SYMLINK+="t5"' >/etc/udev/rules.d/t5.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ENV{ID_MODEL_ID}=="6001", MODE:="0666", GROUP:="dialout",  SYMLINK+="t5"' >/etc/udev/rules.d/t5.rules

service udev reload
sleep 2
service udev restart
