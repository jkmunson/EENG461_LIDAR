#!/bin/bash
sudo stty -parenb -F /dev/ttyACM0 921600 cs8 -cstopb raw
sudo cat /dev/ttyACM0 | xxd -u
#sudo screen /dev/ttyACM0 921600
