#!/bin/bash
sudo stty -parenb -F /dev/ttyACM0 4000000 cs8 -cstopb raw
sudo cat /dev/ttyACM0 | xxd -u
#sudo screen /dev/ttyACM0 921600
