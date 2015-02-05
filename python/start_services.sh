#!/bin/bash
source ~/.virtualenvs/urpobotti/bin/activate
./motorctrl.py /dev/ttyO3 &
./pinger.py /dev/ttyUSB0 &

