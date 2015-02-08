#!/bin/bash
echo "This is for tesing only, remember to stop the services managed by upstart before running this script"
source ~/.virtualenvs/urpobotti/bin/activate
./motorctrl.py /dev/ttyO3 &
./pinger.py /dev/ttyUSB0 &

