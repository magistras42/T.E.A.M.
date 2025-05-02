#!/bin/bash

echo "\n" > speed_script.log

while true; do
    # cat /sys/module/speed_driver/parameters/elapsedTime >> speed_script.log
    cat /sys/module/speed_driver/parameters/elapsedTime 
    sleep 0.01
done

