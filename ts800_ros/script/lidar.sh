#!/bin/bash

# 0. lidar control filemode change
chown ebot:ebot /sys/class/pwm/pwmchip1/pwm0/duty_cycle
chmod u+x /sys/class/pwm/pwmchip1/pwm0/duty_cycle