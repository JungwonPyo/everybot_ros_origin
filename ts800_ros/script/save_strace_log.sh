#!/bin/bash

echo "[save_strace_log.sh] start"
ps_name="ts800_app"
ppid=""


# 1. check process is exist
while [${ppid} -eq ""]
do
    echo "[save_strace_log.sh] wait "${ps_name}" ppid..."

    # get process's ppid
    ppid=$(ps -ef | grep ${ps_name} | grep -v grep | head -n 1 | awk '{print $2}')
    sleep 1
done
echo "[save_strace_log.sh] "
echo "[save_strace_log.sh] =========================="
echo "[save_strace_log.sh] "${ps_name}" ppid: "${ppid}
echo "[save_strace_log.sh] =========================="
echo "[save_strace_log.sh] "


# 2. run strace & save strace log
strace -ftp ${ppid} -o /home/ebot/strace_log_${ps_name}.txt
echo "[save_strace_log.sh] end"