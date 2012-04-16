#!/bin/bash
mspjtag.py -r
if [ "$?" -ne "0" ]; then
  echo "Sorry, cannot reset device"
  exit 1
fi

for (( addr = 35328 ; addr <= 65024 ; addr+=512 )) ; do
    args=${args}" --erase=${addr}"
done

mspjtag.py ${args} && mspjtag.py -s 512 -R 2048 -pv $1

