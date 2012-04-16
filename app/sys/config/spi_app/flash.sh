#!/bin/bash
mspjtag.py -r
if [ "$?" -ne "0" ]; then
  echo "Sorry, cannot reset device"
  exit 1
fi

for (( addr = 0x4000 ; addr < 0xdc00 ; addr+=0x200 )) ; do # [0x4000 ; 0x6600 ]
    args=${args}" --erase=${addr}"
done
# erase module infomem as well
args=${args}" --erase=0x1080"

mspjtag.py ${args} && mspjtag.py -s 512 -R 2048 -pv $1

