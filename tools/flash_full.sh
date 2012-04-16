#!/bin/bash
mspjtag.py -r -m --eraseinfo && mspjtag.py -s 512 -R 2048 -p $1

