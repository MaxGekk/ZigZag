from sysobj_generator import *

profile =\
[
		{
		"num":0x24, 
		"name":"BATTERY_VOLTAGE",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
		},
        ]

generate(profile, 0x8000)
