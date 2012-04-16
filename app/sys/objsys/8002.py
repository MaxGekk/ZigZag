from sysobj_generator import *

profile =\
[
		{
		"num":0xE2,
		"name":"INPUT",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"r"
		},
		{
		"num":0xE3,
		"name":"OUTPUT",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"rw",
		"object_access":"rw",
		}
]

generate(profile, 0x8002)

