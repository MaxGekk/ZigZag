from sysobj_generator import *

profile =\
[
		{
		"num":0xE2,
		"name":"INPUT",
		"size":32,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw"
		},
]

generate(profile, 0x8005)

