from sysobj_generator import *

profile =\
[
		{
		"num":0xE2,
		"name":"VIBRO_STATE",
		"size":8,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
        "flash":True,
        "default":"0",
		},
		{
		"num":0xE3,
		"name":"VIBRO_THRESHOLD_HIGH",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
        "flash":True,
        "default":"30000",
		},
		{
		"num":0xE4,
		"name":"VIBRO_THRESHOLD_LOW",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
        "flash":True,
		"default":"8718",
		},
		{
		"num":0xE5,
		"name":"VIBRO_STATE_CHANGE_NUM",
		"size":8,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
        "flash":True,
		"default":"1",
		},
]

generate(profile, 0x8007)

