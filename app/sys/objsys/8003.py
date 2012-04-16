from sysobj_generator import *

profile =\
[
		{
		"num":0x24, 
		"name":"ANALOG_INPUT1",
		"size":16,
		"range":whole_range,
		"thresholds":( {"name":"THRESHOLD1", "default":(1000, 2000), "active":(True, True) }, ), 
		"network_acces":"r",
		"object_access":"rw",
		},
		{
		"num":0x4a, 
		"name":"ANALOG_INPUT2",
		"size":16,
		"range":whole_range,
		"thresholds":({"name":"THRESHOLD1", "default":(1000, 2000), "active":(True, True) }, ), 
		"network_acces":"r",
		"object_access":"rw",
		},
		{
		"num":0x70, 
		"name":"ANALOG_INPUT3",
		"size":16,
		"range":whole_range,
		"thresholds":({"name":"THRESHOLD1", "default":(1000, 2000), "active":(True, True) }, ), 
		"network_acces":"r",
		"object_access":"rw",
		},
		{
		"num":0x96, 
		"name":"ANALOG_INPUT4",
		"size":16,
		"range":whole_range,
		"thresholds":({"name":"THRESHOLD1", "default":(1000, 2000), "active":(True, True) }, ), 
		"network_acces":"r",
		"object_access":"rw",
		},
		{
		"num":0xE2,
		"name":"ANALOG_INPUT1_TEST",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
        "default": 0,
		},
        {
		"num":0xE3,
		"name":"ANALOG_INPUT2_TEST",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
        "default": 0,
		},
        {
		"num":0xE4,
		"name":"ANALOG_INPUT3_TEST",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
        "default": 0,
		},
        {
		"num":0xE5,
		"name":"ANALOG_INPUT4_TEST",
		"size":16,
		"range":whole_range,
		"thresholds":None,
		"network_acces":"r",
		"object_access":"rw",
        "default": 0,
		},
]

generate(profile, 0x8003)

