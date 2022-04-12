#!/usr/bin/python3
# coding:utf-8 
import rospy
from doodbot.msg import Gamepad

from pyjoycon import JoyCon, get_R_id

import hid

for device_dict in hid.enumerate():
    keys = list(device_dict.keys())
    keys.sort()
    for key in keys:
        print("%s : %s" % (key, device_dict[key]))
    print()

devices = hid.enumerate()
print(devices)

print(get_R_id())
joycon_id = get_R_id()
joycon = JoyCon(*joycon_id)

print(joycon.get_status())
