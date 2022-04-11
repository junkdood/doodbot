#!/usr/bin/python3
# coding:utf-8 
import rospy
from doodbot.msg import Gamepad

from pyjoycon import JoyCon, get_R_id

joycon_id = get_R_id()
joycon = JoyCon(*joycon_id)

print(joycon.get_status())