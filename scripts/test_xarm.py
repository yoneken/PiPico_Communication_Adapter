#!/usr/bin/env python3
import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from configparser import ConfigParser
#parser = ConfigParser()
#parser.read('./robot.conf', encoding='utf-8')
#try:
#    ip = parser.get('xArm', 'ip')
#except:
#    ip = input('Please input the xArm ip address[192.168.0.244]:')
#    if not ip:
#        ip = '192.168.0.244'

ip = '192.168.0.244'

arm = XArmAPI(ip)
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)

# code = arm.set_tgpio_modbus_timeout(20)
# print('set_tgpio_modbus_timeout, code={}'.format(code))

# set tool gpio modbus baudrate
#code = arm.set_tgpio_modbus_baudrate(2000000)
#code = arm.set_tgpio_modbus_baudrate(921600)
code = arm.set_tgpio_modbus_baudrate(460800)
#code = arm.set_tgpio_modbus_baudrate(230400)
print('set_tgpio_modbus_baudrate, code={}'.format(code))
time.sleep(2)

# enable gripper
code, ret = arm.getset_tgpio_modbus_data([0x08, 0x06, 0x01, 0x00, 0x00, 0x01])
print('set_gripper_enable, code={}, ret={}'.format(code, ret))

time.sleep(3)

# calibrate gripper
code, ret = arm.getset_tgpio_modbus_data([0x08, 0x06, 0x01, 0x10, 0x00, 0x01])
print('set_gripper_calibrate, code={}, ret={}'.format(code, ret))

time.sleep(8)

while arm.connected and arm.error_code == 0:
    code, ret = arm.getset_tgpio_modbus_data([0x08, 0x06, 0x07, 0x00, 0x0, 0xd0])
    print('open_bio_gripper, code={}, ret={}'.format(code, ret))

    time.sleep(4)

    code, ret = arm.getset_tgpio_modbus_data([0x08, 0x06, 0x07, 0x00, 0x0, 0x20])
    print('close_bio_gripper, code={}, ret={}'.format(code, ret))
    time.sleep(4)