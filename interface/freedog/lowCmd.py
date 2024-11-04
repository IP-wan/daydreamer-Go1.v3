import crcmod
import struct
import binascii
from enum import Enum
from interface.freedog.enums import MotorModeLow, GaitType, SpeedLevel
from interface.freedog.common import float_to_hex, encryptCrc, genCrc
from interface.freedog.complex import bmsCmd, motorCmd, motorCmdArray


class lowCmd:
    def __init__(self):
        self.head = bytes.fromhex('FEEF')
        self.levelFlag = 0xff
        self.frameReserve = 0
        self.SN = bytearray(8)
        self.version = bytearray(8)
        self.bandWidth = bytes.fromhex('3ac0')
        self.motorCmd = motorCmdArray()
        self.bms = bmsCmd(0,[0,0,0])
        self.wirelessRemote = bytearray(40)
        self.reserve = bytearray(4)
        self.crc = None
        self.encrypt = True

    def buildCmd(self, debug=False):
        cmd = bytearray(614)
        cmd[0:2] = self.head
        cmd[2] = self.levelFlag
        cmd[3] = self.frameReserve

        cmd[4:12] = self.SN
        cmd[12:20] = self.version
        cmd[20:22] = self.bandWidth

        cmd[22:562] = self.motorCmd.getBytes()
        cmd[562:566] = self.bms.getBytes()
        cmd[566:606] = self.wirelessRemote

        if self.encrypt:
            cmd[-4:] = encryptCrc(genCrc(cmd[:-6]))
        else:
            cmd[-4:] = genCrc(cmd[:-6])

        if debug:
            print(f'Length: {len(cmd)}')
            print('Data: '+ ''.join('{:02x}'.format(x) for x in cmd))

        return cmd

    def low_cmd_from_bytes(data):
        lcmd = lowCmd()
        lcmd.head = data[0:2]
        lcmd.levelFlag = data[2]
        lcmd.frameReserve = data[3]

        lcmd.SN = data[4:12]
        lcmd.version = data[12:20]
        lcmd.bandWidth = data[20:22]

        lcmd.motorCmd = motorCmdArray().fromBytes(data[22:562])
        lcmd.bms = bmsCmd().fromBytes(data[562:566])
        lcmd.wirelessRemote = data[566:606]
        # lcmd.reserve = int.from_bytes(data[606:610], byteorder='little')

        # lcmd.crc = int.from_bytes(data[610:614], byteorder='little')

        lcmd.encrypt = None
        if data[-4:] == encryptCrc(genCrc(data[:-6])):
            lcmd.encrypt = True

        if data[-4:] == genCrc(data[:-6]):
            lcmd.encrypt = False
        return data