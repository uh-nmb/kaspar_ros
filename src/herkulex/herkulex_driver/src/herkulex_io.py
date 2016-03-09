"""
HerkuleX
A Python Processing library for Dongbu HerkuleX Servo adapted from:
https://github.com/dongburobot/HerkuleXProcessing/

This library is free software you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General
Public License along with this library if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330,
Boston, MA  02111-1307  USA

@author: Nathan Burke (natbur@natbur.com)
@modified    2015.02.28
@version     0.2
"""

from time import time, sleep
import yaml

import logging
import serial

from threading import RLock

__all__ = ['HerkuleX', 'LedColor', 'Error']


BROADCAST_ID = 0xFE
BASIC_PKT_SIZE = 7
WAIT_TIME_BY_ACK = 30


class Command(object):
    # SERVO HERKULEX COMMAND - See Manual p40
    HEEPWRITE = 0x01  # Rom write
    HEEPREAD = 0x02  # Rom read
    HRAMWRITE = 0x03  # Ram write
    HRAMREAD = 0x04  # Ram read
    HIJOG = 0x05  # Write n servo with different timing
    HSJOG = 0x06  # Write n servo with same time
    HSTAT = 0x07  # Read error
    HROLLBACK = 0x08  # Back to factory value
    HREBOOT = 0x09  # Reboot


class LedColor(object):
    # HERKULEX LED - See Manual p29
    LED_RED = 0x10
    LED_GREEN = 0x04
    LED_BLUE = 0x08
    LED_OFF = 0x00


class Error(object):
    # HERKULEX STATUS ERROR - See Manual p39
    H_STATUS_OK = 0x00
    H_ERROR_INPUT_VOLTAGE = 0x01
    H_ERROR_POS_LIMIT = 0x02
    H_ERROR_TEMPERATURE_LIMIT = 0x04
    H_ERROR_INVALID_PKT = 0x08
    H_ERROR_OVERLOAD = 0x10
    H_ERROR_DRIVER_FAULT = 0x20
    H_ERROR_EEPREG_DISTORT = 0x40

    H_DETAIL_MOVING = 0x01
    H_DETAIL_INPOSITION = 0x02
    H_PKTERR_CHECKSUM = 0x04
    H_PKTERR_UNKNOWN_CMD = 0x08
    H_PKTERR_EXCEED_REG_RANGE = 0x10
    H_PKTERR_GARBAGE = 0x20
    H_DETAIL_MOTORON = 0x04

with open('herkulex.t_adc.yaml') as adc:
    T_ADC = yaml.load(adc)


class HerkuleX(object):

    def __init__(self, port, baudrate):
        self._logger = logging.getLogger(self.__class__.__name__)
        self._portLock = RLock()
        self._port = serial.Serial(port=port,
                                   baudrate=baudrate,
                                   timeout=0.015)
        self._portName = port
        self.setAckPolicy(1)  # set ACK policy
        self._multipleMoveData = []

    def __del__(self):
        self.close()

    def close(self):
        if self._port:
            self._port.close()

    def initialize(self, servoID=BROADCAST_ID):
        """
        Initialize servos by clearing any errors and enabling torque

        Keyword arguments:
        @param: servoID -- the id of a single servo to initialize (default broadcast)
        """
        if servoID < 0 or servoID > 0xFF:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 255" % (servoID,))

        self.clearError(servoID)
        sleep(WAIT_TIME_BY_ACK / 1000)
        self.torqueOn(servoID)
        sleep(WAIT_TIME_BY_ACK / 1000)

    def performIDScan(self):
        """
        Scan for all connected servos

        @return: list of servoIds
        """
        matchedIDs = []

        for i in range(0, 254):
            if self.getPosition(i) != -1:
                matchedIDs.append(i)

        return matchedIDs

    def setAckPolicy(self, valueAck):
        """
        Set Ack Policy for all servos

        @param: valueACK 0=No Reply, 1=Only reply to READ CMD, 2=Always reply
        """
        if valueAck not in [0, 1, 2]:
            return

        optionalData = [0] * 3
        optionalData[0] = 0x01  # Address
        optionalData[1] = 0x01  # Length
        optionalData[2] = valueAck

        packet = self._buildPacket(
            0xFE, Command.HRAMWRITE, optionalData)
        self._sendData(packet)

    def clearError(self, servoID=BROADCAST_ID):
        """
        Error servo clear

        @param: servoID 0 ~ 254 (0x00 ~ 0xFE), 0xFE : BROADCAST_ID
        """
        if servoID < 0 or servoID > 0xFE:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 254" % (servoID,))

        optionalData = [0] * 4
        optionalData[0] = 0x30  # Address
        optionalData[1] = 0x02  # Length
        optionalData[2] = 0x00  # Write error=0
        optionalData[3] = 0x00  # Write detail error=0

        packet = self._buildPacket(
            servoID, Command.HRAMWRITE, optionalData)
        self._sendData(packet)

    def getVoltage(self, servoID):
        """
        Get servo voltage

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @return:current voltage 0 ~ 18.9 (-1: failure)
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        optionalData = [0] * 2
        optionalData[0] = 0x36  # Address
        optionalData[1] = 0x01  # Length

        packet = self._buildPacket(
            servoID, Command.HRAMREAD, optionalData)
        readBuffer = self._sendDataForResult(packet)

        if not self._isRightPacket(readBuffer):
            return -1

        if len(readBuffer) < 11:
            self._logger.error(
                "Strange Packet, expected len=11: %s", [str(x) for x in readBuffer])
            return -1
        adc = ((readBuffer[10] & 0x03) << 8) | (readBuffer[9] & 0xFF)
        return round(adc * 0.074, 2)  # return ADC converted back to voltage

    def getTemperature(self, servoID):
        """
        Get servo voltage

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @return: current temperature in C -80 ~ 310 (-1: failure)
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        optionalData = [0] * 2
        optionalData[0] = 0x37  # Address
        optionalData[1] = 0x01  # Length

        packet = self._buildPacket(
            servoID, Command.HRAMREAD, optionalData)
        readBuffer = self._sendDataForResult(packet)

        if not self._isRightPacket(readBuffer):
            return -1

        if len(readBuffer) < 11:
            self._logger.error(
                "Strange Packet, expected len=11: %s", [str(x) for x in readBuffer])
            return -1
        adc = ((readBuffer[10] & 0x03) << 8) | (readBuffer[9] & 0xFF)
        return T_ADC.get(adc, -1)

    def getTorque(self, servoID):
        """
        Get servo torque

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @return: current torque 0 ~ 1023 (-1: failure)
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        optionalData = [0] * 2
        optionalData[0] = 0x40  # Address
        optionalData[1] = 0x01  # Length

        packet = self._buildPacket(
            servoID, Command.HRAMREAD, optionalData)
        readBuffer = self._sendDataForResult(packet)

        if not self._isRightPacket(readBuffer):
            return -1

        if len(readBuffer) < 11:
            self._logger.error(
                "Strange Packet, expected len=11: %s", [str(x) for x in readBuffer])
            return -1

        return ((readBuffer[10] & 0x03) << 8) | (readBuffer[9] & 0xFF)

    def torqueOn(self, servoID):
        """
        Torque ON

        @param: servoID 0 ~ 254 (0x00 ~ 0xFE), 0xFE : BROADCAST_ID
        """
        if servoID < 0 or servoID > 0xFE:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 254" % (servoID,))

        optionalData = [0] * 3
        optionalData[0] = 0x34  # Address
        optionalData[1] = 0x01  # Length
        optionalData[2] = 0x60  # 0x60=Torque ON

        packet = self._buildPacket(
            servoID, Command.HRAMWRITE, optionalData)
        self._sendData(packet)

    def torqueOff(self, servoID):
        """
        Torque OFF

        @param: servoID 0 ~ 254 (0x00 ~ 0xFE), 0xFE : BROADCAST_ID
        """
        if servoID < 0 or servoID > 0xFE:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 254" % (servoID,))

        optionalData = [0] * 3
        optionalData[0] = 0x34  # Address
        optionalData[1] = 0x01  # Length
        optionalData[2] = 0x00  # 0x60=Torque ON

        packet = self._buildPacket(
            servoID, Command.HRAMWRITE, optionalData)
        self._sendData(packet)

    def moveSpeedOne(self, servoID, goalSpeed, playTime, ledColor=LedColor.LED_OFF):
        """
        Move one servo with continous rotation

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: goalSpeed -1023 ~ 1023 [CW:Negative Value(-), CCW:Positive Value(+)]
        @param: playTime 0 ~ 2856ms
        @param: ledColor LedColor.LED_RED | LedColor.LED_GREEN | LedColor.LED_BLUE
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))
        if goalSpeed > 1023 or goalSpeed < -1023:
            raise ValueError(
                "Invalid goalSpeed. Got: %s Expected -1023 ~ 1023" % (goalSpeed,))
        if playTime < 0 or playTime > 2856:
            raise ValueError(
                "Invalid playTime. Got: %s Expected 0 ~ 2856" % (goalSpeed,))

        if goalSpeed < 0:
            goal_speedSign = (-1) * goalSpeed
            goal_speedSign |= 0x4000
        else:
            goal_speedSign = goalSpeed

        goal_speed_LSB = goal_speedSign & 0X00FF
        goal_speed_MSB = (goal_speedSign & 0xFF00) >> 8

        playTime = playTime / 11.2  # ms --> value
        ledColor = ledColor | 0x02  # Speed Ctrl Mode

        optionalData = [0] * 5
        optionalData[0] = playTime  # Execution time
        optionalData[1] = goal_speed_LSB
        optionalData[2] = goal_speed_MSB
        optionalData[3] = ledColor
        optionalData[4] = servoID

        packet = self._buildPacket(servoID, Command.HSJOG, optionalData)
        self._sendData(packet)

    def getSpeed(self, servoID):
        """
        Get current servo speed (-1023 ~ 1023)

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @return: current speed -1023 ~ 1023 [CW:Negative Value(-), CCW:Positive Value(+)]
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        optionalData = [0] * 2
        optionalData[0] = 0x40  # Address
        optionalData[1] = 0x02  # Length

        packet = self._buildPacket(
            servoID, Command.HRAMREAD, optionalData)
        readBuffer = self._sendDataForResult(packet)

        if not self._isRightPacket(readBuffer):
            return -1

        if len(readBuffer) < 11:
            self._logger.error(
                "Strange Packet, expected len=11: %s", [str(x) for x in readBuffer])
            return -1

        speed = ((readBuffer[10] & 0x03) << 8) | (readBuffer[9] & 0xFF)

        if (readBuffer[10] & 0x40) == 0x40:
            speed *= -1

        return speed

    def moveOne(self, servoID, goalPosition, playTime, ledColor=LedColor.LED_OFF):
        """
        Move one servo at goal position 0 - 1023

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: goalPos 0 ~ 1023
        @param: playTime 0 ~ 2856ms
        @param: ledColor LedColor.LED_RED | LedColor.LED_GREEN | LedColor.LED_BLUE
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))
        if goalPosition > 1023 or goalPosition < 0:
            raise ValueError(
                "Invalid goalPosition. Got: %s Expected -1023 ~ 1023" % (goalPosition,))
        if playTime < 0 or playTime > 2856:
            raise ValueError(
                "Invalid playTime. Got: %s Expected 0 ~ 2856" % (playTime,))

        pos_LSB = goalPosition & 0X00FF
        pos_MSB = (goalPosition & 0XFF00) >> 8
        playtimeVal = int(round(playTime / 11.2))  # ms --> value
        ledColor = ledColor & 0xFD  # Pos Ctrl Mode

        self._logger.log(1, "Moving %s to %s in %sms" %
                         (servoID, goalPosition, playTime))

        optionalData = [0] * 5
        optionalData[0] = playtimeVal  # Execution time in ms / 11.2
        optionalData[1] = pos_LSB
        optionalData[2] = pos_MSB
        optionalData[3] = ledColor
        optionalData[4] = servoID

        packet = self._buildPacket(servoID, Command.HSJOG, optionalData)
        self._sendData(packet)
        # self._logger.debug(self.errorText(servoID))

    def getPosition(self, servoID):
        """
        Get servo position

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @return: current position 0 ~ 1023 (-1: failure)
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        optionalData = [0] * 2
        optionalData[0] = 0x3A  # Address
        optionalData[1] = 0x02  # Length

        packet = self._buildPacket(
            servoID, Command.HRAMREAD, optionalData)
        readBuffer = self._sendDataForResult(packet)

        if not self._isRightPacket(readBuffer):
            return -1

        if len(readBuffer) < 11:
            self._logger.error(
                "Strange Packet, expected len=11: %s", [str(x) for x in readBuffer])
            self._logger.error("Sent packet: %s", [str(x) for x in packet])
            return -1
        pos = ((readBuffer[10] & 0x03) << 8) | (readBuffer[9] & 0xFF)
        return pos

    def moveOneAngle(self, servoID, angle, playTime, ledColor=LedColor.LED_OFF):
        """
        Move one servo to an angle between -167 and 167

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: angle -166 ~ 166 degrees
        @param: playTime 0 ~ 2856 ms
        @param: ledColor LedColor.LED_RED | LedColor.LED_GREEN | LedColor.LED_BLUE
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        if angle > 167.0 or angle < -167.0:
            return
        position = (angle / 0.325) + 512
        self.moveOne(
            servoID, position, playTime, ledColor=LedColor.LED_OFF)

    def getAngle(self, servoID):
        """
        Get servo position in degrees

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @return: current angle -166.7 ~ 166.7 degrees
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        pos = self.getPosition(servoID)
        if pos < 0:
            return -1
        return (pos - 512) * 0.325

    def addMove(self, servoID, goal, ledColor=LedColor.LED_OFF):
        """
        Add one servo movement data

        @example:  addMove(0, 512, LedColor.LED_RED)
                   addMove(1, 235, LedColor.LED_GREEN)
                   addMove(2, 789, LedColor.LED_BLUE)
                   actionAll(1000)

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: goal 0 ~ 1023
        @param: ledColor LedColor.LED_RED | LedColor.LED_GREEN | LedColor.LED_BLUE
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        if goal > 1023 or goal < 0:
            return  # 0 <--> 1023 range

        # Position definition
        posLSB = goal & 0X00FF  # MSB Pos
        posMSB = (goal & 0XFF00) >> 8  # LSB Pos
        ledColor = ledColor & 0xFD  # Pos Ctrl Mode

        optionalData = [0] * 4
        optionalData[0] = posLSB
        optionalData[1] = posMSB
        optionalData[2] = ledColor
        optionalData[3] = servoID

        self.addData(optionalData)  # add servo data to list, pos mode

    def addAngle(self, servoID, angle, ledColor=LedColor.LED_OFF):
        """
        Add one servo movement data in degrees

        @example:  addAngle(0, -90.5, LedColor.LED_RED)
                   addAngle(1, 0, LedColor.LED_BLUE)
                   addAngle(2, 90.5, LedColor.LED_GREEN)
                   actionAll(1000)

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: angle -167 ~ 167 degrees
        @param: ledColor LedColor.LED_RED | LedColor.LED_GREEN | LedColor.LED_BLUE
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        if angle > 167.0 or angle < -167.0:
            return  # out of the range
        position = (angle / 0.325) + 512
        self.addMove(servoID, position, ledColor=LedColor.LED_OFF)

    def addSpeed(self, servoID, goalSpeed, ledColor=LedColor.LED_OFF):
        """
        Add one servo infinite turn speed data

        @example:  addSpeed(0, 512, LedColor.LED_RED)
                   addSpeed(1, -512, LedColor.LED_GREEN)
                   addSpeed(2, -300, LedColor.LED_BLUE)
                   actionAll(1000)

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: goalSpeed -1023 ~ 1023 [CW:Negative Value(-), CCW:Positive Value(+)]
        @param: ledColor LedColor.LED_RED | LedColor.LED_GREEN | LedColor.LED_BLUE
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))
        if goalSpeed > 1023 or goalSpeed < -1023:
            raise ValueError(
                "Invalid goalSpeed. Got: %s Expected -1023 ~ 1023" % (goalSpeed,))

        if goalSpeed < 0:
            goalSpeedSign = (-1) * goalSpeed
            goalSpeedSign |= 0x4000
        else:
            goalSpeedSign = goalSpeed

        goalSpeedLSB = goalSpeedSign & 0X00FF
        goalSpeedMSB = (goalSpeedSign & 0xFF00) >> 8

        ledColor = ledColor | 0x02  # Speed Ctrl Mode

        optionalData = [0] * 4
        optionalData[0] = goalSpeedLSB
        optionalData[1] = goalSpeedMSB
        optionalData[2] = ledColor
        optionalData[3] = servoID

        self.addData(optionalData)  # add servo data to list, speed mode

    def addData(self, optionalData):
        """
        Add raw data to the multiple move list

        @param: optionalData Raw data to add
        """
        if len(self._multipleMoveData) + len(optionalData) >= 4 * 53:
            raise ValueError(
                "A SJOG can deal with only 53 motors at one time.  Adding data would overload SJOG")

        for i in range(0, len(optionalData)):
            self._multipleMoveData.append(optionalData[i])

    def actionAll(self, playTime):
        """
        Move(Turn) all servos with the same execution time

        @example:  addMove(0, 512, LedColor.LED_RED)
                   addAngle(1, 90.5f, LedColor.LED_GREEN)
                   addSpeed(2, -300, LedColor.LED_BLUE)
                   actionAll(1000)

        @param: playTime 0 ~ 2865 ms
        """
        if playTime < 0 or playTime > 2856:
            raise ValueError(
                "Invalid goal_speed. Got: %s Expected 0 ~ 2865" % (playTime,))

        optionalDataSize = len(self._multipleMoveData)
        if optionalDataSize < 4:
            return

        optionalData = [0] * optionalDataSize + 1
        optionalData[0] = playTime
        for i in range(0, optionalDataSize):
            optionalData[i + 1] = self._multipleMoveData[i]

        packet = self._buildPacket(0xFE, Command.HSJOG, optionalData)
        self._sendData(packet)

        self._multipleMoveData.clear()

    def setLed(self, servoID, ledColor):
        """
        LED Control -  GREEN, BLUE, RED, OFF

        @param: servoID 0 ~ 254 (0x00 ~ 0xFE), 0xFE : BROADCAST_ID
        @param: ledColor LedColor.LED_RED | LedColor.LED_GREEN | LedColor.LED_BLUE
        """
        if servoID < 0 or servoID > 0xFE:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 254" % (servoID,))

        optionalData = [0] * 3
        optionalData[0] = 0x35  # Address
        optionalData[1] = 0x01  # Length

        ledColor2 = 0x00
        if (ledColor & LedColor.LED_GREEN) == LedColor.LED_GREEN:
            ledColor2 |= 0x01
        if (ledColor & LedColor.LED_BLUE) == LedColor.LED_BLUE:
            ledColor2 |= 0x02
        if (ledColor & LedColor.LED_RED) == LedColor.LED_RED:
            ledColor2 |= 0x04

        optionalData[2] = ledColor2
        packet = self._buildPacket(
            servoID, Command.HRAMWRITE, optionalData)
        self._sendData(packet)

    def reboot(self, servoID):
        """
        Reboot a servo

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        packet = self._buildPacket(servoID, Command.HREBOOT, None)
        self._sendData(packet)

    def rollback(self, servoID):
        """
        Revert a servo to factory defaults

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        packet = self._buildPacket(servoID, Command.HROLLBACK, None)
        self._sendData(packet)

    def status(self, servoID, detail=False):
        """
        Servo Status

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: detail return detail about H_ERROR_INVALID_PKT
        @return: (error, detail) if detail==True else error
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        packet = self._buildPacket(servoID, Command.HSTAT, None)
        readBuffer = self._sendDataForResult(packet)

        if not self._isRightPacket(readBuffer):
            if detail:
                return (-1, None)
            return -1

        if detail:
            return (readBuffer[7], readBuffer[8])
        else:
            return readBuffer[7]

    def errorText(self, servoID):
        """
        Return a text representation of the current servo errors

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @return: list containing all error text
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        statuscode, detailcode = self.status(servoID, True)

        if statuscode <= -1:
            return ['Invalid response recieved, unknown status', ]

        if statuscode == Error.H_STATUS_OK:
            return []

        codes = []
        if statuscode & Error.H_ERROR_INPUT_VOLTAGE == Error.H_ERROR_INPUT_VOLTAGE:
            codes.append('Exceeded Input Voltage')
        if statuscode & Error.H_ERROR_POS_LIMIT == Error.H_ERROR_POS_LIMIT:
            codes.append('Exceeded Position Limit')
        if statuscode & Error.H_ERROR_TEMPERATURE_LIMIT == Error.H_ERROR_TEMPERATURE_LIMIT:
            codes.append('Exceeded Temperature Limit')
        if statuscode & Error.H_ERROR_INVALID_PKT == Error.H_ERROR_INVALID_PKT:
            details = []
            if detailcode & Error.H_PKTERR_CHECKSUM == Error.H_PKTERR_CHECKSUM:
                details.append('Checksum Error')
            if detailcode & Error.H_PKTERR_UNKNOWN_CMD == Error.H_PKTERR_UNKNOWN_CMD:
                details.append('Unknown Command')
            if detailcode & Error.H_PKTERR_EXCEED_REG_RANGE == Error.H_PKTERR_EXCEED_REG_RANGE:
                details.append('Exceed REG range')
            if detailcode & Error.H_PKTERR_GARBAGE == Error.H_PKTERR_GARBAGE:
                details.append('Garbage detected')
            codes.append('Invalid Packet Recieved: %s' % details)
        if statuscode & Error.H_ERROR_OVERLOAD == Error.H_ERROR_OVERLOAD:
            codes.append('Overload')
        if statuscode & Error.H_ERROR_DRIVER_FAULT == Error.H_ERROR_DRIVER_FAULT:
            codes.append('Driver Fault')
        if statuscode & Error.H_ERROR_EEPREG_DISTORT == Error.H_ERROR_EEPREG_DISTORT:
            codes.append('EEP Registry Distorted')

        return codes

    def model(self, servoID):
        """
        Get the model of a servo

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @return:1 = DRS-0101, 2 = DRS-0201
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        optionalData = [0] * 2
        optionalData[0] = 0x00  # Address
        optionalData[1] = 0x01  # Length

        packet = self._buildPacket(servoID, Command.HEEPREAD, optionalData)

        readBuffer = self._sendDataForResult(packet)

        if not self.is_right_packet(readBuffer):
            return -1

        return readBuffer[8]  # return model

    def setID(self, oldID, newID):
        """
        change the ID of a servo

          CAUTION - If there are duplicated servo IDs on your Serial Line,It does not work.
                  When you change your servo's ID, make sure there is only the servo on the line if possible.

        @param: oldID the current servo id 0 ~ 253 (0x00 ~ 0xFD)
        @param: newID the id to change to 0 ~ 253 (0x00 ~ 0xFD)
        @return: true - success, false - failure
        """
        if oldID < 0 or oldID > 0xFD:
            raise ValueError(
                "Invalid oldID. Got: %s Expected 0 ~ 253" % (oldID,))
        if newID < 0 or newID > 0xFD:
            raise ValueError(
                "Invalid newID. Got: %s Expected 0 ~ 253" % (newID,))

        if self.getPosition(oldID) == -1:
            return False

        optionalData = [0] * 3
        optionalData[0] = 0x06  # Address
        optionalData[1] = 0x01  # Length
        optionalData[2] = newID

        packet = self._buildPacket(oldID, Command.HEEPWRITE, optionalData)
        self._sendData(packet)

        self.reboot(oldID)
        return True

    def _writeRegistryRAM(self, servoID, address, data, length=1):
        """
        Write registry in the RAM: one byte

        See HerkuleX Manual for addresses and valid values

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: address
        @param: data
        @param: length of data to write (1 or 2)
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        if length is None:
            length = 1 + (data > 255)
        optionalData = [0] * (2 + length)
        optionalData[0] = address  # Address
        optionalData[1] = length  # Length
        if length == 1:
            optionalData[2] = data
        else:
            optionalData[2] = data & 0X00FF
            optionalData[3] = (data & 0XFF00) >> 8

        packet = self._buildPacket(
            servoID, Command.HRAMWRITE, optionalData)
        self._sendData(packet)

    def _writeRegistryEEP(self, servoID, address, data, length=1):
        """
        write registry in the EEP memory (ROM)

        See HerkuleX Manual for addresses and valid values

        @param: servoID 0 ~ 253 (0x00 ~ 0xFD)
        @param: address
        @param: data
        @param: length of data to write (1 or 2)
        """
        if servoID < 0 or servoID > 0xFD:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 253" % (servoID,))

        if length is None:
            length = 1 + (data > 255)
        optionalData = [0] * (2 + length)
        optionalData[0] = address  # Address
        optionalData[1] = length  # Length
        if length == 1:
            optionalData[2] = data
        else:
            optionalData[2] = data & 0X00FF
            optionalData[3] = (data & 0XFF00) >> 8

        packet = self._buildPacket(servoID, Command.HEEPWRITE, optionalData)
        self._sendData(packet)

    def _isRightPacket(self, packetBuffer):
        """
        Verify that packet is complete and passes checksums

        @param: packetBuffer packet to be checked
        @return: True if valid, else False
        """
        if len(packetBuffer) < 7:
            return False

        if len(packetBuffer) != packetBuffer[2]:
            self._logger.warning(
                "Invalid packet length! %s", [str(x) for x in packetBuffer])
            return False

        chksum1 = self._checksum1(packetBuffer)  # Checksum1
        chksum2 = self._checksum2(chksum1)  # Checksum2

        if chksum1 != packetBuffer[5]:
            self._logger.warning(
                "Invalid packet checksum1! %s", [str(x) for x in packetBuffer])
            return False
        if chksum2 != packetBuffer[6]:
            self._logger.warning(
                "Invalid packet checksum2! %s", [str(x) for x in packetBuffer])
            return False

        return True

    def _buildPacket(self, servoID, command, optionalData=None):
        """
        Build a packet for sending, adding headers and checksums

        @param: servoID 0 ~ 254 (0x00 ~ 0xFE)
        @param: command command to send to servo
        @param: optionalData optional data per command specificiation
        @return: completed servo packet
        """
        if servoID < 0 or servoID > 0xFE:
            raise ValueError(
                "Invalid servoID. Got: %s Expected 0 ~ 254" % (servoID,))

        if optionalData is None:
            pktSize = BASIC_PKT_SIZE
        else:
            pktSize = BASIC_PKT_SIZE + len(optionalData)

        packet = [0] * pktSize

        packet[0] = 0xFF  # Packet Header
        packet[1] = 0xFF  # Packet Header
        packet[2] = pktSize  # Packet Size
        packet[3] = servoID  # Servo ID
        packet[4] = command  # Command

        for i in range(0, pktSize - BASIC_PKT_SIZE):
            packet[7 + i] = optionalData[i]

        packet[5] = self._checksum1(packet)
        packet[6] = self._checksum2(packet[5])

        return packet

    def _checksum1(self, packetBuffer):
        """
        Compute checksum 1 for packet
             (PacketSize ^ pID ^ CMD ^ Data[0] ^ ... ^ Data[n]) & 0xFE

        @param: packetBuffer the servo packet to compute
        @return computed checksum
        """
        chksum1 = 0x00

        skip = [0, 1, 5, 6]  # Header and checksum 1/2
        for i in range(0, len(packetBuffer)):
            if i in skip:
                continue
            chksum1 ^= packetBuffer[i]

        return chksum1 & 0xFE

    def _checksum2(self, packetBuffer):
        """
        Compute checksum 2 for packet
            (~(PacketSize ^ pID ^ CMD ^ Data[0] ^ ... ^ Data[n-1])) & 0xFE

        @param: packetBuffer the servo packet to compute
        @return computed checksum
        """
        # Should be:
        #  (~(Packet Size ^ pID ^ CMD ^ Data[0] ^ ... ^ Data[n-1])) & 0xFE
        return (~packetBuffer) & 0xFE

    def _sendData(self, packetBuffer):
        """
        Send data to the servo line

        @param: packetBuffer data to be sent
        """
        with self._portLock:
            self._logger.log(
                1, "Sending packet: [%s]" % ', '.join([str(x) for x in packetBuffer]))
            packet = ''.join([chr(x) for x in packetBuffer])
            self._port.write(packet)

    def _sendDataForResult(self, packetBuffer):
        """
        Send data to the servo line and wait for a response

        @param: packetBuffer data to be sent
        @return: the response data or [] on error
        """
        with self._portLock:
            # Flush any errant data on the input buffer
            self._port.flushInput()
            self._sendData(packetBuffer)
            ack_delay = WAIT_TIME_BY_ACK / 1000.0

            # Extra checking and filtering to handle noisy serial lines
            # Locate the start of the header
            readBuffer = [0xFF, 0xFF]
            startTime = time()
            while time() - startTime < self._port.timeout + ack_delay:
                inBuffer = self._port.read(1)
                if len(inBuffer) == 0:
                    continue
                byte = ord(inBuffer) & 0xFF
                if byte > 0xE9:  # max valid packet length
                    continue
                readBuffer.append(byte)
                break

            if len(readBuffer) > 2:
                inBuffer = self._port.read(readBuffer[2] - 3)
                [readBuffer.append(ord(c) & 0xFF) for c in inBuffer]

            if len(readBuffer) > 2 and len(readBuffer) < readBuffer[2] and self._port.inWaiting():
                self._logger.warning("Not all bytes received before timeout!")
                inBuffer = self._port.read(
                    min(self._port.inWaiting(), readBuffer[2] - len(readBuffer)))

        if len(readBuffer) == 2:
            readBuffer = []
            self._logger.warning(
                "No data received before timeout! Send packet: %s", [str(x) for x in packetBuffer])
        self._logger.log(
            1, "Received packet: [%s]" % ', '.join([str(x) for x in readBuffer]))

        return readBuffer
