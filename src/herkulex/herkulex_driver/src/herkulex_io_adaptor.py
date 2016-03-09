"""
Adaptor for herkulex servos to immitate dynamixel controller

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
@modified    2016.03.14
@version     0.1
"""

from herkulex_io import *

from dynamixel_io import ChecksumError, DroppedPacketError, ErrorCodeError, FatalErrorCodeError, NonfatalErrorCodeError, SerialOpenError, UnsupportedFeatureError


class HerkulexIO(object):
    """ Provides low level IO with the Dynamixel servos through pyserial. Has the
    ability to write instruction packets, request and read register value
    packets, send and receive a response to a ping packet, and send a SYNC WRITE
    multi-servo instruction packet.
    """

    def __init__(self, port, baudrate, readback_echo=False):
        """ Constructor takes serial port and baudrate as arguments. """
        try:
            self._herkulex = HerkuleX(port, baudrate)
        except Exception:
            raise SerialOpenError(port, baudrate)

    def __del__(self):
        """ Destructor calls DynamixelIO.close """
        self.close()

    def close(self):
        """
        Be nice, close the serial port.
        """
        if self._herkulex:
            self._herkulex.close()

    def read(self, servo_id, address, size):
        """ Read "size" bytes of data from servo with "servo_id" starting at the
        register with "address". "address" is an integer between 0 and 57. It is
        recommended to use the constants in module dynamixel_const for readability.

        To read the position from servo with id 1, the method should be called
        like:
            read(1, DXL_GOAL_POSITION_L, 2)
        """
        self._herkulex._readRegistryRAM(servo_id, address, size)

    def write(self, servo_id, address, data):
        """ Write the values from the "data" list to the servo with "servo_id"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data). "address" is an integer between
        0 and 49. It is recommended to use the constants in module dynamixel_const
        for readability. "data" is a list/tuple of integers.

        To set servo with id 1 to position 276, the method should be called
        like:
            write(1, DXL_GOAL_POSITION_L, (20, 1))
        """
        self._herkulex._writeRegistryRAM(servo_id, address, data)

    def sync_write(self, address, data):
        """ Use Broadcast message to send multiple servos instructions at the
        same time. No "status packet" will be returned from any servos.
        "address" is an integer between 0 and 49. It is recommended to use the
        constants in module dynamixel_const for readability. "data" is a tuple of
        tuples. Each tuple in "data" must contain the servo id followed by the
        data that should be written from the starting address. The amount of
        data can be as long as needed.

        To set servo with id 1 to position 276 and servo with id 2 to position
        550, the method should be called like:
            sync_write(DXL_GOAL_POSITION_L, ( (1, 20, 1), (2 ,38, 2) ))
        """
        pass

    def ping(self, servo_id):
        """ Ping the servo with "servo_id". This causes the servo to return a
        "status packet". This can tell us if the servo is attached and powered,
        and if so, if there are any errors.
        """
        return self._herkulex.status(servo_id)

    ######################################################################
    # These function modify EEPROM data which persists after power cycle #
    ######################################################################

    def set_id(self, old_id, new_id):
        """
        Sets a new unique number to identify a motor. The range from 1 to 253
        (0xFD) can be used.
        """
        self._herkulex.setID(old_id, new_id)

    def set_baud_rate(self, servo_id, baud_rate):
        """
        Sets servo communication speed. The range from 0 to 254.
        """
        pass

    def set_return_delay_time(self, servo_id, delay):
        """
        Sets the delay time from the transmission of Instruction Packet until
        the return of Status Packet. 0 to 254 (0xFE) can be used, and the delay
        time per data value is 2 usec.
        """
        pass

    def set_angle_limit_cw(self, servo_id, angle_cw):
        """
        Set the min (CW) angle of rotation limit.
        """
        pass

    def set_angle_limit_ccw(self, servo_id, angle_ccw):
        """
        Set the max (CCW) angle of rotation limit.
        """
        pass

    def set_angle_limits(self, servo_id, min_angle, max_angle):
        """
        Set the min (CW) and max (CCW) angle of rotation limits.
        """
        pass

    def set_drive_mode(self, servo_id, is_slave=False, is_reverse=False):
        """
        Sets the drive mode for EX-106 motors
        """
        pass

    def set_voltage_limit_min(self, servo_id, min_voltage):
        """
        Set the minimum voltage limit.
        NOTE: the absolute min is 5v
        """
        pass

    def set_voltage_limit_max(self, servo_id, max_voltage):
        """
        Set the maximum voltage limit.
        NOTE: the absolute min is 25v
        """
        pass

    def set_voltage_limits(self, servo_id, min_voltage, max_voltage):
        """
        Set the min and max voltage limits.
        NOTE: the absolute min is 5v and the absolute max is 25v
        """
        pass

    ###############################################################
    # These functions can send a single command to a single servo #
    ###############################################################

    def set_torque_enabled(self, servo_id, enabled):
        """
        Sets the value of the torque enabled register to 1 or 0. When the
        torque is disabled the servo can be moved manually while the motor is
        still powered.
        """
        pass

    def set_compliance_margin_cw(self, servo_id, margin):
        """
        The error between goal position and present position in CW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        pass

    def set_compliance_margin_ccw(self, servo_id, margin):
        """
        The error between goal position and present position in CCW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        pass

    def set_compliance_margins(self, servo_id, margin_cw, margin_ccw):
        """
        The error between goal position and present position in CCW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        pass

    def set_compliance_slope_cw(self, servo_id, slope):
        """
        Sets the level of Torque near the goal position in CW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        pass

    def set_compliance_slope_ccw(self, servo_id, slope):
        """
        Sets the level of Torque near the goal position in CCW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        pass

    def set_compliance_slopes(self, servo_id, slope_cw, slope_ccw):
        """
        Sets the level of Torque near the goal position in CW/CCW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        pass

    def set_d_gain(self, servo_id, d_gain):
        """
        Sets the value of derivative action of PID controller.
        Gain value is in range 0 to 254.
        """
        pass

    def set_i_gain(self, servo_id, i_gain):
        """
        Sets the value of integral action of PID controller.
        Gain value is in range 0 to 254.
        """
        pass

    def set_p_gain(self, servo_id, p_gain):
        """
        Sets the value of proportional action of PID controller.
        Gain value is in range 0 to 254.
        """
        pass

    def set_punch(self, servo_id, punch):
        """
        Sets the limit value of torque being reduced when the output torque is
        decreased in the Compliance Slope area. In other words, it is the mimimum
        torque. The initial value is 32 (0x20) and can be extended up to 1023
        (0x3FF). (Refer to Compliance margin & Slope)
        """
        pass

    def set_acceleration(self, servo_id, acceleration):
        """
        Sets the acceleration. The unit is 8.583 Degree / sec^2.
        0 - acceleration control disabled, 1-254 - valid range for acceleration.
        """
        pass

    def set_position(self, servo_id, position):
        """
        Set the servo with servo_id to the specified goal position.
        Position value must be positive.
        """
        pass

    def set_speed(self, servo_id, speed):
        """
        Set the servo with servo_id to the specified goal speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        pass

    def set_torque_limit(self, servo_id, torque):
        """
        Sets the value of the maximum torque limit for servo with id servo_id.
        Valid values are 0 to 1023 (0x3FF), and the unit is about 0.1%.
        For example, if the value is 512 only 50% of the maximum torque will be used.
        If the power is turned on, the value of Max Torque (Address 14, 15) is used as the initial value.
        """
        pass

    def set_goal_torque(self, servo_id, torque):
        """
        Set the servo to torque control mode (similar to wheel mode, but controlling the torque)
        Valid values are from -1023 to 1023.
        Anything outside this range or 'None' disables torque control.
        """
        pass

    def set_position_and_speed(self, servo_id, position, speed):
        """
        Set the servo with servo_id to specified position and speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        pass

    def set_led(self, servo_id, led_state):
        """
        Turn the LED of servo motor on/off.
        Possible boolean state values:
            True - turn the LED on,
            False - turn the LED off.
        """
        pass

    #################################################################
    # These functions can send multiple commands to multiple servos #
    # These commands are used in ROS wrapper as they don't send a   #
    # response packet, ROS wrapper gets motor states at a set rate  #
    #################################################################

    def set_multi_torque_enabled(self, valueTuples):
        """
        Method to set multiple servos torque enabled.
        Should be called as such:
        set_multi_servos_to_torque_enabled( (id1, True), (id2, True), (id3, True) )
        """
        pass

    def set_multi_compliance_margin_cw(self, valueTuples):
        """
        Set different CW compliance margin for multiple servos.
        Should be called as such:
        set_multi_compliance_margin_cw( ( (id1, margin1), (id2, margin2), (id3, margin3) ) )
        """
        pass

    def set_multi_compliance_margin_ccw(self, valueTuples):
        """
        Set different CCW compliance margin for multiple servos.
        Should be called as such:
        set_multi_compliance_margin_ccw( ( (id1, margin1), (id2, margin2), (id3, margin3) ) )
        """
        pass

    def set_multi_compliance_margins(self, valueTuples):
        """
        Set different CW and CCW compliance margins for multiple servos.
        Should be called as such:
        set_multi_compliance_margins( ( (id1, cw_margin1, ccw_margin1), (id2, cw_margin2, ccw_margin2) ) )
        """
        pass

    def set_multi_compliance_slope_cw(self, valueTuples):
        """
        Set different CW compliance slope for multiple servos.
        Should be called as such:
        set_multi_compliance_slope_cw( ( (id1, slope1), (id2, slope2), (id3, slope3) ) )
        """
        pass

    def set_multi_compliance_slope_ccw(self, valueTuples):
        """
        Set different CCW compliance slope for multiple servos.
        Should be called as such:
        set_multi_compliance_slope_ccw( ( (id1, slope1), (id2, slope2), (id3, slope3) ) )
        """
        pass

    def set_multi_compliance_slopes(self, valueTuples):
        """
        Set different CW and CCW compliance slopes for multiple servos.
        Should be called as such:
        set_multi_compliance_slopes( ( (id1, cw_slope1, ccw_slope1), (id2, cw_slope2, ccw_slope2) ) )
        """
        pass

    def set_multi_punch(self, valueTuples):
        """
        Set different punch values for multiple servos.
        NOTE: according to documentation, currently this value is not being used.
        Should be called as such:
        set_multi_punch( ( (id1, punch1), (id2, punch2), (id3, punch3) ) )
        """
        pass

    def set_multi_position(self, valueTuples):
        """
        Set different positions for multiple servos.
        Should be called as such:
        set_multi_position( ( (id1, position1), (id2, position2), (id3, position3) ) )
        """
        pass

    def set_multi_speed(self, valueTuples):
        """
        Set different speeds for multiple servos.
        Should be called as such:
        set_multi_speed( ( (id1, speed1), (id2, speed2), (id3, speed3) ) )
        """
        pass

    def set_multi_torque_limit(self, valueTuples):
        """
        Set different torque limits for multiple servos.
        Should be called as such:
        set_multi_torque_limit( ( (id1, torque1), (id2, torque2), (id3, torque3) ) )
        """
        pass

    def set_multi_position_and_speed(self, valueTuples):
        """
        Set different positions and speeds for multiple servos.
        Should be called as such:
        set_multi_position_and_speed( ( (id1, position1, speed1), (id2, position2, speed2), (id3, position3, speed3) ) )
        """
        pass

    #################################
    # Servo status access functions #
    #################################

    def get_model_number(self, servo_id):
        """ Reads the servo's model number (e.g. 12 for AX-12+). """
        pass

    def get_firmware_version(self, servo_id):
        """ Reads the servo's firmware version. """
        pass

    def get_return_delay_time(self, servo_id):
        """ Reads the servo's return delay time. """
        pass

    def get_angle_limits(self, servo_id):
        """
        Returns the min and max angle limits from the specified servo.
        """
        pass

    def get_drive_mode(self, servo_id):
        """ Reads the servo's drive mode. """
        pass

    def get_voltage_limits(self, servo_id):
        """
        Returns the min and max voltage limits from the specified servo.
        """
        pass

    def get_position(self, servo_id):
        """ Reads the servo's position value from its registers. """
        pass

    def get_speed(self, servo_id):
        """ Reads the servo's speed value from its registers. """
        pass

    def get_voltage(self, servo_id):
        """ Reads the servo's voltage. """
        pass

    def get_current(self, servo_id):
        """ Reads the servo's current consumption (if supported by model) """
        pass

    def get_feedback(self, servo_id):
        """
        Returns the id, goal, position, error, speed, load, voltage, temperature
        and moving values from the specified servo.
        """
        pass

    def get_led(self, servo_id):
        """
        Get status of the LED. Boolean return values:
            True - LED is on,
            False - LED is off.
        """
        pass

    def exception_on_error(self, error_code, servo_id, command_failed):
        pass
