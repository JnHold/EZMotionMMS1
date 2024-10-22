"""
EZMotion_python.py

This module contains the class EZMotion_MMS_1 which is used to control and interact with Gen1 Motors using the MODBUS protocol over a serial connection. 

The class includes constants for converting between different units of measurement (e.g., speed, position), MODBUS function codes, and register addresses for the Gen1 Motors. 

The class also includes an initializer that sets up a serial connection with specific parameters (port, baudrate, bytesize, and parity).

The module depends on the `serial` and `crccheck.crc` libraries.

Classes:
    EZMotion_MMS_1: A class used to control and interact with Gen1 Motors using the MODBUS protocol over a serial connection.

Methods:
    __init__: Initializes the serial connection with specific parameters.
    Set_Slave_Address: Sets the slave address for the MODBUS communication.
    MODBUS_CRC: Calculates the MODBUS CRC of a given message.
    writeRegSingle: Writes a single register with a given value.
    writeRegMultiple: Writes multiple registers with given values.
    #TODO: Add rest of the Methods
Constants:
    SPD_REG2RPM, SPD_RPM2REG, POS_DEG2REG, SLP_RPM2REG, POS_DEG2REG: Constants for converting between different units of measurement.
    READ_COILS, READ_DISCRETE_INPUTS, READ_HOLDING_REGISTERS, READ_INPUT_REGISTERS, WRITE_SINGLE_COIL, WRITE_SINGLE_REGISTER, WRITE_MULTIPLE_COILS, WRITE_MULTIPLE_REGISTERS: MODBUS function codes.
    G1_REG_MODE, G1_REG_ENABLE, G1_REG_SPD_H, G1_REG_SPD_L, G1_REG_UPDATE, G1_REG_TURNS, G1_REG_DEGREES, G1_REG_POS_SLOPE, G1_REG_PLOOP_LIM, G1_REG_IQ_CMD, G1_REG_AD_GAIN, G1_REG_IQ_NOMINAL, G1_REG_RATED_POWER, G1_REG_RATED_SPEED, G1_REG_GET_SPEED, G1_REG_GET_TORQ_CONST: Register addresses for the Gen1 Motors.
    # TODO: Add rest of the Constants

Author: Jonathon Holder
Date: 12/12/23
Last Modified: 18/1/24
    """

import serial
import time
from crccheck.crc import Crc16Modbus


class EZMotion_MMS_1:
    # Constants
    SPD_REG2RPM: float = 0.00013970
    SPD_RPM2REG: float = 7158.3
    POS_DEG2REG: float = 182.04
    SLP_RPM2REG: float = 0.1092
    POS_DEG2REG: float = 182.04
    POS_REG2DEG: float = 0.00549

    # MODBUS FUNCTION CODES (Not all FC are used)
    READ_COILS = 0x01
    READ_DISCRETE_INPUTS = 0x02
    READ_HOLDING_REGISTERS = 0x03
    READ_INPUT_REGISTERS = 0x04
    WRITE_SINGLE_COIL = 0x05
    WRITE_SINGLE_REGISTER = 0x06
    WRITE_MULTIPLE_COILS = 0x0F
    WRITE_MULTIPLE_REGISTERS = 0x10

    # COMMON REGISTERS for Gen1 Motors
    G1_REG_MODE = 0x34  # Control Mode[0:1]
    G1_REG_ENABLE = 0x70  # Motors enable
    G1_REG_SPD_H = 0x4D  # Speed reference - High
    G1_REG_SPD_L = 0x4E  # Speed reference - Low
    G1_REG_UPDATE = 0x76  # Command update
    G1_REG_TURNS = 0x4A  # Target Position (High byte) - number of turns or revolution
    G1_REG_DEGREES = 0x4B  # Target Position (Low byte) - angle in degrees
    G1_REG_POS_SLOPE = 0x4C  # Position reference - slope (acceleration)
    G1_REG_PLOOP_LIM = (
        0x1C  # Position loop limit (default 0x0147) (Speed limit under Pos_loop)
    )
    G1_REG_IQ_CMD = 0x002F  # 12bit register - Target torque
    G1_REG_AD_GAIN = 0x0059  # AD_Gain
    G1_REG_IQ_NOMINAL = 0x0A  # Nominal Motor rated IQ current
    G1_REG_RATED_POWER = 0x03  # Motor Power Rating
    G1_REG_RATED_SPEED = 0x05  # Motor Rated Speed
    G1_REG_GET_SPEED = 0x61  # Motor Speed Feedback
    G1_REG_GET_TORQ_CONST = 0x12  # Motor Torque Constant

    def __init__(
        self,
        port: str = "/dev/ttyTHS0",
        baudrate: int = 115200,
        bytesize: int = serial.EIGHTBITS,
        parity: str = serial.PARITY_ODD,
        stopbits: int = serial.STOPBITS_ONE,
        timeout: float = 0.01,
    ) -> None:
        # Serial port setup
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout,
        )

    def Set_Slave_Address(self, Slave_add):
        # Set the slave address of the motor, needed for communication
        self.BROAD_SA = Slave_add

    def MODBUS_CRC(self, buf):
        """
        Calculate the Modbus CRC-16 checksum for the given data.

        This function uses the Crc16Modbus class to calculate the CRC-16 checksum
        for the given data. The CRC-16 checksum is commonly used in the Modbus
        protocol for error checking.

        Args:
            buf (bytes): The data to calculate the checksum for.

        Returns:
            int: The calculated CRC-16 checksum.
        """
        crc = Crc16Modbus.calc(buf)
        return crc

    def writeRegSingle(self, regAdd, value):
        """
        Write a single register value over a Modbus connection.

        This function constructs a Modbus message with the given register address and value,
        calculates the CRC for error checking, and sends the message over a serial connection.

        Args:
            regAdd (int): The register address to write to.
            value (int): The value to write.
        """

        time.sleep(0.00175)
        msgBuf = bytearray(
            [
                self.BROAD_SA,
                self.WRITE_SINGLE_REGISTER,
                (regAdd & 0xFF00) >> 8,
                regAdd & 0x00FF,
                (value & 0xFF00) >> 8,
                value & 0x00FF,
            ]
        )
        CRC_result = self.MODBUS_CRC(msgBuf)
        msgBuf.extend([(CRC_result & 0x00FF), (CRC_result & 0xFF00) >> 8])
        self.ser.write(msgBuf)
        self.ser.flush()
        time.sleep(0.00175)

    def writeRegMultiple(self, regAdd, regNum, numBytes, value1):
        """
        Write multiple register values over a Modbus connection.

        This function constructs a Modbus message with the given register address, number of registers,
        number of bytes, and value, calculates the CRC for error checking, and sends the message over a serial connection.

        Args:
            regAdd (int): The starting register address to write to.
            regNum (int): The number of registers to write.
            numBytes (int): The number of bytes to write.
            value1 (int): The value to write.
        """

        time.sleep(0.00175)
        msgBuf1 = bytearray(
            [
                self.BROAD_SA,
                self.WRITE_MULTIPLE_REGISTERS,
                (regAdd & 0xFF00) >> 8,
                regAdd & 0x00FF,
                (regNum & 0xFF00) >> 8,
                regNum & 0x00FF,
                0x04,
                (value1 & 0xFF000000) >> 24,
                (value1 & 0x00FF0000) >> 16,
                (value1 & 0x0000FF00) >> 8,
                value1 & 0x000000FF,
            ]
        )
        CRC_result = self.MODBUS_CRC(msgBuf1)
        msgBuf1.extend([(CRC_result & 0x00FF), (CRC_result & 0xFF00) >> 8])
        self.ser.write(msgBuf1)
        # print(f"msgBuf1: {msgBuf1}")
        self.ser.flush()
        time.sleep(0.00175)

    def readRegSingle(self, regAdd, quantity):
        """
        Read a single register or multiple registers from a Modbus device.

        This function constructs a Modbus message with the given register address and quantity of registers to read,
        calculates the CRC for error checking, sends the message over a serial connection, and reads the response.

        Args:
            regAdd (int): The starting register address to read from.
            quantity (int): The number of registers to read.

        Returns:
            int: The register values read from the Modbus device.
        """

        time.sleep(0.00175)
        msgBuf = bytearray(
            [
                self.BROAD_SA,
                self.READ_HOLDING_REGISTERS,
                (regAdd & 0xFF00) >> 8,
                regAdd & 0x00FF,
                (quantity & 0xFF00) >> 8,
                quantity & 0x00FF,
            ]
        )
        CRC_result = self.MODBUS_CRC(msgBuf)
        msgBuf.extend([(CRC_result & 0x00FF), (CRC_result & 0xFF00) >> 8])
        while self.ser.in_waiting:
            msgBuf1 = bytearray(self.ser.read(1))

        self.ser.write(msgBuf)
        # print(f"msgBuf: {msgBuf}")
        self.ser.flush()
        time.sleep(0.00055)

        size = 5 + (quantity * 2)
        msgBuf1 = bytearray(size)

        while self.ser.in_waiting:
            msgBuf1 = bytearray(self.ser.read(size))

        if not self.check_crc(msgBuf1):
            print("CRC check failed")
        value_2 = 0
        if quantity == 2:
            value_2 = (
                msgBuf1[4] + (msgBuf1[3] << 8) + (msgBuf1[6] << 16) + (msgBuf1[5] << 24)
            )

        elif quantity == 1:
            value_2 = int.from_bytes(msgBuf1[3:5], byteorder="big")

        return value_2

    def init(self, rated_power, slave_address=0x01) -> None:
        """
        Initialize the device with the given rated power.

        This function sets the slave address, reads the AD gain from a register,
        sets the AD gain and maximum torque based on the rated power.

        Args:
            rated_power (int): The rated power of the device.

        """

        self.Set_Slave_Address(slave_address)
        AD_Gain_temp = self.readRegSingle(self.G1_REG_AD_GAIN, 0x0001)
        self.AD_Gain = [12, 8, 7, 6, 5, 4, 3, 2][AD_Gain_temp]
        self.max_torque = self.Get_max_torque(rated_power)

    def Set_Velocity(self, RPM):
        """
        Set the velocity of the device in RPM (Revolutions Per Minute).

        This function calculates the velocity in register units, writes it to the appropriate register,
        and then triggers an update.

        Args:
            RPM (int): The desired velocity in RPM.
        """

        vel = int((RPM) * float(self.SPD_RPM2REG))
        Num_of_Reg = 0x0002
        Num_of_Bytes = 0x04
        self.writeRegMultiple(self.G1_REG_SPD_H, Num_of_Reg, Num_of_Bytes, vel)
        self.writeRegSingle(self.G1_REG_UPDATE, 0x0000)

    def Get_Velocity(self):
        """
        Get the current velocity of the device in RPM (Revolutions Per Minute).

        This function reads the velocity from the appropriate register, converts it from register units to RPM,
        and returns it.

        Returns:
            int: The current velocity of the device in RPM.
        """
        Num_of_Reg = 0x0002
        Speed = self.readRegSingle(self.G1_REG_GET_SPEED, Num_of_Reg)
        if Speed > 2147483648:  # check sign
            Speed = Speed - 2**32
            Speed = (Speed - 2**16) * self.SPD_REG2RPM
        else:
            Speed = Speed * self.SPD_REG2RPM
        return int(Speed)

    def Set_Position(self, dir, turns, angle):
        """
        Set the position of the device in terms of turns and angle.

        This function calculates the position in register units, writes it to the appropriate register,
        and then triggers an update.

        Args:
            dir (int): The direction of movement. If 0, the turns and angle are negated.
            turns (int): The number of turns to move.
            angle (float): The angle to move in degrees.
        """
        if dir == 0:
            turns = -turns
            angle = -angle
        position1 = (turns << 16) + int(angle * self.POS_DEG2REG)
        Num_of_Reg = 0x0002
        Num_of_Bytes = 0x04
        self.writeRegMultiple(self.G1_REG_TURNS, Num_of_Reg, Num_of_Bytes, position1)
        self.writeRegSingle(self.G1_REG_UPDATE, 0x0000)

    def Get_Position(self):
        """
        Get the current position of the device in terms of revolutions and angle.

        This function reads the position from the appropriate register, converts it from register units to revolutions and degrees,
        and returns it.

        Returns:
            tuple: The current position of the device in revolutions and degrees.
        """

        Num_of_Reg = 0x0002
        position2 = self.readRegSingle(0x005F, Num_of_Reg)

        check_sign = ((position2 & 0xFFFF0000) >> 16) + ((position2 & 0x0000FFFF) << 16)
        Angle = int((position2 & 0xFFFF0000) >> 16) / self.POS_DEG2REG

        if check_sign > 2147483648:
            position2 = position2 - 2**32
            Revs = (int(position2) & 0xFFFF) - 2**16

        else:
            Revs = int(position2) & 0xFFFF
        return Revs, Angle

    def Pos_Reach_Flag(self):
        """
        Get the position reached flag from the device.

        This function reads the position status from the appropriate register, extracts the position reached flag,
        and returns it.

        Returns:
            int: The position reached flag. 1 if the position has been reached, 0 otherwise.
        """

        ### Check if this is a bool ###

        Num_of_Reg = 0x0001
        Pos_status = self.readRegSingle(0x006A, Num_of_Reg)
        Pos_status = (Pos_status & 0b00000010) >> 1
        return Pos_status

    def Set_Torque(self, set_torque):
        """
        Set the torque of the motor.

        This function calculates the IQ command value based on the set torque, and writes it to the appropriate register.

        Args:
            set_torque (float): The desired torque to be set.
        """

        iq = (float(set_torque / 100.0)) * float(self.IQ_Nominal)
        iq = iq / 1000.0
        # print(f"iq: {iq}")
        temp = (iq * 1.5 * 0.01 * float(self.AD_Gain) * 1024.0) / 1.6
        # print(f"temp: {temp}")
        iq_CMD = int(temp)
        # print(f"iq_CMD: {iq_CMD}")
        self.writeRegSingle(self.G1_REG_IQ_CMD, iq_CMD)

    def Get_Torque(self):
        """
        Get the current torque of the device.

        This function reads the actual torque from the appropriate register, converts it from register units to the torque value,
        and returns it.

        Returns:
            float: The current torque of the device.
        """

        Num_of_Reg = 0x0001
        act_torq = self.readRegSingle(0x63, Num_of_Reg)
        act_torq1 = int(act_torq)
        if act_torq1 >= 2048:
            act_torq1 = (4096 - act_torq1) * (-1)
        temp = (float(act_torq1) * 1.6 * 1000.0) / (
            1.5 * 0.01 * float(self.AD_Gain) * 1024.0
        )
        iq = (float(temp) / float(self.IQ_Nominal)) * 100.0
        return iq

    def Set_Position_Slope(self, RPM):
        """
        Set the position slope of the device.

        This function calculates the position slope in register units based on the given RPM, and writes it to the appropriate register.

        Args:
            RPM (float): The desired RPM for the position slope.
        """

        temp = int(RPM * self.SLP_RPM2REG)
        if temp == 0:
            temp = 1
        self.writeRegSingle(self.G1_REG_PLOOP_LIM, temp)

    def Enable_Op(self):
        """
        Enable the operation of the motor.
        """

        value = 0x0001
        self.writeRegSingle(self.G1_REG_ENABLE, value)

    def Disable_Op(self):
        """
        Disable the operation of the motor.
        """

        value = 0x0000
        self.writeRegSingle(self.G1_REG_ENABLE, value)

    def Op_mode(self, mode) -> None:
        """
        Set the operation mode of the device.

        This function sets the operation mode of the device based on the given mode, and writes the corresponding value to the appropriate register.

        Args:
            mode (str): The desired operation mode. Can be "Position_Abs", "Position_Rel", "Speed", or "Torque".
        """

        if mode == "Position_Abs":
            value = 0x1100
        elif mode == "Position_Rel":
            value = 0x1110
        elif mode == "Speed":
            value = 0x0100
        elif mode == "Torque":
            value = 0x2100
        else:
            print("Invalid Operation Mode")
            return

        self.writeRegSingle(self.G1_REG_MODE, value)
        self.writeRegSingle(self.G1_REG_UPDATE, 0x0000)

    def Get_max_torque(self, Power):
        """
        Get the maximum torque of the motor based on its power rating.

        This function sets the nominal IQ value and returns the rated torque based on the power rating of the motor.

        Args:
            Power (int): The power rating of the motor.

        The function checks the power rating and sets the nominal IQ value and rated torque accordingly.

        Returns:
            int: The rated torque of the motor.
        """
        # print(f"Power: {Power}")

        if Power == 38:  # MMS742038-24 Motor
            self.IQ_Nominal = 2400  # iq (mAmps)
            rated_torque = 120  # Torque (mNm)
            return rated_torque

        elif Power == 52:  # MMS742052-24 Motor
            self.IQ_Nominal = 3600  # iq (mAmps)
            rated_torque = 125  # Torque (mNm)
            return rated_torque

        elif Power == 77:  # MMS742077-24 Motor
            self.IQ_Nominal = 5200  # iq (mAmps)
            rated_torque = 185  # Torque (mNm)
            return rated_torque

        elif Power == 105:  # MMS7420105-24 Motor
            self.IQ_Nominal = 7200  # iq (mAmps)
            rated_torque = 250  # Torque (mNm)
            return rated_torque

        elif Power == 94:  # MMS757094-36 Motor
            self.IQ_Nominal = 4100  # iq (mAmps)
            rated_torque = 300  # Torque (mNm)
            return rated_torque

        elif Power == 141:  # MMS7570141-36 Motor
            self.IQ_Nominal = 6000  # iq (mAmps)
            rated_torque = 450  # Torque (mNm)
            return rated_torque

        elif Power == 188:  # MMS7570188-36 Motor
            self.IQ_Nominal = 2400  # 8000; iq (mAmps)
            rated_torque = 120  # 600; Torque (mNm)
            return rated_torque

    def check_crc(self, msgBuf1):
        """
        Check the CRC (Cyclic Redundancy Check) of the received message.

        This function separates the received CRC from the received data, calculates the CRC of the received data,
        converts the received CRC to an integer, and compares the calculated CRC with the received CRC.

        Args:
            msgBuf1 (bytes): The received message, which includes the data and the CRC.

        Returns:
            bool: True if the CRC check passed, False otherwise.
        """

        # Separate the received CRC from the received data
        received_crc = msgBuf1[-2:]
        data = msgBuf1[:-2]

        # Calculate the CRC of the received data
        calculated_crc = self.MODBUS_CRC(data)

        # Convert the received CRC to an integer
        received_crc_int = int.from_bytes(received_crc, byteorder="little")

        # Compare the calculated CRC with the received CRC
        if calculated_crc == received_crc_int:
            # print("CRC check passed")
            return True
        else:
            # print("CRC check failed")
            return False

    def reset_pos(self):
        """
        Reset the position of the device.
        """
        # self.Set_Position(0, 0, 0)
        self.writeRegSingle(0x7F, 0x686F)
        # time.sleep(1)
        # self.writeRegSingle(0x686F, 0x7F)
