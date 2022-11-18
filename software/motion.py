import math
import numpy as np
import struct
import serial
import serial.tools.list_ports


class SerialPortNotFound(Exception):
    """Raised when no mainboard serial port is found."""

    def __init__(self):
        super().__init__("Serial port not found")


class OmniRobot():
    """Movement code class"""

    def __init__(self):
        gearboxReductionRatio = 18.75
        encoderEdgesPerMotorRevolution = 64
        wheelRadius = 0.035  # metres
        pidControlFrequency = 100  # Hz
        # middle, left, right
        self.wheelAngles = np.radians([240, 0, 120])
        self.wheelSpeedToMainboardUnits = gearboxReductionRatio * \
            encoderEdgesPerMotorRevolution / \
            (2 * math.pi * wheelRadius * pidControlFrequency)
        self.wheelDistanceFromCenter = 0.11  # metres

    def find_serial_port(self):
        """Finds the serial port corresponding to the mainboard

        Raises:
            SerialPortNotFound: Mainboard serial port not found

        Returns:
            string: Serial port
        """
        serial_port = None
        ports = serial.tools.list_ports.comports()
        devices = {}
        STM_32_HWID = "USB VID:PID=0483:5740"
        for port, _, hwid in sorted(ports):
            devices[hwid] = port
        for hwid in devices.keys():
            if STM_32_HWID in hwid:
                serial_port = devices[hwid]
                break
        if serial_port is None:
            raise SerialPortNotFound
        return serial_port

    def open(self):
        """Opens the serial connection with the mainboard."""
        self.ser = serial.Serial()
        self.ser.port = self.find_serial_port()
        self.ser.baudrate = 9600
        self.ser.open()
        print("Serial opened")

    def close(self):
        """Closes the serial connection with the mainboard."""
        if self.ser.isOpen():
            self.stop()
            self.ser.close()
            print("Serial closed")
        else:
            print("Serial not open!")

    def send(self, speeds, throwerSpeed, disableFailsafe=0):
        """This function sends speeds to the mainboard.

        Args:
            speeds (list of ints): List of speeds to send (X, Y and rotational)
            throwerSpeed (int): Speed to use for the thrower
            disableFailsafe (int, optional): Enables continuous movement. Defaults to 0.
        """
        sent_data = struct.pack(
            '<hhhHBH', speeds[0], speeds[1], speeds[2], throwerSpeed, disableFailsafe, 0xAAAA)
        self.ser.write(sent_data)
        received_data = self.ser.read(8)
        actual_speed1, actual_speed2, actual_speed3, _ = struct.unpack(
            '<hhhH', received_data)
        #print(
        #    f"Sent speed: {speeds}, Actual speed: [{actual_speed1}, {actual_speed2}, {actual_speed3}]")

    def get_wheel_speed(self, motorID, robotSpeed, robotDirectionAngle, robotAngularVelocity):
        """Calculate wheel speed in mainboard units

        Args:
            motorID (int): ID of the current wheel
            robotSpeed (float): Robot speed in metres
            robotDirectionAngle (float): Direction angle of the robot
            robotAngularVelocity (float): Angular velocity of the robot

        Returns:
            int: Wheel angular speed in mainboard units
        """
        wheelLinearVelocity = robotSpeed * \
            math.cos(robotDirectionAngle -
                     self.wheelAngles[motorID]) + self.wheelDistanceFromCenter * robotAngularVelocity
        wheelAngularSpeedInMainboardUnits = wheelLinearVelocity * \
            self.wheelSpeedToMainboardUnits
        if math.isnan(wheelAngularSpeedInMainboardUnits):
            return 0
        else:
            return wheelAngularSpeedInMainboardUnits

    def move(self, x_speed, y_speed, rot_speed, thrower_speed=0, disableFailsafe=0):
        """Move the robot

        Args:
            x_speed (float): Speed on the X axis (metres)
            y_speed (float): Speed on the Y axis (metres)
            rot_speed (float): Rotational speed
            thrower_speed (int, optional): Speed to use for the thrower. Defaults to 0.
            disableFailsafe (int, optional): Enables continuous movement. Defaults to 0.
        """
        robot_speed = math.sqrt(x_speed**2 + y_speed**2)
        robot_angle = math.atan2(y_speed, x_speed)
        speeds = [int(self.get_wheel_speed(i, robot_speed,
                      robot_angle, rot_speed)) for i in range(3)]
        self.send(speeds, thrower_speed, disableFailsafe=disableFailsafe)

    def stop(self):
        """Stops all the motors."""
        self.move(0, 0, 0)

    def motor_test(self, speed=10, thrower=0):
        """Sends a fixed speed to all motors for testing purposes

        Args:
            speed (int, optional): Motor speed in mainboard units. Defaults to 10.
            thrower (int, optional): Thrower speed. Defaults to 0.
        """
        self.send([speed, speed, speed], thrower)
