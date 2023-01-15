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

    def __init__(self, logger, config):
        self.logger = logger
        self.config = config
        self.wheel_distance_from_center = self.config[
            "motor"]["wheel_distance_from_center"]
        self.wheel_angles = self.string_to_angles(
            self.config["motor"]["motor_order"])
        self.wheel_speed_to_mainboard_units = self.config["motor"]["gearbox_reduction_ratio"] * \
            self.config["motor"]["encoder_edges_per_motor_revolution"] / \
            (2 * math.pi * self.config["motor"]["wheel_radius"]
             * self.config["motor"]["pid_control_frequency"])

    def string_to_angles(self, angle_str):
        angles_list = list()
        for char in angle_str.lower().strip():
            if char == "c":
                angles_list.append(240)
            elif char == "l":
                angles_list.append(0)
            elif char == "r":
                angles_list.append(120)
            else:
                raise ValueError(f"Invalid wheel position {char}!")
        return np.radians(angles_list)

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
        self.logger.log.info("Serial opened")

    def close(self):
        """Closes the serial connection with the mainboard."""
        if self.ser.isOpen():
            self.stop()
            self.ser.close()
            self.logger.log.info("Serial closed")
        else:
            self.logger.log.warning("Serial not open!")

    def send(self, speeds, thrower_speed, disable_failsafe=0):
        """This function sends speeds to the mainboard.

        Args:
            speeds (list of ints): List of speeds to send (X, Y and rotational)
            thrower_speed (int): Speed to use for the thrower
            disable_failsafe (int, optional): Enables continuous movement. Defaults to 0.
        """
        sent_data = struct.pack(
            '<hhhHBH', speeds[0], speeds[1], speeds[2], thrower_speed, disable_failsafe, 0xAAAA)  # for old electronics
        self.ser.write(sent_data)
        received_data = self.ser.read(8)
        if self.config["logging"]["motor_speeds"]:
            actual_speed1, actual_speed2, actual_speed3, _ = struct.unpack(
                '<hhhH', received_data)
            self.logger.log.info(
                f"Sent speed: {speeds}, Actual speed: [{actual_speed1}, {actual_speed2}, {actual_speed3}]")

    def get_wheel_speed(self, motor_ID, robot_speed, robot_direction_angle, robot_angular_velocity):
        """Calculate wheel speed in mainboard units

        Args:
            motor_ID (int): ID of the current wheel
            robot_speed (float): Robot speed in metres
            robot_direction_angle (float): Direction angle of the robot
            robot_angular_velocity (float): Angular velocity of the robot

        Returns:
            int: Wheel angular speed in mainboard units
        """
        wheel_linear_velocity = robot_speed * \
            math.cos(robot_direction_angle -
                     self.wheel_angles[motor_ID]) + self.wheel_distance_from_center * robot_angular_velocity
        wheel_angular_speed_in_mainboard_units = wheel_linear_velocity * \
            self.wheel_speed_to_mainboard_units
        if math.isnan(wheel_angular_speed_in_mainboard_units):
            self.logger.log.warning("Tried to send NaN as speeds!")
            return 0
        else:
            return wheel_angular_speed_in_mainboard_units

    def move(self, x_speed, y_speed, rot_speed, thrower_speed=0, disable_failsafe=0):
        """Move the robot

        Args:
            x_speed (float): Speed on the X axis (metres)
            y_speed (float): Speed on the Y axis (metres)
            rot_speed (float): Rotational speed
            thrower_speed (int, optional): Speed to use for the thrower. Defaults to 0.
            disable_failsafe (int, optional): Enables continuous movement. Defaults to 0.
        """
        robot_speed = math.sqrt(x_speed**2 + y_speed**2)
        robot_angle = math.atan2(y_speed, x_speed)
        speeds = [int(self.get_wheel_speed(i, robot_speed,
                      robot_angle, rot_speed)) for i in range(3)]
        self.send(speeds, thrower_speed, disable_failsafe=disable_failsafe)

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


class FakeMotion(OmniRobot):
    """This exists for the purpose of being able to run the code without a functional robot."""

    def open(self):
        """FakeMotion - Opens the serial connection with the mainboard."""
        pass

    def close(self):
        """FakeMotion - Closes the serial connection with the mainboard."""
        pass

    def send(self, speeds, thrower_speed, disable_failsafe=0):
        """This function sends speeds to the mainboard, or it would, if this weren't FakeMotion.

        Args:
            speeds (list of ints): List of speeds to send (X, Y and rotational)
            thrower_speed (int): Speed to use for the thrower
            disable_failsafe (int, optional): Enables continuous movement. Defaults to 0.
        """
        pass
