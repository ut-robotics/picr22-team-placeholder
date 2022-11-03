import math
import numpy as np
import struct
import serial
import serial.tools.list_ports

class SerialPortNotFound(Exception): 
     def __init__(self): 
         super().__init__("Serial port not found") 

class OmniRobot():
    # initializee
    def __init__(self):
        gearboxReductionRatio = 18.75
        encoderEdgesPerMotorRevolution = 64
        wheelRadius = 0.035  # metres
        pidControlFrequency = 100  # Hz
        # middle, left, right
        self.wheelAngles = np.radians([0, 240, 120])
        self.wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution / (2 * math.pi* wheelRadius * pidControlFrequency)
        self.wheelDistanceFromCenter = 0.11 # metres

    def find_serial_port(self):
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

    # opening the serial connection
    def open(self):
        self.ser = serial.Serial()
        self.ser.port = self.find_serial_port()
        self.ser.baudrate = 9600
        self.ser.open()
        print("Serial opened")

    # closing serial connection
    def close(self):
        if self.ser.isOpen():
            self.stop()
            self.ser.close()
            print("Serial closed")
        else:
            print("Serial not open!")

    # writing and reading to and from serial

    def send(self, speeds, throwerSpeed, disableFailsafe=0, delimiter=0xAAAA):
        # Tere Timo ja Jens!!!!!!!!!!
        sent_data = struct.pack(
            '<hhhHBH', speeds[0], speeds[1], speeds[2], throwerSpeed, disableFailsafe, delimiter)
        self.ser.write(sent_data)
        received_data = self.ser.read(8)
        actual_speed1, actual_speed2, actual_speed3, _ = struct.unpack(
            '<hhhH', received_data)
        print(f"Sent speed: {speeds}, Actual speed: [{actual_speed1}, {actual_speed2}, {actual_speed3}]")
        # i hate python - Artur

    # wheel speed calculation
    def get_wheel_speed(self, motorID, robotSpeed, robotDirectionAngle, robotAngularVelocity):
        wheelLinearVelocity = robotSpeed * \
            math.cos(robotDirectionAngle -
                     self.wheelAngles[motorID]) + self.wheelDistanceFromCenter * robotAngularVelocity
        wheelAngularSpeedInMainboardUnits = wheelLinearVelocity * \
            self.wheelSpeedToMainboardUnits
        return wheelAngularSpeedInMainboardUnits

    # movement calculation + sending
    def move(self, x_speed, y_speed, rot_speed, thrower_speed=0, disableFailsafe=0):
        """speed is in metres"""
        robot_speed = math.sqrt(x_speed**2 + y_speed**2)
        robot_angle = math.atan2(y_speed, x_speed)  # verify
        speeds = [int(self.get_wheel_speed(i, robot_speed,
                      robot_angle, rot_speed)) for i in range(3)]
        self.send(speeds, thrower_speed, disableFailsafe=disableFailsafe)

    def stop(self):
        self.move(0, 0, 0)

    # for testing motors
    def motor_test(self, speed=10, thrower=0):
        self.send([speed, speed, speed], thrower)
